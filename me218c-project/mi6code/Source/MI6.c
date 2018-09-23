/****************************************************************************
 Module
	MI6.c

 Revision			Revised by: 
	1.0.0				Vikram
	1.0.1				Eric
	1.1.0				Eric
	1.2.0				Eric
	1.2.1				Eric
	1.3.0				Eric
	

 Description
	UART for TIVA asynchronous communication for Lab 10

 Edits:
	1.0.0				Established connection between Xbees
	1.0.1				Added checkSum function for error checking, added PWM and 
							ADC modules
	1.1.0				MI6 Xbee setup
	1.2.0				Added input functionality for Joystick and unpair buttons
							Added PIC UART (needs testing)
							General Cleanup
	1.2.1				Added communication with Watch PIC
							Added second interrupt for Pop command
	1.3.0				Fixed unpairing bug that results in failed next pairing attempt
							Fixed improper thrust and orientation control values; now they
								follow the protocol 
							Added third interrupt for brake command
							General Cleanup
							

****************************************************************************/


/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

// Module headers
#include "MI6.h"
#include "CommRxSM.h"
#include "UART.h"
#include "ADMulti.h"

// TivaWare Headers
#include "driverlib/sysctl.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble 			4
#define TicksPerMS 					40000				// 40Mhz
#define ALL_BITS 						(0xff<<2)
#define MY_ADDRESS					0x0C	//team 12

//receive defines:
#define API_ID							0
#define SOURCE_ADDR_MSB			1
#define	SOURCE_ADDR_LSB			2
#define	RSSI								3
#define RX_OPTIONS					4
#define RX_HEADER						5
//for Pair Ack
#define PAIR_ACK						6
//for Status
#define STATUS							6
#define	RX_CHKSUM						7

//transmit defines:
#define DEST_ADDR_MSB				5
#define DEST_ADDR_LSB				6
#define TX_OPTIONS					7
#define TX_HEADER						8
//for broadcast
#define REQ_PAIR						9
#define	BROAD_CHKSUM				10
//for control
#define THRUST							9
#define ORIENT							10
#define	ACTION							11
#define	EXA									12
#define	EXB									13
#define	EXC									14
#define	CTRL_CHKSUM					15
//header defines
#define BROADCAST						0x01
#define	SP_PAIR_ACK					0x02
#define CTRL								0x03	
#define SP_STATUS						0x04
//message status defines
#define	SUCCESSFUL					0x00
#define NACK								0x01
#define STATUS_ACK					2
//spectial action/status defines
#define MIND_CTRL						0x01
#define	POP									0x01
#define BRAKE								0x02
#define	UNPAIR							0x80
//watch Status values
#define	RED									0x01
#define BLUE								0x02
#define GREEN								0x04
//Tx arrays
#define TXBROAD							0
#define TXCTRL							1


/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.
*/
//interrupt functions
void UART1InterruptResponse( void );
void UART2InterruptResponse( void );
void PopInterruptResponse( void );
void BrakeInterruptResponse( void ); 
void FlaskInterruptResponse( void );
void SoftwareReset( void );

//comms related functions
static void importData(void);
uint8_t checkSum(uint8_t chkArray[], uint8_t size);
static void LoadUpPairingReq(uint8_t SpectreAddr);
static void LoadUpControl(void);
static void SendXbeeData(uint8_t);
//debugging functions
static void printRx(void);
static void printTx(void);
//initialization functions
static void InitInputCaptureWT1( void );
static void InitInputCaptureWT2( void );
static void InitInputCaptureWT3( void );
static void InitGPIO( void );

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static MI6State CurrentState;

static uint8_t SpectreAddrMSB = 0x00;
static uint8_t SpectreAddrLSB = 0x00;
static uint8_t ThrustVal = 0x00;
static signed int sThrust = 0x00; //temporary value to fit our retarded protocol
static uint8_t OrientVal = 0x00;
static signed int sOrient = 0x00; //temporary value to fit our retarded protocol
static uint8_t ActionVal = 0x00;
static uint8_t ExaVal = 0x00;
static uint8_t ExbVal = 0x00;
static uint8_t ExcVal = 0x00;
static uint8_t InitStatus = 0xFF; //initial mind control value to be rewritten upon pairing


static uint8_t TeamCorrection;		//correction for the potentiometer issue
static uint8_t *rx;
static uint8_t rxData[7];					//received data
static uint8_t txDataCTRL[16];		//control array
static uint8_t txDataBroad[11];		//broadcast array
static bool flag = false;
static bool popStatus = true;
static bool debounce = false;
static bool brakeStatus = false;
static int txIndex = 0;

static uint32_t ADResults[4];
static bool BAC = false;					//determines whether or not the user is drunk enough to drive
static uint8_t txData;						//control or broadcast placeholder
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMI6

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
****************************************************************************/

bool InitMI6 ( uint8_t Priority )
{
  ES_Event ThisEvent;
  
  MyPriority = Priority;
	InitUART();		  				// Initialize UART
	InitInputCaptureWT1();	// Initialize Input Capture
	InitInputCaptureWT2();
	InitInputCaptureWT3();
	InitGPIO();	
	ADC_MultiInit(4);						// Initialize 4 Analog ports E0-E3
	CurrentState = IdleState;

  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService( MyPriority, ThisEvent) == true)
  {
      return true;
  }else
  {
      return false;
  }
}
/****************************************************************************
 Function
     PostMI6

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostMI6( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMI6

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here

****************************************************************************/
ES_Event RunMI6 ( ES_Event CurrentEvent )
{
	ES_Event ReturnEvent = {ES_NO_EVENT};
	MI6State NextState = CurrentState;
	
	switch(CurrentState) {
		case IdleState :
			if(CurrentEvent.EventType == ES_INIT) {
				ES_Timer_InitTimer(WatchInitTimer,500);
				//Timer to let the Watch start up before we send it shit
			}
			//the PIC says it's time to start pairing
			else if(CurrentEvent.EventType == BeginPairing) {
				//prepare the pairing request
				TeamCorrection = (CurrentEvent.EventParam + 4)%12 + 1; //correcting for pot issue
				LoadUpPairingReq(TeamCorrection); //prepare the array for sending
				SendXbeeData(TXBROAD);		//send the pair request
				NextState = RequestPairingState;
				ES_Timer_InitTimer(CommTimer,1000); //start a comm timer for lost signal
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT) {
				if(CurrentEvent.EventParam == PopTimer) {
					popStatus = true; //allow for a pop attempt to happen again
				}else if(CurrentEvent.EventParam == WatchInitTimer) {
					HWREG(UART5_BASE + UART_O_DR) = GREEN; 	//turn green LED (NEUTRAL)
				}else if(CurrentEvent.EventParam == SobrietyTimer) {
					BAC = false; //allow for controls to be sent
				}else if(CurrentEvent.EventParam == debounceTimer) {
					debounce = false; //allow for pairing request to be sent
				}
			}
			break;
	
			
		case RequestPairingState :
			if(CurrentEvent.EventType == PackageReceived) {
				importData();			//import the data for useability
				//check to see if it's the tx status
				if(rxData[API_ID] == 0x89) {
					break; //just ignore it
				}
				//check to see if the message is what we were looking for
				if(rxData[RX_HEADER] == 0x02 && rxData[PAIR_ACK] == 0x01) {
					//set the master's address
					SpectreAddrMSB = rxData[SOURCE_ADDR_MSB];
					SpectreAddrLSB = rxData[SOURCE_ADDR_LSB];
					LoadUpControl();				//prepare control data to be sent
					SendXbeeData(TXCTRL);		//send control data
					NextState = MasterState;
					//reset comm timer and start the pulse timer
					ES_Timer_InitTimer(CommTimer,1000);
					ES_Timer_InitTimer(PulseTimer,200);
				}
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT) {
				if(CurrentEvent.EventParam == CommTimer) {
					NextState = IdleState; //go back to idle
				}
				else if(CurrentEvent.EventParam == PopTimer) {
					popStatus = true;
				}else if(CurrentEvent.EventParam == debounceTimer) {
					debounce = false;
				}
			else if(CurrentEvent.EventType == Unpair) {
				NextState = IdleState;
			}
			break;
		}
			
		case MasterState :
			if(CurrentEvent.EventType == PackageReceived) {
				//import the data for useability
				importData();
				if(rxData[API_ID] == 0x89) {
					//check the receive bit
					if(rxData[STATUS_ACK] == NACK) { //failed transmission
						//try sending data again by sending a pulse timeout 
						ES_Event retryEvent;
						retryEvent.EventType = ES_TIMEOUT;
						retryEvent.EventParam = PulseTimer; 
						PostMI6(retryEvent);
						ES_Timer_InitTimer(PulseTimer,200); //reset timer
					}else if(rxData[STATUS_ACK] == SUCCESSFUL) {
						//reset the action byte
						ActionVal = 0x00;
					}else {
						//well somethings fucked up
						ActionVal = 0x00;
						printf("Other Error\r\n");
					}
					break;
				}

				//check to see if the message is what we were looking for
				if(rxData[RX_HEADER] != SP_STATUS) {
					break; //incorrect header
				}
				//set the beginning status state to compare to for balloon pop
				if(InitStatus == 0xFF) {
					InitStatus = rxData[STATUS];
					//check the balloon status to see if we're heroes or villians
					if(rxData[STATUS] == MIND_CTRL) {
						HWREG(UART5_BASE + UART_O_DR) = RED; //turn on mind controlled LED
					}else {
						HWREG(UART5_BASE + UART_O_DR) = BLUE; //we're free
					}
				}
				
				//check for a balloon pop
				if(rxData[STATUS] != InitStatus) {
					NextState = IdleState;
					InitStatus = 0xFF;			//go back to reset status
					//update paired indicator
					HWREG(UART5_BASE + UART_O_DR) = GREEN; //back to neutral
					ES_Timer_InitTimer(SobrietyTimer,5000); //set the sobriety timer
				}
				//re-initialize timer
				ES_Timer_InitTimer(CommTimer,1000);
			}
			//check the timeouts
			else if(CurrentEvent.EventType == ES_TIMEOUT) {
				if(CurrentEvent.EventParam == CommTimer) {
					//update paired indicator
					HWREG(UART5_BASE + UART_O_DR) = GREEN; //back to neutral
					NextState = IdleState;
					InitStatus = 0xFF;			//go back to reset status
					ES_Timer_InitTimer(SobrietyTimer,5000);
				}
				else if(CurrentEvent.EventParam == PulseTimer) {
					//see if we are drunk enough
					if(BAC) {
						//first the joystick
						ADC_MultiRead(ADResults);
						ThrustVal = (ADResults[0] >> 4);	//just want 8 bit resolution
						sThrust = ThrustVal - 128;
						//convert it to signed
						ThrustVal = (uint8_t) sThrust;	//then back to uint because fuck the protocol
						OrientVal = (ADResults[1] >> 4);	//same thing for orient
						sOrient = OrientVal - 128;
						OrientVal = (uint8_t) sOrient;
					}else {
						//you're too sober to pull this off
						ThrustVal = 0x00;
						OrientVal = 0x00;
					}
					//check brake
					if(HWREG(GPIO_PORTD_BASE + (GPIO_O_DATA + ALL_BITS)) & BIT0HI ) {
						ActionVal |= BRAKE; //flip the brake bit in the action byte
					}
					// send a control package
					LoadUpControl();
					SendXbeeData(TXCTRL);
					ES_Timer_InitTimer(PulseTimer,200); //restart timer
				}
				else if(CurrentEvent.EventParam == PopTimer) {
					popStatus = true;
				}else if(CurrentEvent.EventParam == debounceTimer) {
					debounce = false;
				}

			}
			else if(CurrentEvent.EventType == Pop) {
				// check the x (blue, PE2, ADC[2]) and z (green PE3, ADC[3]) values for idiocy
				// x = 1350-2650; z = 1500-2500
				ADC_MultiRead(ADResults);
				if (ADResults[2] < 1350) {
					printf("Hold the gun upright dumbass\r\n");
				}
				else if (ADResults[2] > 2650) {
					printf("How can that be comfortable? You look like a fool.\r\n");
				}
				else if (ADResults[3] < 1500) {
					printf("What are you shooting at? A blimp?\r\n");
					
				}
				else if (ADResults[3] > 2650) {
					printf("Do you want to get shot in the foot? Because that's how you get shot in the foot.\r\n");
				}
				else {
					printf("good shot\r\n");
					ActionVal |= POP; //flip the pop attempt bit
				}
			}
				
			else if(CurrentEvent.EventType == Unpair) {
				//send ctrl package one last time
				ActionVal |= 0x80; //flip the unpair bit
				LoadUpControl();
				SendXbeeData(TXCTRL);
				NextState = IdleState;
				InitStatus = 0xFF;			//go back to reset status
				// update paired indicator on the watch
				HWREG(UART5_BASE + UART_O_DR) = GREEN; //back to neutral
				ActionVal &= 0x7F; 			//clear the unpair bit
				ES_Timer_InitTimer(SobrietyTimer,5000);
			}
			break;
	}
	
	CurrentState = NextState;
  return ReturnEvent;
}



/****************************************************************************
 Function
   UART1InterruptResponse

 Description
	Rx and Tx interrupt response for UART 1 communcation with the Xbee
****************************************************************************/
void UART1InterruptResponse( void ) 
{
	// If the interrupt response is from Tx FIFO open
	if(((HWREG(UART1_BASE+UART_O_RIS)& UART_RIS_TXRIS) >0)||flag){
		//Clear the flag
		HWREG(UART1_BASE + UART_O_ICR) |=UART_ICR_TXIC;
		//Send the appropriate data, one byte at a time, incrementing the array each time
		if(txData == TXBROAD) { 
			if((txIndex%sizeof(txDataBroad)>0)||flag){
				flag = false;
				HWREG(UART1_BASE + UART_O_DR) = txDataBroad[txIndex%sizeof(txDataBroad)];
				txIndex++;
			}
		}else if(txData == TXCTRL) {
			if((txIndex%sizeof(txDataCTRL)>0)||flag){
				flag = false;
				HWREG(UART1_BASE + UART_O_DR) = txDataCTRL[txIndex%sizeof(txDataCTRL)];
				txIndex++;
			}
		}
	}
	
	// If the interrupt response is from Rx (new data)
	if((HWREG(UART1_BASE+UART_O_RIS)& UART_RIS_RXRIS) >0){
		// Clear the source of the interrupt
		HWREG(UART1_BASE + UART_O_ICR) |= UART_ICR_RXIC;
		//post byte received to the CommRxSM
		ES_Event ThisEvent;
		ThisEvent.EventType = ByteReceived;
		ThisEvent.EventParam = HWREG(UART1_BASE + UART_O_DR);
		PostCommRxSM(ThisEvent);
	}
}


/****************************************************************************
 Function
   UART2InterruptResponse

 Description
	Rx interrupt response for UART 2 communcation with the PIC
****************************************************************************/
void UART2InterruptResponse( void ) 
{

	if(((HWREG(UART5_BASE+UART_O_RIS)& UART_RIS_TXRIS) >0)||flag){
			//Clear the flag
			HWREG(UART5_BASE + UART_O_ICR) |=UART_ICR_TXIC;
	}
	
	// If the interrupt response is from Rx (new data)
	if((HWREG(UART5_BASE+UART_O_RIS)& UART_RIS_RXRIS) >0){
		// Clear the source of the interrupt
		HWREG(UART5_BASE + UART_O_ICR) |= UART_ICR_RXIC;
		uint8_t data = HWREG(UART5_BASE + UART_O_DR);
		// Let ComRXSM let us know that we received a byte if enough time
		// has passed between attempts
		if(debounce == false) {
			if(CurrentState == IdleState) {
				ES_Event GotByte;
				GotByte.EventType = BeginPairing;
				GotByte.EventParam = data;
				PostMI6(GotByte);
			}
			else if(CurrentState == MasterState) {
				ES_Event UnpairEvent;
				UnpairEvent.EventType = Unpair;
				PostMI6(UnpairEvent);
			}
			ES_Timer_InitTimer(debounceTimer,500);
			debounce = true;
		}
	}
}


/****************************************************************************
 Function
   PopInterruptResponse

 Description
	Interrupt response for an unpair button press
****************************************************************************/
void PopInterruptResponse( void ) 
{
	// Clear the source of the interrupt
	HWREG(WTIMER1_BASE + TIMER_O_ICR) = TIMER_ICR_CBECINT;
	//post a pop event to the Master
	if(popStatus == true) {
		popStatus = false;
		ES_Timer_InitTimer(PopTimer,400); //400ms timer 
		ES_Event PopEvent;
		PopEvent.EventType = Pop;
		PostMI6(PopEvent);
	}
	
}

/****************************************************************************
 Function
   BrakeInterruptResponse

 Description
	Interrupt response for an unpair button press
****************************************************************************/
void BrakeInterruptResponse( void ) 
{
	// Clear the source of the interrupt
	HWREG(WTIMER2_BASE + TIMER_O_ICR) = TIMER_ICR_CAECINT;
	brakeStatus	= !brakeStatus; //flip the value
}

void FlaskInterruptResponse( void) 
{
	// clear the source of the interrupt
	HWREG(WTIMER2_BASE + TIMER_O_ICR) = TIMER_ICR_CBECINT;
	BAC = true; //allow for controls
}


/****************************************************************************
 Function
   importData

 Description
	imports data from the data acquired in SpectreXbee.c into an array for 
	easier accessibility
****************************************************************************/
static void importData(void) {
	rx = GetRxData();
	for(uint8_t i = 0; i < sizeof(rxData); i++) {
		rxData[i] = *(rx + i);
	}
}


/****************************************************************************
 Function
   checkSum

 Parameters
   chkArray, size (needed as the array decays into a point when passed into
							a function, i.e. we can't call sizeof() 

 Returns
   hex value

 Description
	calculates the checkSum for error checking
****************************************************************************/
uint8_t checkSum(uint8_t chkArray[], uint8_t size) {
	uint8_t sum = 0;
	for(int i = 3; i < size-1; i++) {
		sum += chkArray[i];
	}
	return 0xFF - sum;
}


/****************************************************************************
 Function
   LoadUpPairingReq

 Description
	Loads the pairing requrest array for transmission
****************************************************************************/
static void LoadUpPairingReq(uint8_t SpectreAddr) {
	txDataBroad[0] = 0x7E;
	txDataBroad[1] = 0x00;
	txDataBroad[2] = 0x07;
	txDataBroad[3] = 0x01;
	txDataBroad[4] = 0x52;
	txDataBroad[DEST_ADDR_MSB] = 0xFF; //Set address to broadcast
	txDataBroad[DEST_ADDR_LSB] = 0xFF; 
	txDataBroad[TX_OPTIONS] = 0x04; //set options to be broadcast
	txDataBroad[TX_HEADER] = BROADCAST; //set the header to be pairing
	txDataBroad[REQ_PAIR] = SpectreAddr; //set req_pair address
	txDataBroad[BROAD_CHKSUM] = checkSum(txDataBroad,11); //calculate the checksum
}


/****************************************************************************
 Function
   LoadUpControl

 Description
	Loads to control array for transmission
****************************************************************************/
static void LoadUpControl(void) {
	txDataCTRL[0] = 0x7E;
	txDataCTRL[1] = 0x00;
	txDataCTRL[2] = 0x0C;
	txDataCTRL[3] = 0x01;
	txDataCTRL[4] = 0x52;
	txDataCTRL[DEST_ADDR_MSB] = SpectreAddrMSB; //set destination address msb
	txDataCTRL[DEST_ADDR_LSB] = SpectreAddrLSB; //set destination address lsb
	txDataCTRL[TX_OPTIONS] = 0x00; //set options to not be broadcast
	txDataCTRL[TX_HEADER] = CTRL; //set the header to be pair acknowledge
	txDataCTRL[THRUST] = ThrustVal; //set the thrust values
	txDataCTRL[ORIENT] = OrientVal;
	txDataCTRL[ACTION] = ActionVal;
	txDataCTRL[EXA] = ExaVal;
	txDataCTRL[EXB] = ExbVal;
	txDataCTRL[EXC] = ExcVal;
	txDataCTRL[CTRL_CHKSUM] = checkSum(txDataCTRL,15); //calculate the checksum
}


/****************************************************************************
 Function
   SendXbeeData

 Description
	Interrupt response for an unpair button press
****************************************************************************/
static void SendXbeeData( uint8_t d ) 
{
	txData = d;
	txIndex = 0;
	flag = true;
	UART1InterruptResponse();
}

/****************************************************************************
 Function
   InitInputCaptureWT2

 Parameters
   none

 Returns
   none

 Description
	 Initialize Input Capture on C6 (Wide timer 1/PWM1-WT1CCP0)
****************************************************************************/
static void InitInputCaptureWT2( void )
	{
		/*
	Steps to initialize Tiva Wide Timer Input Capture
	1.	Enable the clock to wide timer 1 using RCGCWTIMER_R1 register
	2.	Enable the clock to the appropriate GPIO module RCGCGPIO
	3.	Disable wtimer 1 timer A to configure with TAEN in TIMER_O_CTL register
	4.	Set timer to 32bit wide mode with TIMER_CFG_16_BIT in TIMER_O_CTL register
	5.	Set timer to use full 32bit count by setting TIMER_O_TAILR to 0xffffffff
	6.	Set timer A to capture mode for edge time and up-counting (Clear TAMMS and Set TACMR,TACDIR,TAMR_CAP in TIMERTAMR)
	7.	Set event to rising edge by clearing TAEVENT_M in TIMER_O_CTL (Rising edge = 00)
	8.	Select the alternate function for the Timer pins (AFSEL)
	9.	Configure the PMCn fields in the GPIOPCTL register to assign the WTIMER pins (WT1CCP0)
	10.	Enable appropriate pint on GPIO port for digital I/O (GPIODIR)
	11.	Set appropriate pins on GPIO as inputs (GPIODEN)
	12. Locally enable interrupts for the capture interrupt mask (CAEIM in TIMERIMR)
	13. Enable WTIMER1 Timer A interrupt vector in NVIC (96 = Bit 0 in NVIC_EN3) (Tiva p.104 for vector table)
	14. Ensure interrupts are enabled globally (__enable_irq())
	14.	Enable WTIMER1 Timer A with TAEN in TIMER_O_CTL register
	
	WTIMER1 Timer A - Tiva PC6
	*/
		
		
  // Enable the clock to the timer (Wide Timer 2)
  HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R2;
		
	// Enable the clock to Port D
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
  
  // Disable timer A & B before configuring
  HWREG(WTIMER2_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
	
  // Set up in 32bit wide (individual, not concatenated) mode
  HWREG(WTIMER2_BASE + TIMER_O_CFG) = TIMER_CFG_16_BIT;

  // Use the full 32 bit count, so initialize the Interval Load
  // register to 0xffff.ffff
  HWREG(WTIMER2_BASE + TIMER_O_TAILR) = 0xffffffff;
	HWREG(WTIMER2_BASE + TIMER_O_TBILR) = 0xffffffff;
	
  // set up timer A in capture mode (TAAMS = 0), 
  // for edge time (TACMR = 1) and up-counting (TACDIR = 1)
  HWREG(WTIMER2_BASE + TIMER_O_TAMR) = 
      (HWREG(WTIMER2_BASE + TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) | 
        (TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);
	HWREG(WTIMER2_BASE + TIMER_O_TBMR) = 
      (HWREG(WTIMER2_BASE + TIMER_O_TBMR) & ~TIMER_TBMR_TBAMS) | 
        (TIMER_TBMR_TBCDIR | TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP);

  // To set the event to rising edge, we need to modify the TAEVENT bits 
  // in GPTMCTL. Rising edge = 00, so we clear the TAEVENT bits
  HWREG(WTIMER2_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEVENT_M | TIMER_CTL_TBEVENT_M);

  // Set up the alternate function for Port C bit 6 (WT1CCP0)
  HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) |= (BIT0HI | BIT1HI);

	// Set mux position on GPIOPCTL to select WT1CCP0 alternate function on C6
	// Mux value = 7 offset mask to clear nibble 6
	HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) = 
    (HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) & ~0x0f000000) + (7 << ( 0 * BitsPerNibble)) +
																													(7 << ( 1 * BitsPerNibble));
  // Enable pin 6 on Port C for digital I/O
  HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= (BIT0HI | BIT1HI);
	
  // Make pin 6 on Port C into an input
  HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) &= (BIT0LO & BIT1LO);

  // Enable a local capture interrupt
  HWREG(WTIMER2_BASE + TIMER_O_IMR) |= (TIMER_IMR_CAEIM | TIMER_IMR_CBEIM);

  // Enable Timer A Wide Timer 1 interrupt in the NVIC
  // NVIC number 96 (EN3 bit 0) Tiva p.104
  HWREG(NVIC_EN3) |= (BIT2HI | BIT3HI);

  // Make sure interrupts are enabled globally
  __enable_irq();

  // Enable Timer
  HWREG(WTIMER2_BASE + TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL |
																				TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
	
	// Print to console if successful initialization
	printf("Wide Timer 5 A interrupt initialized\n\r");
}


/****************************************************************************
 Function
   InitInputCaptureWT1

 Parameters
   none

 Returns
   none

 Description
	 Initialize Input Capture on C6 (Wide timer 1/PWM1-WT1CCP0)
****************************************************************************/
static void InitInputCaptureWT1( void )
	{
		/*
	Steps to initialize Tiva Wide Timer Input Capture
	1.	Enable the clock to wide timer 1 using RCGCWTIMER_R1 register
	2.	Enable the clock to the appropriate GPIO module RCGCGPIO
	3.	Disable wtimer 1 timer A to configure with TAEN in TIMER_O_CTL register
	4.	Set timer to 32bit wide mode with TIMER_CFG_16_BIT in TIMER_O_CTL register
	5.	Set timer to use full 32bit count by setting TIMER_O_TAILR to 0xffffffff
	6.	Set timer A to capture mode for edge time and up-counting (Clear TAMMS and Set TACMR,TACDIR,TAMR_CAP in TIMERTAMR)
	7.	Set event to rising edge by clearing TAEVENT_M in TIMER_O_CTL (Rising edge = 00)
	8.	Select the alternate function for the Timer pins (AFSEL)
	9.	Configure the PMCn fields in the GPIOPCTL register to assign the WTIMER pins (WT1CCP0)
	10.	Enable appropriate pint on GPIO port for digital I/O (GPIODIR)
	11.	Set appropriate pins on GPIO as inputs (GPIODEN)
	12. Locally enable interrupts for the capture interrupt mask (CAEIM in TIMERIMR)
	13. Enable WTIMER1 Timer A interrupt vector in NVIC (96 = Bit 0 in NVIC_EN3) (Tiva p.104 for vector table)
	14. Ensure interrupts are enabled globally (__enable_irq())
	14.	Enable WTIMER1 Timer A with TAEN in TIMER_O_CTL register
	
	WTIMER1 Timer A - Tiva PC6
	*/
		
		
  // Enable the clock to the timer (Wide Timer 1)
  HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R1;
		
	// Enable the clock to Port C
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
  
  // Disable timer A before configuring
  HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
	
  // Set up in 32bit wide (individual, not concatenated) mode
  HWREG(WTIMER1_BASE + TIMER_O_CFG) = TIMER_CFG_16_BIT;

  // Use the full 32 bit count, so initialize the Interval Load
  // register to 0xffff.ffff
  HWREG(WTIMER1_BASE + TIMER_O_TAILR) = 0xffffffff;
	HWREG(WTIMER1_BASE + TIMER_O_TBILR) = 0xffffffff;
	
  // set up timer A in capture mode (TAAMS = 0), 
  // for edge time (TACMR = 1) and up-counting (TACDIR = 1)
  HWREG(WTIMER1_BASE + TIMER_O_TAMR) = 
      (HWREG(WTIMER1_BASE + TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) | 
        (TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);

	HWREG(WTIMER1_BASE + TIMER_O_TBMR) = 
      (HWREG(WTIMER1_BASE + TIMER_O_TBMR) & ~TIMER_TBMR_TBAMS) | 
        (TIMER_TBMR_TBCDIR | TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP);
				
  // To set the event to rising edge, we need to modify the TAEVENT bits 
  // in GPTMCTL. Rising edge = 00, so we clear the TAEVENT bits
  HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEVENT_M | TIMER_CTL_TBEVENT_M);

  // Set up the alternate function for Port C bit 6 (WT1CCP0)
  HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= (BIT6HI | BIT7HI);

	// Set mux position on GPIOPCTL to select WT1CCP0 alternate function on C6
	// Mux value = 7 offset mask to clear nibble 6
	HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) = 
    (HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) & ~0x0f000000) + (7 << ( 6 * BitsPerNibble)) + 
																													 (7 << ( 7 * BitsPerNibble));
			
  // Enable pin 6 on Port C for digital I/O
  HWREG(GPIO_PORTC_BASE + GPIO_O_DEN) |= (BIT6HI | BIT7HI);
	
  // Make pin 6 on Port C into an input
  HWREG(GPIO_PORTC_BASE + GPIO_O_DIR) &= (BIT6LO & BIT7LO);

  // Enable a local capture interrupt
  HWREG(WTIMER1_BASE + TIMER_O_IMR) |= (TIMER_IMR_CAEIM | TIMER_IMR_CBEIM);

  // Enable Timer A Wide Timer 1 interrupt in the NVIC
  // NVIC number 96 (EN3 bit 0) Tiva p.104
  HWREG(NVIC_EN3) |= (BIT0HI | BIT1HI);

  // Make sure interrupts are enabled globally
  __enable_irq();

  // Enable Timer
  HWREG(WTIMER1_BASE + TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL | 
																				TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
	
	// Print to console if successful initialization
	printf("Wide Timer 1 A&B interrupt initialized\n\r");
}
/****************************************************************************
 Function
   InitInputCaptureWT3

 Parameters
   none

 Returns
   none

 Description
	 Initialize Input Capture on C6 (Wide timer 1/PWM1-WT1CCP0)
****************************************************************************/
static void InitInputCaptureWT3( void )
	{
		/*
	Steps to initialize Tiva Wide Timer Input Capture
	1.	Enable the clock to wide timer 3 using RCGCWTIMER_R3 register
	2.	Enable the clock to the appropriate GPIO module RCGCGPIO
	3.	Disable wtimer 3 timer A and B to configure with TAEN/TBEN in TIMER_O_CTL register
	4.	Set timer to 32bit wide mode with TIMER_CFG_16_BIT in TIMER_O_CTL register
	5.	Set timer to use full 32bit count by setting TIMER_O_TAILR to 0xffffffff
	6.	Set timer B to capture mode for edge time and up-counting (Clear TAMMS and Set TACMR,TACDIR,TAMR_CAP in TIMERTAMR)
	7.	Set event to rising edge by clearing TAEVENT_M/TBEVENT_M in TIMER_O_CTL (Rising edge = 00)
	8.	Select the alternate function for the Timer pins (AFSEL)
	9.	Configure the PMCn fields in the GPIOPCTL register to assign the WTIMER pins (WT3CCP0/WT3CCP1)
	10.	Enable appropriate pint on GPIO port for digital I/O (GPIODIR)
	11.	Set appropriate pins on GPIO as inputs (GPIODEN)
	12. Locally enable interrupts for the capture interrupt mask (CAEIM in TIMERIMR)
	13. Enable WTIMER3 Timer A and B interrupt vector in NVIC (96 = Bit 0 in NVIC_EN3) (Tiva p.104 for vector table)
	14. Ensure interrupts are enabled globally (__enable_irq())
	14.	Enable WTIMER3 Timer A and B with TAEN in TIMER_O_CTL register
	
	WTIMER3 Timer A, B - Tiva PD2, PD3
	*/
		
		
  // Enable the clock to the timer (Wide Timer 3)
  HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R3;
		
	// Enable the clock to Port D
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
  
  // Disable timers A and B before configuring
  HWREG(WTIMER3_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
	
  // Set up in 32bit wide (individual, not concatenated) mode
  HWREG(WTIMER3_BASE + TIMER_O_CFG) = TIMER_CFG_16_BIT;

  // Use the full 32 bit count, so initialize the Interval Load
  // register to 0xffff.ffff
  HWREG(WTIMER3_BASE + TIMER_O_TAILR) = 0xffffffff;
	HWREG(WTIMER3_BASE + TIMER_O_TBILR) = 0xffffffff;
	
  // set up timer A and B in capture mode (TAAMS = 0), 
  HWREG(WTIMER3_BASE + TIMER_O_TAMR) = 
      (HWREG(WTIMER3_BASE + TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) | 
        (TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);

	HWREG(WTIMER3_BASE + TIMER_O_TBMR) = 
      (HWREG(WTIMER3_BASE + TIMER_O_TBMR) & ~TIMER_TBMR_TBAMS) | 
        (TIMER_TBMR_TBCDIR | TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP);
				
  // To set the event to rising edge, we need to modify the TAEVENT bits 
  // in GPTMCTL. Rising edge = 00, so we clear the TAEVENT bits
  HWREG(WTIMER3_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEVENT_M | TIMER_CTL_TBEVENT_M);

  // Set up the alternate function for D6,7
  HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) |= (BIT2HI | BIT3HI);

	// Set mux position on GPIOPCTL to select alternate function on D6 and D7
	// Mux value = 7 offset mask to clear nibble 6
	HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) = 
    (HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) & ~0x0000ff00) + (7 << ( 2 * BitsPerNibble)) + 
																													 (7 << ( 3 * BitsPerNibble));
			
  // Enable pins 2 and 3 on Port D for digital I/O
  HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= (BIT2HI | BIT3HI);
	
  // Make pins 2 and 3 on Port D into an input
  HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) &= (BIT2LO & BIT3LO);

  // Enable a local capture interrupt
  HWREG(WTIMER3_BASE + TIMER_O_IMR) |= (TIMER_IMR_CAEIM | TIMER_IMR_CBEIM);

  // Enable Timer A Wide Timer 1 interrupt in the NVIC
  // NVIC number 100,101 (EN3 bits 4,5) Tiva p.104
  HWREG(NVIC_EN3) |= (BIT4HI | BIT5HI);

  // Make sure interrupts are enabled globally
  __enable_irq();

  // Enable Timer
  HWREG(WTIMER3_BASE + TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL | 
																				TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
	
	// Print to console if successful initialization
	printf("Wide Timer 3 A&B interrupt initialized\n\r");
}


/****************************************************************************
 Function
   InitGPIO

 Description
	 Initialize GPIO pins for inputs/outputs
****************************************************************************/
static void InitGPIO( void )
{
	
	// Enable Port B to the timer
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1; 
	
	// Wait for GPIO Port B to be ready
	while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1 ) != SYSCTL_PRGPIO_R1 )
		;
	
	// Set Pin B7 as digital
	HWREG(GPIO_PORTB_BASE + GPIO_O_DEN) |= BIT7HI;
	// Set Pin B7 as output
	HWREG(GPIO_PORTB_BASE + GPIO_O_DIR) |= BIT7HI;
	// Make sure pin B7 is off to start
	HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) |= BIT7HI;
	
	// Print to console if successful initialization
	printf("GPIO Initialized\n\r");
	
}
/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
