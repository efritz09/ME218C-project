/****************************************************************************
 Module
   PWM.c

	Revision			Revised by: 
	1.0.0					Eric Fritz				

 Description
   PWM service to initialize the Tiva's hardware PWM output for general use
	 
	Edits:
	1.0.0 - Initial start with basic functionality
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "PWM.h"

/*----------------------------- Module Defines ----------------------------*/
// 40,000 ticks per mS assumes a 40Mhz clock, we will use SysClk/32 for PWM

// Set PWMTicksPerMicroS as /32000 for microseconds
#define BitsPerNibble 			4
#define ALL_BITS 						(0xff<<2)
#define ONE_SEC 						976
#define PWMTicksPerMicroS 	40000/32000
#define PeriodInMicroS 			333
#define ServoPeriod 				20000 // standard 20 ms
#define ONE_MS 							1000
#define TWO_MS 							2000


/*---------------------------- Module Functions ---------------------------*/

void InitPWM(void);
void SetPWMDuty(uint8_t duty, int channel);
void SetPWMWidth(uint32_t width, int channel);

void SetLastPWM(uint8_t lastPWM, int channel);
uint8_t GetLastPWM(int channel);

void SetLastDirection(int dir, int channel);
int GetLastDirection(int channel);

/*---------------------------- Module Variables ---------------------------*/
uint8_t lastPWM_Port = 0;
uint8_t lastPWM_Starboard = 0;
bool dirPort = false;
bool dirStar = false;

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitPWM

 Parameters
     none

 Returns
     none

 Description
		Initializes TIVA pins B4-7 as PWM

****************************************************************************/
void InitPWM( void )
{
	// Start by enabling the clock to the PWM Module (PWM0 & PWM1)
  HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R0;
	
	// Enable the clock to Port B
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;
	
	// Select the PWM clock as System Clock/32
  HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) |
    (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32);
  
	// Make sure that the PWM module clock has gotten going
	while ((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R0) != SYSCTL_PRPWM_R0)
    ;

	// Disable the PWM while initializing
  HWREG( PWM0_BASE+PWM_O_0_CTL ) = 0; //PB6 & PB7
  HWREG( PWM0_BASE+PWM_O_1_CTL ) = 0; //PB4 & PB5
	
	// Program generator A to go to 0 at rising comare A, 1 on falling compare A  
  HWREG( PWM0_BASE+PWM_O_0_GENA) = 
    (PWM_0_GENA_ACTCMPAU_ZERO | PWM_0_GENA_ACTCMPAD_ONE );
  HWREG( PWM0_BASE+PWM_O_1_GENA) = 
    (PWM_1_GENA_ACTCMPAU_ZERO | PWM_1_GENA_ACTCMPAD_ONE );

	// Program generator B to go to 0 at rising comare B, 1 on falling compare B  
  HWREG( PWM0_BASE+PWM_O_0_GENB) = 
    (PWM_0_GENA_ACTCMPBU_ZERO | PWM_0_GENA_ACTCMPBD_ONE );
  HWREG( PWM0_BASE+PWM_O_1_GENB) = 
    (PWM_1_GENA_ACTCMPBU_ZERO | PWM_1_GENA_ACTCMPBD_ONE );

	// Set the PWM period. Since we are counting both up & down, we initialize
	// the load register to 1/2 the desired total period
  HWREG( PWM0_BASE+PWM_O_0_LOAD) = ((PeriodInMicroS * PWMTicksPerMicroS)-1)>>1;
	// Servo Motor period is set to 20ms
  HWREG( PWM0_BASE+PWM_O_1_LOAD) = ((ServoPeriod * PWMTicksPerMicroS)-1)>>1;
	
	// Set the initial Duty cycle on A to 50% by programming the compare value
	// to 1/2 the period to count up (or down) 
  HWREG( PWM0_BASE+PWM_O_0_CMPA) = ((PeriodInMicroS * PWMTicksPerMicroS)-1)>>2;
  HWREG( PWM0_BASE+PWM_O_1_CMPA) = ((ONE_MS * PWMTicksPerMicroS)-1)>>2;

	// Set the initial Duty cycle on B to 25% by programming the compare value
	// to 1/4 the period  
  HWREG( PWM0_BASE+PWM_O_0_CMPB) = ((PeriodInMicroS * PWMTicksPerMicroS)-1)>>3;
  HWREG( PWM0_BASE+PWM_O_1_CMPB) = ((TWO_MS * PWMTicksPerMicroS)-1)>>3;
	
	// Set changes to the PWM output Enables to be locally syncronized to a 
	// zero count
  HWREG(PWM0_BASE+PWM_O_ENUPD) =  (HWREG(PWM0_BASE+PWM_O_ENUPD) & 
      ~(PWM_ENUPD_ENUPD0_M | PWM_ENUPD_ENUPD1_M)) |
      (PWM_ENUPD_ENUPD0_LSYNC | PWM_ENUPD_ENUPD1_LSYNC);
  HWREG(PWM0_BASE+PWM_O_ENUPD) =  (HWREG(PWM0_BASE+PWM_O_ENUPD) & 
      ~(PWM_ENUPD_ENUPD2_M | PWM_ENUPD_ENUPD3_M)) |
      (PWM_ENUPD_ENUPD2_LSYNC | PWM_ENUPD_ENUPD3_LSYNC);	
	
	// Enable the PWM outputs
  HWREG( PWM0_BASE+PWM_O_ENABLE) |= (PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM0EN |
									 PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN);

	// Now configure the Port B pins to be PWM outputs
	// Start by selecting the alternate function for PB4-PB7
  HWREG(GPIO_PORTB_BASE+GPIO_O_AFSEL) |= (BIT7HI | BIT6HI | BIT5HI | BIT4HI);

	// Now choose to map PWM to those pins, this is a mux value of 4 that we
	// want to use for specifying the function on bits 6 & 7
  HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) = 
    (HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) & 0x00fffff) + (4<<(7*BitsPerNibble)) +
      (4<<(6*BitsPerNibble)) + (4<<(5*BitsPerNibble)) + (4<<(4*BitsPerNibble));

	// Enable pins 4, 5, 6, & 7 on Port B for digital I/O
  HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (BIT7HI | BIT6HI | BIT5HI | BIT4HI | BIT3HI | BIT2HI);
	
	// Set pins 4, 5, 6, & 7 on Port B as outputs
  HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (BIT7HI | BIT6HI | BIT5HI | BIT4HI | BIT3HI | BIT2HI);

  HWREG(GPIO_PORTB_BASE+ALL_BITS) &= ~(BIT2HI | BIT3HI);
	
	//Preset the PWMs to be off
	SetPWMDuty(0,0);
	SetPWMDuty(0,1);
	
	SetPWMWidth(0,2);
	SetPWMWidth(0,3);

	// Set the up/down count mode and enable the PWM generator
  HWREG(PWM0_BASE+ PWM_O_0_CTL) |= (PWM_0_CTL_MODE | PWM_0_CTL_ENABLE);
  HWREG(PWM0_BASE+ PWM_O_1_CTL) |= (PWM_1_CTL_MODE | PWM_1_CTL_ENABLE);
	printf("PWM Initialized\n\r");
}

/****************************************************************************
 Function
     SetPWMDuty

 Parameters
     Duty cycle to set

 Returns
     none

 Description
		Sets PWM duty cycle to given value
****************************************************************************/
void SetPWMDuty(uint8_t duty, int channel) {
	int newDuty;
	static bool zeroStatus = false;
	//check if requested duty is 0 more than 100
	if(duty == 0) {
		newDuty = 0;
		zeroStatus = true;
	}
	else {
		if(channel == 1 || channel == 0) {
			if(duty >= 100) {
				newDuty = ((PeriodInMicroS * PWMTicksPerMicroS) - 1);
			}
			else {
				newDuty = ((PeriodInMicroS * PWMTicksPerMicroS) - 1)*duty/100;
			}
		}
		else if(channel == 2 || channel == 3) {
			if(duty >= 100) {
				newDuty = ((ServoPeriod * PWMTicksPerMicroS) -1);
			}
			else {
				newDuty = ((ServoPeriod * PWMTicksPerMicroS) - 1)*duty/100;
			}
		}
		zeroStatus = false; 
	}
	//Write the PWMs to the proper channel
	switch (channel) {
		case 0 : //PB6 selected
			lastPWM_Starboard = duty;
			if(zeroStatus){ // Disable the output
				HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM0EN;
			}
			else { //Ensure the output is enabled
				if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM0EN) != PWM_ENABLE_PWM0EN){
					HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM0EN;
				}
			}
			// Write newDuty
			HWREG( PWM0_BASE+PWM_O_0_CMPA) = (newDuty)>>1; 
			break;
		case 1 : //PB7 selected	
			lastPWM_Port = duty;
			if(zeroStatus){ // Disable the output				
				HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM1EN;
			}
			else { // Ensure output is enabled
				if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM1EN) != PWM_ENABLE_PWM1EN){
					HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM1EN;
				}
			}
			// Write newDuty
			HWREG( PWM0_BASE+PWM_O_0_CMPB) = (newDuty)>>1;
			break;
		case 2 : //PB4 selected
			if(zeroStatus){ // Disable the output
				HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM2EN;
			}
			else { // Ensure output is enabled
				if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM2EN) != PWM_ENABLE_PWM2EN){
					HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM2EN;
				}
			}
			// Write newDuty
			HWREG( PWM0_BASE+PWM_O_1_CMPA) = (newDuty)>>1; 
			break;
		case 3 : //PB5 selected
			if(zeroStatus){ // Disable the output				
				HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM3EN;
			}
			else { // Ensure output is enabled
				HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM3EN;
			}
			// Write newDuty
			HWREG( PWM0_BASE+PWM_O_1_CMPB) = (newDuty)>>1;
			break;
	}
}

/****************************************************************************
 Function
     Set PWMWidth

 Parameters
     PWM width to set

 Returns
     none

 Description
		Set PWM to specified width
****************************************************************************/
void SetPWMWidth(uint32_t width, int channel) {
	int newWidth;
	static bool zeroStatus = false;
	
	if(width == 0) {
		newWidth = 0;
		zeroStatus = true;
	}
	else {
		if(channel == 0 || channel == 1) {
			if(width >= PeriodInMicroS) {
				newWidth = ((PeriodInMicroS * PWMTicksPerMicroS) -1);
			}
			else {
				newWidth = ((width*PWMTicksPerMicroS) -1);
			}
		}
		else if(channel == 2 || channel == 3) {
			if(width >= ServoPeriod) {
				newWidth = ((ServoPeriod * PWMTicksPerMicroS) - 1);
			}
			else {
				newWidth = ((width * PWMTicksPerMicroS) - 1);
			}
		}
		zeroStatus = false;
	}
	switch (channel) {
		case 0: //PB6 selected
			if(zeroStatus){ // Disable the output
				HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM0EN;
			}
			else { // Ensure output is enabled
				HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM0EN;
			}
			// Write newWidth
			HWREG(PWM0_BASE+PWM_O_0_CMPA) = (newWidth)>>1;
			break;
		case 1: //PB7 selected
			if(zeroStatus){ // Disable the output
				HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM1EN;
			}
			else { // Ensure output is enabled
				HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM1EN;
			}
			// Write newWidth
			HWREG(PWM0_BASE+PWM_O_0_CMPB) = (newWidth)>>1;
			break;
		case 2: //PB4 selected
			if(zeroStatus){ // Disable the output
				HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM2EN;
			}
			else { // Ensure output is enabled
				HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM2EN;
			}
			// Write newWidth
			HWREG(PWM0_BASE+PWM_O_1_CMPA) = (newWidth)>>1;
			break;
		case 3: //PB5 selected
			if(zeroStatus){ // Disable the output
				HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM3EN;
			}
			else { // Ensure output is enabled
				HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM3EN;
			}
			// Write newWidth
			HWREG(PWM0_BASE+PWM_O_1_CMPB) = (newWidth)>>1;
			break;
	}
}

// Set value of last PWM
void SetLastPWM(uint8_t lastPWM, int channel) {
	// Starboard
	if(channel == 0) {
		lastPWM_Starboard = lastPWM;
	}
	// Port
	else if(channel == 1) {
		lastPWM_Port = lastPWM;
	}
}

// Get value of last set PWM
uint8_t GetLastPWM(int channel) {
	// Starboard
	if(channel == 0) {
		return lastPWM_Starboard;
	}
	// Port
	else if(channel == 1) {
		return lastPWM_Port;
	}
	else return 0;
}

// Set value of last direction
void SetLastDirection(int dir, int channel) {
	// Starboard
	if(channel == 0) {
		dirStar = dir;
	}
	// Port
	else if(channel == 1) {
		dirPort = dir;
	}
}

// Get last set direction
int GetLastDirection(int channel) {
	// Starboard
	if(channel == 0) {
		return dirStar;
	}
	// Port
	else if(channel == 1) {
		return dirPort;
	}
	else return 0;
}

