/****************************************************************************
 Module
	MotorTester.c

 Revision			Revised by: 
	0.1.1				Denny
	0.1.2				Denny

 Description
	Test harness for controlling hovercraft lift and propulsion

 Edits:
	0.1.1 - Set up
	0.1.2 - Added PWM functionality to drive ESC's 
	0.1.3 - Added PWM functionality for door servos, expanded tests


****************************************************************************/
// If we are debugging
//#define TEST

/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
// Standard headers
#include <stdint.h>
#include <stdbool.h>

// ES_framework headers
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "ES_Types.h"
#include "ES_Events.h"

// TIVA headers
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "bitdefs.h"

// Module headers
#include "MotorTester.h"
#include "PWM.h"


/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble 			4
#define TicksPerMS 					40000				//40Mhz
#define ALL_BITS 						(0xff<<2)
#define BLOWER_MASK					0x01				//mask to toggle the lift fan
#define PROP_MASK						0x02				//mask to toggle the propellers
#define PROP_WIDTH					1200				//1000 = full stop, 1100 = slowest, 1300 = fastest
#define PROP_OFF						1000				//full brake
#define PROP_MIN						1100				//slowest steady state speed
#define PROP_MAX						1300				//full speed
#define DOOR_MIN						1000				//placeholder
#define DOOR_MAX						2000				//placeholder


/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.
*/
static void InitGPIO( void );

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static bool propOn;
static bool liftOn;
static uint32_t propWidth;
static uint32_t doorWidth;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
	InitMotorTester

 Parameters
	uint8_t : the priorty of this service

 Returns
	bool, false if error in initialization, true otherwise

 Description
	B0 = Lift fan
	B4 = Prop1
	B5 = Prop2
	E4 = Doors
	E6 = Fins
****************************************************************************/
bool InitMotorTester  ( uint8_t Priority )
{
	ES_Event ThisEvent;
	MyPriority = Priority;
	InitPWM();						// Initialize servo PWM
	InitGPIO();						// Initialize GPIO output for lift fan
	
	// Disable motors initially
	HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) &= ~( BIT0HI);
	SetPWMWidth(PROP_OFF, SERVO_1);
	SetPWMWidth(PROP_OFF, SERVO_2);
	SetPWMWidth(DOOR_MIN, SERVO_3);
	SetPWMWidth(DOOR_MIN, SERVO_4);

	// Set initial states
	liftOn = false;
	propOn = false;
	propWidth = PROP_OFF;
	doorWidth = DOOR_MIN;
	
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
	PostMotorTester 

 Parameters
    ES_Event ThisEvent ,the event to post to the queue

 Returns
    bool false if the Enqueue operation failed, true otherwise

 Description
    Posts an event to this state machine's queue
****************************************************************************/
bool PostMotorTester( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
	RunMotorTester 

 Parameters
	ES_Event : the event to process

 Returns
	ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
	
****************************************************************************/
ES_Event RunMotorTester ( ES_Event CurrentEvent )
{
	ES_Event ReturnEvent = {ES_NO_EVENT};	

	if ( CurrentEvent.EventType == TOGGLE_BLOWER ) {
		liftOn = !liftOn;
		// Toggle the blower motor (1 = on, 0 = off)
		HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) = 
			HWREG(GPIO_PORTB_BASE + ALL_BITS) ^ BLOWER_MASK;
		if (liftOn) printf("Lift Motor on\n\r");
		else printf("Lift Motor off\n\r");
	}
	else if ( CurrentEvent.EventType == TOGGLE_PROP ) {
		// Toggle the propulsion (1 = on, 0 = off)
		propOn = !propOn;
		if (propOn)	propWidth = PROP_WIDTH;
		else propWidth = PROP_OFF;
		SetPWMWidth(propWidth, SERVO_1);
		SetPWMWidth(propWidth, SERVO_2);
		printf("Propulsion Motor: %d\n\r", PROP_WIDTH);
	}
	else if ( CurrentEvent.EventType == INC_PROP ) {
		// Increase the door servo position, constrain to max
		propWidth +=5;
		if(propWidth >= PROP_MAX) doorWidth = PROP_MAX;
		SetPWMWidth(propWidth, SERVO_1);
		SetPWMWidth(propWidth, SERVO_2);
		printf("doorWidth increased to: %d\n\r", propWidth);
	}
	else if ( CurrentEvent.EventType == DEC_PROP ) {
		// Decrease the door servo position, constrain to min
		propWidth -=5;
		if(propWidth < PROP_MIN) propWidth = PROP_MIN;
		SetPWMWidth(propWidth, SERVO_1);
		SetPWMWidth(propWidth, SERVO_2);
		printf("doorWidth decreased to: %d\n\r", propWidth);
	}
	else if ( CurrentEvent.EventType == INC_DOOR ) {
		// Increase the door servo position, constrain to max
		doorWidth +=1000;
		if(doorWidth >= DOOR_MAX) doorWidth = DOOR_MAX;
		SetPWMWidth(doorWidth, SERVO_3);
		printf("doorWidth increased to: %d\n\r", doorWidth);
	}
	else if ( CurrentEvent.EventType == DEC_DOOR ) {
		// Decrease the door servo position, constrain to min
		doorWidth -=1000;
		if(doorWidth < DOOR_MIN) doorWidth = DOOR_MIN;
		SetPWMWidth(doorWidth, SERVO_3);
		printf("doorWidth decreased to: %d\n\r", doorWidth);
	}

	return ReturnEvent;
}


/****************************************************************************
 Function
   InitGPIO

 Parameters
   none

 Returns
   none

 Description
	 Initialize GPIO pins for inputs/outputs

		Inputs:						Outputs:
									B0 as impeller

****************************************************************************/
static void InitGPIO( void )
{
	// Enable Port B to the timer
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1; 
	
	// Enable Port A Timer
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R0;
	
	// Wait for GPIO Port B to be ready
	while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1 ) != SYSCTL_PRGPIO_R1 )
		;
	
	// Set Pins B0-B1 as digital
	HWREG(GPIO_PORTB_BASE + GPIO_O_DEN) |= ( BIT0HI );
	// Set Pins B0-B1 as outputs
	HWREG(GPIO_PORTB_BASE + GPIO_O_DIR) |= ( BIT0HI );
	
	// Print to console if successful initialization
	printf("GPIO Initialized\n\r");
	
}
/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
