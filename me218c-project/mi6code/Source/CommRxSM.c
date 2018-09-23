/****************************************************************************
 Module
	CommRxSM.c

 Revision			Revised by: 
	1.0.0				Vikram
	1.0.1				Eric
	1.1.0				Eric

 Description
	UART for TIVA asynchronous communication for Lab 10

 Edits:
	1.0.0				Established connection between Xbees
	1.0.1				Added checkSum function for error checking, added PWM and 
							ADC modules
	1.1.0				SPECTRE Xbee setup

****************************************************************************/


/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

// Module headers
// Standard headers
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

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
#include "inc/hw_uart.h"
#include "inc/hw_nvic.h"
#include "inc/hw_timer.h"
#include "bitdefs.h"


#include "CommRxSM.h"
#include "MI6.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble 			4
#define TicksPerMS 					40000				// 40Mhz
#define ALL_BITS 						(0xff<<2)
#define START_DELIMETER			0x7E
#define RX_TIMEOUT					150


/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.
*/



/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static RxState CurrentState;

uint16_t DataLen = 0x00;
static uint8_t rxData[12] = {0x00};

static uint16_t arrayIndex = 0;
static uint8_t chksum = 0x00;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitCommRxSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
****************************************************************************/

bool InitCommRxSM ( uint8_t Priority )
{
  ES_Event ThisEvent;
  
  MyPriority = Priority;

	CurrentState = WaitForStartState;
	//printf("comms in initialized\r\n");
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
     PostCommRxSM

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostCommRxSM( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}



/****************************************************************************
 Function
    RunCommRxSM

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here

****************************************************************************/
ES_Event RunCommRxSM ( ES_Event CurrentEvent )
{
	ES_Event ReturnEvent = {ES_NO_EVENT};
	RxState NextState = CurrentState;

	switch(CurrentState) {
		case WaitForStartState:
			//if a byte has been received, start the transition
			if(CurrentEvent.EventType == ByteReceived){ 
				if(CurrentEvent.EventParam == START_DELIMETER){ // CHANGES
					NextState = WaitForMSBState;
					ES_Timer_InitTimer(RxTimer,RX_TIMEOUT);
					//printf("Incoming message...\n\r");
				}
			}
			break;
		case WaitForMSBState:
			if(CurrentEvent.EventType == ByteReceived) {
				//store MSB value
				DataLen = CurrentEvent.EventParam<<8;
				NextState = WaitForLSBState;
				ES_Timer_InitTimer(RxTimer,RX_TIMEOUT); // CHANGES moved out of else
				//printf("MSB received...\n\r");
			}
			//if the data was interrupted for some reason
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == RxTimer) {
				NextState = WaitForStartState;
				//printf("message timeout\r\n");
			}
			break;
		case WaitForLSBState:
			if(CurrentEvent.EventType == ByteReceived) {
				//store LSB value
				DataLen += CurrentEvent.EventParam;
				NextState = ProcessDataState;
				ES_Timer_InitTimer(RxTimer,RX_TIMEOUT);
				//printf("LSB received, data length = %d , beginning data extraction\n\r",DataLen);
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == RxTimer) {
				NextState = WaitForStartState;
				//printf("message timeout\r\n");
			}
			break;
		case ProcessDataState:
			if(CurrentEvent.EventType == ByteReceived) {
				ES_Timer_InitTimer(RxTimer,RX_TIMEOUT);
				//while there's still data coming in
				if(DataLen > arrayIndex) { // CHANGES 
					//store the value in the array
					rxData[arrayIndex++] = CurrentEvent.EventParam;
					//printf("data at this time %x\r\n",CurrentEvent.EventParam);
					chksum += CurrentEvent.EventParam;
				}
				//if we've reached our last value
				else if(DataLen == arrayIndex) {
					//printf("arrayIndex = %d\r\n",arrayIndex);
					//printf("checksum %x\r\n",0xFF-chksum);
					arrayIndex = 0;
					if((0xFF-chksum) == CurrentEvent.EventParam) { //check the checksum value
						//printf("Extraction successful, proceeding to processing\r\n");
						ES_Timer_StopTimer(RxTimer);
						ES_Event ThisEvent;
						ThisEvent.EventType = PackageReceived; //return event for a good checksum, otherwise ignore
						PostMI6(ThisEvent);
					}
					//reset values regardless
					NextState = WaitForStartState; // CHANGES
					chksum = 0x00;
					arrayIndex = 0;
				}
				else { //somehow we've gone over?
					chksum = 0x00;
					arrayIndex = 0;
					NextState = WaitForStartState; // CHANGES
					//printf("how the hell did this happen?\r\n");
				}
			}
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == RxTimer) {
				NextState = WaitForStartState;
				//printf("arrayindex = %d\r\n",arrayIndex);
				arrayIndex = 0;
				//printf("the fuck? rx timeout\r\n");
			}
			break;

	}
	CurrentState = NextState;
  return ReturnEvent;
}


/***************************************************************************
 private functions
 ***************************************************************************/
 
uint8_t * GetRxData(void) {
	return rxData;
}


/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
