/****************************************************************************
 Module
	Sound.c

 Revision			Revised by: 
	0.1.1				Denny

 Description
	Test Service to set up the WTV020SD chip to play sounds based on events
	used so the WTV can be set up as a stand alone module.

 Edits:
	0.1.1 - Initial coding


****************************************************************************/


/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include <stdint.h>
#include <stdbool.h>

// ES_framework headers
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "ES_Types.h"
#include "ES_Events.h"

// Module headers
#include "Sound.h"
#include "WTV020SD.h"


/*----------------------------- Module Defines ----------------------------*/


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behaviour of this service
*/


/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
	InitSound

 Parameters
	uint8_t : the priorty of this service

 Returns
	bool, false if error in initialization, true otherwise

 Description
	All this does is calls the initialization routine for the WTV chip
****************************************************************************/
bool InitSound  ( uint8_t Priority )
{
	MyPriority = Priority;
	
	// Initialize the WTV020SD chip
	InitWTV();
	
	return true;
}

/****************************************************************************
 Function
	PostSound

 Parameters
    ES_Event ThisEvent ,the event to post to the queue

 Returns
    bool false if the Enqueue operation failed, true otherwise

 Description
    Posts an event to this state machine's queue
****************************************************************************/
bool PostSound ( ES_Event ThisEvent )
{
	
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
	RunSound

 Parameters
	ES_Event : the event to process

 Returns
	ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
	
****************************************************************************/
ES_Event RunSound ( ES_Event CurrentEvent )
{
	ES_Event ReturnEvent = {ES_NO_EVENT};
	
	if(CurrentEvent.EventType == PlaySound)
	{
		// Test out just passing param on to WTV module to play
		printf("Playing Sound #%d\r\n", CurrentEvent.EventParam);
		WTVPlaySound(CurrentEvent.EventParam);
		
		// Use this switch statement to try out calling the symbolic sounds
		// once above is working (e.g. START_GAME or COUNTDOWN) 
		switch(CurrentEvent.EventParam)
		{
			case 1 :
				break;
			case 2 :
				break;
			case 3 :
				break;
			case 4 :
				break;
			case 5 :
				break;
			case 6 :
				break;
			case 7 :
				break;
			case 8 :
				break;
			case 9 :
				break;
		}
	}

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/

