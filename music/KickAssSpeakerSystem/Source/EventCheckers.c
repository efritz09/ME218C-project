/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.1 

 Description
   Keystroke events to simulate the Command Generator's commands

 Notes
   
****************************************************************************/

// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function 
// definitions too.
#include "ES_PostList.h"
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header & 
// actual functionsdefinition
#include "EventCheckers.h"

// Our project headers
#include "WTV020SDService.h"


/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new key from the keyboard is detected
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker 
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void)
{
  if ( IsNewKeyReady() ) // new key waiting?
  {
    ES_Event ThisEvent;
    ThisEvent.EventType = ES_NEW_KEY;
    ThisEvent.EventParam = GetNewKey();

    if ( ThisEvent.EventParam == '1'){
			// Play 1st sound (0000)
            PlaySound(0x0000);
    }
		else if ( ThisEvent.EventParam == '2'){
			// Play 2nd sound (0001)
            PlaySound(0x0001);
    }
		else if ( ThisEvent.EventParam == '3'){
			// Play 3rd sound (0002)
            PlaySound(0x0002);
    }
		else if ( ThisEvent.EventParam == '4'){
			// Play 4th sound (0003)
            PlaySound(0x0003);
    }
		else if ( ThisEvent.EventParam == '5'){
			// Play 5th sound (0004)
            PlaySound(0x0004);
    }
		else if ( ThisEvent.EventParam == '6'){
			// Play 6th sound (0005)
            PlaySound(0x0005);
    }
		else if ( ThisEvent.EventParam == '7'){
			// Play 7th sound (0006)
            PlaySound(0x0006);
    }
		else if ( ThisEvent.EventParam == '8'){
			// Play 8th sound (0007)
            PlaySound(0x0007);
    }
		else if ( ThisEvent.EventParam == '9'){
			// Play 9th sound (0008)
            PlaySound(0x0008);
    }
		else{   // otherwise post to Service 0 for processing
    }
    return true;
  }
  return false;
}
