/****************************************************************************
 Module
   main.c

 Revision
   1.0.1

 Description
   This is the main() for the KART music subsystem

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 
****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "termio.h"

#define clrScrn() 	printf("\x1b[2J")
#define goHome()		printf("\x1b[1,1H")
#define clrLine()		printf("\x1b[K")

int main (void)
{
		// Set the clock to run at 40MhZ using the PLL and 16MHz external crystal
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
			| SYSCTL_XTAL_16MHZ);
	TERMIO_Init();
  clrScrn();
	
	ES_Return_t ErrorType;
	
	puts("\rInitiating WTV020 Sound Breakout Test Module:\r");
	printf("%s %s\n",__TIME__, __DATE__);
	printf("\n\r\n");


  
// Your hardware initialization function calls go here

// Now initialize the Events and Services Framework and start it running
	ErrorType = ES_Initialize(ES_Timer_RATE_1mS);
	if ( ErrorType == Success ) {

	  ErrorType = ES_Run();

	}
  
//if we got to here, there was an error
  switch (ErrorType){
    case FailedPointer:
      puts("Failed on NULL pointer");
      break;
    case FailedInit:
      puts("Failed Initialization");
      break;
    default:
      puts("Other Failure");
      break;
  }
  for(;;)   // hang after reporting error
    ;
}

