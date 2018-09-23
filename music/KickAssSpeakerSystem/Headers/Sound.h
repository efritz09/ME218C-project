/****************************************************************************
 
  Header file the Epic Mario Kart Sandstorm Ball Blaster Dankbot Sound Module
	testing service... once debugged we should be able to just use the WTV020SD
	module in our master code.

 ****************************************************************************/

#ifndef Sound_H
#define Sound_H

/*----------------------------- Include Files -----------------------------*/

/*----------------------------- Module Defines ----------------------------*/
// Give symbolic defines to the sounds (ie Mario saying "Lets Go!" as 0000


/*----------------------- Public Function Prototypes ----------------------*/

bool InitSound ( uint8_t Priority );
bool PostSound ( ES_Event ThisEvent );
ES_Event RunSound ( ES_Event CurrentEvent );

#endif /* Sound_H */



