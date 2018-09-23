/****************************************************************************
 
  Header file for Lab 10

 ****************************************************************************/

#ifndef MOTORTESTER_H
#define MOTORTESTER_H

/*----------------------------- Include Files -----------------------------*/
// set up a global header for the project

/*----------------------------- Module Defines ----------------------------*/

/*----------------------- Public Function Prototypes ----------------------*/

bool InitMotorTester ( uint8_t Priority );
bool PostMotorTester ( ES_Event ThisEvent );
ES_Event RunMotorTester ( ES_Event CurrentEvent );

#endif /* MOTORTESTER_H */
