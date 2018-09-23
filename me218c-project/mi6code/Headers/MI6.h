/****************************************************************************
 
  Header file for MI6.c

 ****************************************************************************/
 
 
#ifndef MI6_H
#define MI6_H

/*----------------------------- Include Files -----------------------------*/
// set up a global header for the project
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

/*----------------------------- Module Defines ----------------------------*/
typedef enum {IdleState, RequestPairingState, MasterState} MI6State;

/*----------------------- Public Function Prototypes ----------------------*/

bool InitMI6 ( uint8_t Priority );
bool PostMI6 ( ES_Event ThisEvent );
ES_Event RunMI6 ( ES_Event CurrentEvent );
void UART1InterruptResponse( void );
void UART2InterruptResponse( void );
void UnpairInterruptResponse( void );
void PopInterruptResponse( void );
void BrakeInterruptResponse( void );
void FlaskInterruptResponse( void );
void SoftwareReset( void );

#endif /* MI6_H_H */
