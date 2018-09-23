/****************************************************************************
 Module
   PWM.c

    Revision            Revised by: 
    0.1.1               Alex                    
    0.2.1               Denny                       
    0.3.0               Eric                        
    0.3.1               Eric    
    0.4.0               Denny  
    0.4.1               Denny                 

 Description
   Service to initialize the Tiva's hardware PWM output to drive our motors
   2 channels for DC motor PWM
   4 channels for servo PWM
     
 Edits:
    0.1.1 - Restructured as a module, rather than a full service
    0.2.1 - Formatting for report. Changed time references to micro-seconds.
    0.3.0 - Added "SetPWMWidth" function to allow for more precise PWM widths
            than setting the duty cycle. Opened PB4 & PB5 as servo channels.
            Reformed functions to allow for all pwm channels currently to be
            used by both functions
    0.3.1 - Added motor control pins.
    0.4.0 - Added 2 new servo channels on PE4,PE5 (Servo3,Servo4)
    0.4.1 - Converted PWM channel 1 and 2 to servo outputs
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
// Standard headers
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

// TIVA headers
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_pwm.h"
#include "bitdefs.h"

// Module header
#include "PWM.h"

/*----------------------------- Module Defines ----------------------------*/
// 40,000 ticks per mS assumes a 40Mhz clock, we will use SysClk/32 for PWM
#define BitsPerNibble           4
#define ALL_BITS                (0xff<<2)
#define PWMTicksPerMicroS       40000/32000
#define ServoPeriod             20000       // standard 20 ms
#define ONE_MS                  1000
#define TWO_MS                  2000

/*---------------------------- Module Functions ---------------------------*/
// Functions provate to this module
void InitPWM(void);
void SetPWMDuty(uint8_t duty, int channel);
void SetPWMWidth(uint32_t width, int channel);

/*---------------------------- Module Variables ---------------------------*/
// Variables private to this module
static bool zeroStatus = false;

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
Function
  InitPWM

Parameters
  none

 Returns
  none

Description
  Initializes TIVA pins B4-7 and E4,E5 as PWM
    B6: PWM for servo motor 5   (M0PWM0)
    B7: PWM for servo motor 6   (M0PWM1)
    B4: PWM for servo motor 1   (M0PWM2)
    B5: PWM for servo motor 2   (M0PWM3)
    E4: PWM for servo motor 3   (M0PWM4)
    E5: PWM for servo motor 4   (M0PWM5)
  D0: PWM for servo motor 3   (M0PWM6)
    D1: PWM for servo motor 4   (M0PWM7)
****************************************************************************/
void InitPWM( void )
{
  // Start by enabling the clock to the PWM Module (PWM0 & PWM1 & PWM2)
  HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R0;
    
  // Enable the clock to Port B and E
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
    
  // Select the PWM clock as System Clock/32
  HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) |
  (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32);
    
  // Make sure that the PWM module clock has gotten going
  while ((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R0) != SYSCTL_PRPWM_R0)
    ;

  // Disable the PWM while initializing
  HWREG( PWM0_BASE+PWM_O_0_CTL ) = 0; //PB6 & PB7
  HWREG( PWM0_BASE+PWM_O_1_CTL ) = 0; //PB4 & PB5
  HWREG( PWM0_BASE+PWM_O_2_CTL ) = 0; //PE4 & PE5
  HWREG( PWM0_BASE+PWM_O_3_CTL ) = 0; //PD0 & PD1
    
  // Program generator A to go to 0 at rising compare A, 1 on falling compare A  
  HWREG( PWM0_BASE+PWM_O_0_GENA) = 
  (PWM_0_GENA_ACTCMPAU_ZERO | PWM_0_GENA_ACTCMPAD_ONE );
  HWREG( PWM0_BASE+PWM_O_1_GENA) = 
  (PWM_1_GENA_ACTCMPAU_ZERO | PWM_1_GENA_ACTCMPAD_ONE );
  HWREG( PWM0_BASE+PWM_O_2_GENA) = 
  (PWM_2_GENA_ACTCMPAU_ZERO | PWM_2_GENA_ACTCMPAD_ONE );
  HWREG( PWM0_BASE+PWM_O_3_GENA) = 
  (PWM_3_GENA_ACTCMPAU_ZERO | PWM_3_GENA_ACTCMPAD_ONE );

  // Program generator B to go to 0 at rising compare B, 1 on falling compare B  
  HWREG( PWM0_BASE+PWM_O_0_GENB) = 
  (PWM_0_GENB_ACTCMPBU_ZERO | PWM_0_GENB_ACTCMPBD_ONE );
  HWREG( PWM0_BASE+PWM_O_1_GENB) = 
  (PWM_1_GENB_ACTCMPBU_ZERO | PWM_1_GENB_ACTCMPBD_ONE );
  HWREG( PWM0_BASE+PWM_O_2_GENB) = 
  (PWM_2_GENB_ACTCMPBU_ZERO | PWM_2_GENB_ACTCMPBD_ONE );
  HWREG( PWM0_BASE+PWM_O_3_GENB) = 
  (PWM_3_GENB_ACTCMPBU_ZERO | PWM_3_GENB_ACTCMPBD_ONE );

  // Set the PWM period. Since we are counting both up & down, we initialize
  // the load register to 1/2 the desired total period
  // Servo Motor period is set to 20ms
  HWREG( PWM0_BASE+PWM_O_0_LOAD) = ((ServoPeriod * PWMTicksPerMicroS)-1)>>1;
  HWREG( PWM0_BASE+PWM_O_1_LOAD) = ((ServoPeriod * PWMTicksPerMicroS)-1)>>1;
  HWREG( PWM0_BASE+PWM_O_2_LOAD) = ((ServoPeriod * PWMTicksPerMicroS)-1)>>1;
  HWREG( PWM0_BASE+PWM_O_3_LOAD) = ((ServoPeriod * PWMTicksPerMicroS)-1)>>1;
    
  // Set the initial Duty cycle on A to 2`5% by programming the compare value
  // to 1/8 the period to count up (or down) 
  HWREG( PWM0_BASE+PWM_O_0_CMPA) = ((ONE_MS * PWMTicksPerMicroS)-1)>>2;
  HWREG( PWM0_BASE+PWM_O_1_CMPA) = ((ONE_MS * PWMTicksPerMicroS)-1)>>2;
  HWREG( PWM0_BASE+PWM_O_2_CMPA) = ((ONE_MS * PWMTicksPerMicroS)-1)>>2;
  HWREG( PWM0_BASE+PWM_O_3_CMPA) = ((ONE_MS * PWMTicksPerMicroS)-1)>>2;


  // Set the initial Duty cycle on B to 25% by programming the compare value
  // to 1/8 the period  
  HWREG( PWM0_BASE+PWM_O_0_CMPB) = ((TWO_MS * PWMTicksPerMicroS)-1)>>2;
  HWREG( PWM0_BASE+PWM_O_1_CMPB) = ((TWO_MS * PWMTicksPerMicroS)-1)>>2;
  HWREG( PWM0_BASE+PWM_O_2_CMPB) = ((TWO_MS * PWMTicksPerMicroS)-1)>>2;
  HWREG( PWM0_BASE+PWM_O_3_CMPB) = ((TWO_MS * PWMTicksPerMicroS)-1)>>2;
    
  // Set changes to the PWM output Enables to be locally syncronized to a zero count
  HWREG(PWM0_BASE+PWM_O_ENUPD) =  (HWREG(PWM0_BASE+PWM_O_ENUPD) & 
     ~(PWM_ENUPD_ENUPD0_M | PWM_ENUPD_ENUPD1_M)) |
      (PWM_ENUPD_ENUPD0_LSYNC | PWM_ENUPD_ENUPD1_LSYNC);
  HWREG(PWM0_BASE+PWM_O_ENUPD) =  (HWREG(PWM0_BASE+PWM_O_ENUPD) & 
     ~(PWM_ENUPD_ENUPD2_M | PWM_ENUPD_ENUPD3_M)) |
      (PWM_ENUPD_ENUPD2_LSYNC | PWM_ENUPD_ENUPD3_LSYNC);  
  HWREG(PWM0_BASE+PWM_O_ENUPD) =  (HWREG(PWM0_BASE+PWM_O_ENUPD) & 
     ~(PWM_ENUPD_ENUPD4_M | PWM_ENUPD_ENUPD5_M)) |
      (PWM_ENUPD_ENUPD4_LSYNC | PWM_ENUPD_ENUPD5_LSYNC);
  HWREG(PWM0_BASE+PWM_O_ENUPD) =  (HWREG(PWM0_BASE+PWM_O_ENUPD) & 
     ~(PWM_ENUPD_ENUPD6_M | PWM_ENUPD_ENUPD7_M)) |
      (PWM_ENUPD_ENUPD6_LSYNC | PWM_ENUPD_ENUPD7_LSYNC);

  // Enable the PWM outputs
  HWREG( PWM0_BASE+PWM_O_ENABLE) |= 
     (PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM0EN |
      PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN |
      PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM0EN |
      PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN);

  // Now configure the Port B and Port E pins to be PWM outputs
  // Start by selecting the alternate function for PB4-PB7, PE4-PE5
  HWREG(GPIO_PORTB_BASE+GPIO_O_AFSEL) |= (BIT7HI | BIT6HI | BIT5HI | BIT4HI);
  HWREG(GPIO_PORTE_BASE+GPIO_O_AFSEL) |= (BIT5HI | BIT4HI);
  HWREG(GPIO_PORTD_BASE+GPIO_O_AFSEL) |= (BIT0HI)|(BIT1HI);

  // Select PWM alternate functions on B4,B5,B6,B7 and E4,E5
  // Set mux position on GPIOPCTL to a mux value of 4 (p.651)
  // offset mask by clearing the corresponding nibbles in PCTL
  HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) = 
  (HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) & ~0xffff0000) + (4<<(7*BitsPerNibble)) +
    (4<<(6*BitsPerNibble)) + (4<<(5*BitsPerNibble)) + (4<<(4*BitsPerNibble));
  HWREG(GPIO_PORTE_BASE+GPIO_O_PCTL) = 
  (HWREG(GPIO_PORTE_BASE+GPIO_O_PCTL) & ~0x00ff0000) + (4<<(5*BitsPerNibble)) +
    (4<<(4*BitsPerNibble));
  HWREG(GPIO_PORTD_BASE+GPIO_O_PCTL) = 
  (HWREG(GPIO_PORTD_BASE+GPIO_O_PCTL) & ~0x000000ff) +
    (4<<(0*BitsPerNibble))+(4<<(1*BitsPerNibble));

  // Enable pins 4,5,6,7 on Port B and pins 4,5 on Port E for digital I/O
  HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (BIT7HI | BIT6HI | BIT5HI | BIT4HI );
  HWREG(GPIO_PORTE_BASE+GPIO_O_DEN) |= (BIT5HI | BIT4HI );
  HWREG(GPIO_PORTD_BASE+GPIO_O_DEN) |= (BIT0HI)|(BIT1HI);
    
  // Set pins 4,5,6,7 on Port B  and pins 4,5 on Port E as outputs
  HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (BIT7HI | BIT6HI | BIT5HI | BIT4HI );
  HWREG(GPIO_PORTE_BASE+GPIO_O_DIR) |= (BIT5HI | BIT4HI );
  HWREG(GPIO_PORTD_BASE+GPIO_O_DIR) |= (BIT0HI)|(BIT1HI);

  HWREG(GPIO_PORTB_BASE+ALL_BITS) &= ~(BIT2HI | BIT3HI);
  //Preset the PWMs to be off
  SetPWMDuty(0,0);
  SetPWMDuty(0,1);
    
  // Set the width to the 0 position for the servos
  SetPWMWidth(0,SERVO_1);
  SetPWMWidth(0,SERVO_2);
  SetPWMWidth(0,SERVO_3);
  SetPWMWidth(0,SERVO_4);
  SetPWMWidth(0,SERVO_5);
  SetPWMWidth(0,SERVO_6);
  SetPWMWidth(0,SERVO_7);
  SetPWMWidth(0,SERVO_8);
	
  // Set the up/down count mode and enable the PWM generator
  HWREG(PWM0_BASE+PWM_O_0_CTL) |= (PWM_0_CTL_MODE | PWM_0_CTL_ENABLE);
  HWREG(PWM0_BASE+PWM_O_1_CTL) |= (PWM_1_CTL_MODE | PWM_1_CTL_ENABLE);
  HWREG(PWM0_BASE+PWM_O_2_CTL) |= (PWM_2_CTL_MODE | PWM_2_CTL_ENABLE);
  HWREG(PWM0_BASE+PWM_O_3_CTL) |= (PWM_3_CTL_MODE | PWM_3_CTL_ENABLE);

  printf("PWM Initialized\n\r");
}

/****************************************************************************
 Function
     SetPWMDuty

 Parameters
     Duty cycle to set (0-100 range), channel

 Returns
     none

 Description
        Sets PWM duty cycle to given value 
****************************************************************************/
void SetPWMDuty(uint8_t duty, int channel) {
  int newDuty;

  //check if requested duty is 0 or more than 100
  if(duty <= 0) {
    newDuty = 0;
    zeroStatus = true;
  }
  else {
    if(duty >= 100) {
      newDuty = ((ServoPeriod * PWMTicksPerMicroS) -1);
    }
    else {
      newDuty = ((ServoPeriod * PWMTicksPerMicroS) - 1)*duty/100;
    }
    zeroStatus = false; 
  }

  //Write duty to the proper channel
  switch (channel) {

    case 0 : //PB6 selected (DC motor 1/Servo 5)
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

    case 1 : //PB7 selected (DC motor 2/Servo 6)
      if(zeroStatus){ // Disable the output               
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM1EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM1EN) != PWM_ENABLE_PWM1EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM1EN;
      }
      // Write newDuty
      HWREG( PWM0_BASE+PWM_O_0_CMPB) = (newDuty)>>1;
      break;

    case 2 : //PB4 selected (Servo 1)
      if(zeroStatus){ // Disable the output
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM2EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM2EN) != PWM_ENABLE_PWM2EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM2EN;
      }
      // Write newDuty
      HWREG( PWM0_BASE+PWM_O_1_CMPA) = (newDuty)>>1; 
      break;

    case 3 : //PB5 selected (Servo 2)
      if(zeroStatus){ // Disable the output               
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM3EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM3EN) != PWM_ENABLE_PWM3EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM3EN;
      }
      // Write newDuty
      HWREG( PWM0_BASE+PWM_O_1_CMPB) = (newDuty)>>1;
      break;

    case 4 : //PE4 selected (Servo 3)
      if(zeroStatus){ // Disable the output               
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM4EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM4EN) != PWM_ENABLE_PWM4EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM4EN;
      }
      // Write newDuty
      HWREG( PWM0_BASE+PWM_O_2_CMPA) = (newDuty)>>1;
      break;

    case 5 : //PE5 selected (Servo 4)
      if(zeroStatus){ // Disable the output               
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM5EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM5EN) != PWM_ENABLE_PWM5EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM5EN;
      }
      // Write newDuty
      HWREG( PWM0_BASE+PWM_O_2_CMPB) = (newDuty)>>1;
      break;
  }
}

/****************************************************************************
 Function
     Set PWMWidth

 Parameters
     PWM width to set (0-1000 range), channel

 Returns
     none

 Description
        Set PWM to specified width
****************************************************************************/
void SetPWMWidth(uint32_t width, int channel) {
  int newWidth;
  
  if(width <= 0) {
    newWidth = 0;
    zeroStatus = true;
  }
  else {
    if(width >= ServoPeriod) {
      newWidth = ((ServoPeriod * PWMTicksPerMicroS) - 1);
    }
    else {
      newWidth = ((width * PWMTicksPerMicroS) - 1);
    }
    zeroStatus = false;
  }
  switch (channel) {

    case 0: //PB6 selected (DC 1/Servo 5)
      if(zeroStatus){ // Disable the output
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM0EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM0EN) != PWM_ENABLE_PWM0EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM0EN;
      }
      // Write newWidth
      HWREG(PWM0_BASE+PWM_O_0_CMPA) = (newWidth)>>1;
      break;

    case 1: //PB7 selected (DC 2/Servo 6)
      if(zeroStatus){ // Disable the output
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM1EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM1EN) != PWM_ENABLE_PWM1EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM1EN;
      }
      // Write newWidth
      HWREG(PWM0_BASE+PWM_O_0_CMPB) = (newWidth)>>1;
      break;

    case 2: //PB4 selected (Servo 1)
      if(zeroStatus){ // Disable the output
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM2EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM2EN) != PWM_ENABLE_PWM2EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM2EN;
      }
      // Write newWidth
      HWREG(PWM0_BASE+PWM_O_1_CMPA) = (newWidth)>>1;
      break;

    case 3: //PB5 selected (Servo 2)
      if(zeroStatus){ // Disable the output
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM3EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM3EN) != PWM_ENABLE_PWM3EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM3EN;
      }
      // Write newWidth
      HWREG(PWM0_BASE+PWM_O_1_CMPB) = (newWidth)>>1;
      break;

    case 4: //PE4 selected (Servo 3)
      if(zeroStatus){ // Disable the output
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM3EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM4EN) != PWM_ENABLE_PWM4EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM4EN;
      }
      // Write newWidth
      HWREG(PWM0_BASE+PWM_O_2_CMPA) = (newWidth)>>1;
      break;

    case 5: //PE5 selected (Servo 4)
      if(zeroStatus){ // Disable the output
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM3EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM5EN) != PWM_ENABLE_PWM5EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM5EN;
      }
      // Write newWidth
      HWREG(PWM0_BASE+PWM_O_2_CMPB) = (newWidth)>>1;
      break;
    case 6: //PD0 selected (Servo 7)
      if(zeroStatus){ // Disable the output
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM3EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM6EN) != PWM_ENABLE_PWM6EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM6EN;
      }
      // Write newWidth
      HWREG(PWM0_BASE+PWM_O_3_CMPA) = (newWidth)>>1;
      break;
    case 7: //PD1 selected (Servo 8)
      if(zeroStatus){ // Disable the output
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM3EN;
      }
      else { // Ensure output is enabled
        if((HWREG(PWM0_BASE+PWM_O_ENABLE)&PWM_ENABLE_PWM7EN) != PWM_ENABLE_PWM7EN)
          HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM7EN;
      }
      // Write newWidth
      HWREG(PWM0_BASE+PWM_O_3_CMPB) = (newWidth)>>1;
      break;
  }
}
