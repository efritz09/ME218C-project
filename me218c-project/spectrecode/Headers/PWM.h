/****************************************************************************
  
  Header file for PWM.c
  
 ****************************************************************************/
 
#ifndef PWM_H
#define PWM_H

// Symbolic defines for each PWM channel
#define SERVO_1   2   // PWM channel 2 (PB4)
#define SERVO_2   3   // PWM channel 3 (PB5)
#define SERVO_3   4   // PWM channel 4 (PE4)
#define SERVO_4   5   // PWM channel 5 (PE5)
#define SERVO_5   0   // PWM channel 0 (PB6)
#define SERVO_6   1   // PWM channel 1 (PB7)
#define SERVO_7   6   // PWM channel 0 (PD0)
#define SERVO_8   7   // PWM channel 1 (PD1)

// Public functions for this module
void InitPWM(void);
void SetPWMDuty(uint8_t duty, int channel);
void SetPWMWidth(uint32_t width, int channel);

#endif
