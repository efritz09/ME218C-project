/****************************************************************************
 
  Header file for the WTV020SD Sparkfun audio breakout framework service

 ****************************************************************************/

#ifndef WTV020SDService_H
#define WTV020SDService_H

/*----------------------------- Include Files -----------------------------*/

/*----------------------------- Module Defines ----------------------------*/
// WTV State Machine
typedef enum {
				WTVReady,
				WTVResetting,
				WTVTransferring
} WTVState_t;
// Resetting State Machine
typedef enum {
				ResetPulse,
				ResetDelay
} ResettingState_t;
// Transferring State Machine
typedef enum {
				ClockHigh,
                SendData,
				ClockLow
} TransferringState_t;

// Symbolic defines for playback
#define PLAY_PAUSE          0xFFFE
#define STOP                0xFFFF
#define VOLUME_MIN          0xFFF0
#define VOLUME_MAX          0xFFF7

// Symbolic defines for audio clips
#define GOD_DAMN_IT_ARCHER  0x0000	//filename.ad4
#define HOW_YOU_GET_ANTS    0x0001	//mk64countdown.ad4 


/*----------------------- Public Function Prototypes ----------------------*/
bool InitWTV( uint8_t Priority );
bool PostWTV( ES_Event ThisEvent );
ES_Event RunWTV( ES_Event );
void PlaySound( uint16_t );


#endif /* WtV020SD_H */
