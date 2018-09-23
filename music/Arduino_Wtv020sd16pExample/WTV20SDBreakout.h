/*
 WTV20SDBreakout.h - Library to control a WTV020-SD-16P module to play voices from an Arduino board.
 Created by Diego J. Arevalo, August 6th, 2012.
 Modified by Peter Valentine, Dec 12th, 2012.
 Released into the public domain.
 */

#ifndef WTV20SDBreakout_h
#define WTV20SDBreakout_h

class WTV20SDBreakout
{
public:
  WTV20SDBreakout(int resetPin,int clockPin,int dataPin,int busyPin);
  void reset();
  void playVoice(int voiceNumber);
  void asyncPlayVoice(int voiceNumber);
  void stopVoice();
  void pauseVoice();
  void mute();
  void unmute();
  void volume(int level);	//New Command
private:
  void sendCommand(unsigned int command);
  int _resetPin;
  int _clockPin;
  int _dataPin;
  int _busyPin;
  int _busyPinState;
};

#endif


