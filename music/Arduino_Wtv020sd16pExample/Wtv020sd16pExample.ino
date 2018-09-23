/*
 Example: Control a WTV020-SD-16P module to play voices from an Arduino board.
 Created by Diego J. Arevalo, August 6th, 2012.
 Released into the public domain.
 */
#include "WTV20SDBreakout.h"

int resetPin = 2;  // The pin number of the reset pin.
int clockPin = 3;  // The pin number of the clock pin.
int dataPin = 4;  // The pin number of the data pin.
int busyPin = 5;  // The pin number of the busy pin.

/*
Create an instance of the Wtv020sd16p class.
 1st parameter: Reset pin number.
 2nd parameter: Clock pin number.
 3rd parameter: Data pin number.
 4th parameter: Busy pin number.
 */
WTV20SDBreakout WTV20SDBreakout(resetPin,clockPin,dataPin,busyPin);

void setup() {
  //Initializes the module.
  WTV20SDBreakout.reset();
}

void loop() {
  //Plays asynchronously an audio file. 
  //(this dumb shit wording just means it doesn't block the fuck out of the code inside the method)
  WTV20SDBreakout.asyncPlayVoice(0);
  //Plays audio file number 0 during 5 seconds.(Yay! we block here instead!!!!)
  delay(1000);
  
   WTV20SDBreakout.asyncPlayVoice(1);
  //Plays audio file number 1 during 5 seconds.
  delay(1000);
  
   WTV20SDBreakout.asyncPlayVoice(2);
  //Plays audio file number 2 during 5 seconds.
  delay(1000);
  
  WTV20SDBreakout.asyncPlayVoice(3);
  //Plays audio file number 3 during 5 seconds.
  delay(1000);
  
  WTV20SDBreakout.asyncPlayVoice(4);
  //Plays audio file number 4 during 5 seconds.
  delay(10000);
  
  //Pauses audio file number 1 during 5 seconds.  
  //WTV20SDBreakout.pauseVoice();
  //delay(5000);
  //Resumes audio file number 1 during 5 seconds.
  //WTV20SDBreakout.pauseVoice();
  //delay(5000);  
  //Stops current audio file playing.
  //WTV20SDBreakout.stopVoice();
  //Plays synchronously an audio file. Busy pin is used for this method.  
  //WTV20SDBreakout.asyncPlayVoice(2);
  //delay(2000);   
  //Mutes audio file number 2 during 2 seconds.
  //WTV20SDBreakout.mute();
  //delay(2000);
  //Unmutes audio file number 2 during 2 seconds.
  //WTV20SDBreakout.unmute();
  //delay(2000);    
  //Stops current audio file playing.
  //WTV20SDBreakout.stopVoice();
}


