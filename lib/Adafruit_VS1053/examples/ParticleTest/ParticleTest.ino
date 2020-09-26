// ********************************************************************** /
// This is a library for the Adafruit VS1053 Codec Breakout
//
// Designed specifically to work with the Adafruit VS1053 Codec Breakout 
// ----> https://www.adafruit.com/products/1381
//       https://www.adafruit.com/product/3357
//       https://www.adafruit.com/product/3436
// 
// Adafruit invests time and resources providing this open source code, 
// please support Adafruit and open-source hardware by purchasing 
// products from Adafruit!
//// Written by Limor Fried/Ladyada for Adafruit Industries.  
// BSD license, all text above must be included in any redistribution
// 
// Original library: https://github.com/adafruit/Adafruit_VS1053_Library
// 
// Ported for Particle by ScruffR
// Forked and ported: https://github.com/ScruffR/Adafruit_VS1053_Library
// ********************************************************************** /
//
// To run this example prepare a micro SD card with a file hierarchy like
// SD:
// └───01
//        001.mp3
//        002.mp3
// optionally up to ...  
//        998.mp3
//        999.mp3
//
// ********************************************************************** /

SYSTEM_THREAD(ENABLED)
#include "SdFat.h"
#include "Adafruit_VS1053.h"

SerialLogHandler traceLog(LOG_LEVEL_WARN, { { "app", LOG_LEVEL_INFO } });

SdFat SD;

// These are the pins used for the music maker FeatherWing
const int  MP3_RESET        = -1;                 // VS1053 reset pin (unused!)
const int  SD_CS            = D2;                 // SD Card chip select pin
const int  MP3_CS           = D3;                 // VS1053 chip select pin (output)
const int  DREQ             = D4;                 // VS1053 Data request, ideally an Interrupt pin
const int  MP3_DCS          = D5;                 // VS1053 Data/command select pin (output)
const char *fileNamePattern = "%03d.mp3";         // file name pattern to insert track number
Adafruit_VS1053_FilePlayer musicPlayer(MP3_RESET, MP3_CS, MP3_DCS, DREQ, SD_CS); 

int        trackNumber      = 0;
bool       needStart        = false;

void setup() {
  pinMode(D7, OUTPUT);
  Particle.function("playSine", playSine);
  Particle.function("playTrack", playTrack);
  Particle.function("setVolume", setVolume);
  
  if (!SD.begin(SD_CS)) {
    Log.error("SD failed, or not present");
    while(1) yield();                             // don't do anything more
  }
  Serial.println("SD OK!");

  Log.info("Adafruit VS1053 Library Test");
  // initialise the music player
  if (!musicPlayer.begin()) {                     // initialise the music player
     Log.error("Couldn't find VS1053, do you have the right pins defined?");
     while(1) yield();
  }
  Log.info("VS1053 found");

  // Make a tone to indicate VS1053 is working
  musicPlayer.sineTest(0x44, 200);

  // set current working directory
  SD.chdir("/01", true);
  // list files
  SD.ls(&Serial, LS_R);

  // Set volume for left, right channels. lower numbers == louder volume!
  //musicPlayer.setVolume(20, 20);

  // ***** Two interrupt options! ***** 
  // This option uses timer0, this means timer1 & t2 are not required
  // (so you can use 'em for Servos, etc) BUT millis() can lose time
  // since we're hitchhiking on top of the millis() tracker
  //musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT);
  
  // This option uses a pin interrupt. No timers required! But DREQ
  // must be on an interrupt pin. For Uno/Duemilanove/Diecimilla
  // that's Digital #2 or #3
  // See http://arduino.cc/en/Reference/attachInterrupt for other pins
  // *** This method is preferred ***

  if (musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT))
  {
    digitalWrite(D7, HIGH);
    musicPlayer.setIsrCallback(blink);
  }
  else
    Log.info("DREQ pin is not an interrupt pin");

  // Alternatively just play an entire file at once
  // This doesn't happen in the background, instead, the entire
  // file is played and the program will continue when it's done!
  // musicPlayer.playFullFile("001.mp3");
}

void loop() {
  static uint32_t msPlayStarted = 0;
  static uint32_t msLastAction = 0;

  if (needStart && trackNumber) {
    char fileName[32];
    char msg[128];
    uint32_t us = micros();

    // Start playing a file, then we can do stuff while waiting for it to finish
    snprintf(fileName, sizeof(fileName), fileNamePattern, trackNumber);
    Log.trace("Starting: %lu", micros() - us); us = micros();
    if (musicPlayer.startPlayingFile(fileName)) {
      Log.trace("Started: %lu", micros() - us); us = micros();
      snprintf(msg, sizeof(msg), "Started playing '%s'",fileName);
      msPlayStarted = millis();
    }
    else {
      Log.trace("Not started: %lu", micros() - us); us = micros();
      snprintf(msg, sizeof(msg), "Could not open file '%s'",fileName);
    }
    Log.info(msg);
    needStart = false;
  }

  if (millis() - msLastAction >= 1000) {
    uint32_t sec = (millis() - msPlayStarted) / 1000.0;
    // file is now playing in the 'background' so now's a good time
    // to do something else like handling LEDs or buttons :)
    msLastAction = millis();
    Serial.printf("\r%02lu:%02lu %s  ", sec / 60, sec % 60, musicPlayer.playingMusic ? "playing" : "stopped");
  }
}

int playTrack(const char* arg) {
  int n = atoi(arg);

  if (n > 0) {
    trackNumber = n;
    if (musicPlayer.playingMusic) {
      musicPlayer.stopPlaying();
    }
    needStart = true;
  }
  return trackNumber;
}

int setVolume(const char* arg) {
  int vol = atoi(arg);
  musicPlayer.setVolume(vol, vol);
  return vol;
}

int playSine(const char* arg) {
  int freq = atoi(arg);
  musicPlayer.sineTest(freq, 1000);            
  musicPlayer.reset();    
  return freq;
}

void blink(void) {
  digitalWriteFast(D7, !pinReadFast(D7));
}
