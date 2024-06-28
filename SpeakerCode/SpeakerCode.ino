#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

AudioPlaySdWav           playWav1;
AudioOutputI2S           audioOutput;

AudioConnection          patchCord1(playWav1, 0, audioOutput, 0);
AudioConnection          patchCord2(playWav1, 1, audioOutput, 1);

// SD Card Definitions
#define SDCARD_CS_PIN    4
#define SDCARD_MOSI_PIN  11
#define SDCARD_SCK_PIN   13

// Definitions
int     button_switch =                       2; // external interrupt pin
int     start_pin =                           6;
bool    isPlay = false;


void setup() {
  pinMode(button_switch, INPUT);
  pinMode(start_pin, INPUT_PULLUP);
  Serial.begin(9600);
  AudioMemory(8);

  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a me ssage repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
}

void playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);

  // Start playing the file.  This sketch continues to
  // run while the file plays.
  playWav1.play(filename);

  // A brief delay for the library read WAV info
  delay(25);

  // Wait for the file to finish playing or switch to next song if button is pressed.
  while (playWav1.isPlaying()) {
    if (digitalRead(button_switch)) {
      Serial.print("change song\n");
      delay(100);
      break;
      }
  }
}


void loop() {
  if (!digitalRead(start_pin)) {
    Serial.print("pressed on\n");
    isPlay = true;
    delay (500);
    }
  if (isPlay) {
    playFile("SDTEST0.WAV");  // filenames are always uppercase 8.3 format
  delay(500);
  }
  if (isPlay) {
    playFile("SDTEST1.WAV");
  delay(500);
  }
  if (isPlay) {
    playFile("SDTEST2.WAV");
  delay(500);
  }
  if (isPlay) {
    playFile("SDTEST3.WAV");
  delay(500);
  }
  if (isPlay) {
    playFile("SDTEST4.WAV");
    Serial.print("end\n");
    isPlay = false;
    playWav1.togglePlayPause();
    delay (500);
    }
}
