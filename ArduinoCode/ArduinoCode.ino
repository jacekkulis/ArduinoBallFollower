#include "PlatformDefinitions.h"

volatile int analogWriteRightEngine = 0;
volatile int analogWriteLeftEngine = 0;

void brake();
void rideForward();
void setupEngines();

void setup() {
  Serial.begin(115200);
  setupEngines();
}

void loop() {
  
  while (Serial.available() > 0)
  {
    char incommingByte = Serial.read();
    if (incommingByte == 'g'){
      rideForward();
    } 
    
    if (incommingByte == 'r') {
      analogWriteRightEngine = 100;
      analogWriteLeftEngine = 255;
      analogWrite(LEFT_FRONT_ENGINE, analogWriteLeftEngine);
      analogWrite(RIGHT_FRONT_ENGINE, analogWriteRightEngine);
    } 
    
    if (incommingByte == 'l') {
      analogWriteLeftEngine = 100;
      analogWriteRightEngine = 255;
      analogWrite(LEFT_FRONT_ENGINE, analogWriteLeftEngine);
      analogWrite(RIGHT_FRONT_ENGINE, analogWriteRightEngine);
    }
    
    if (incommingByte == 's') {
      brake();
    }
  }
}

void rideForward() {
  analogWriteRightEngine = 255;
  analogWriteLeftEngine = 255;
  analogWrite(RIGHT_FRONT_ENGINE, analogWriteRightEngine);
  analogWrite(LEFT_FRONT_ENGINE, analogWriteLeftEngine);
}

void brake() {
  analogWriteRightEngine = 0;
  analogWriteLeftEngine = 0;
  analogWrite(RIGHT_FRONT_ENGINE, analogWriteRightEngine);
  analogWrite(LEFT_FRONT_ENGINE, analogWriteLeftEngine);
  analogWrite(RIGHT_BACK_ENGINE, analogWriteRightEngine);
  analogWrite(LEFT_BACK_ENGINE, analogWriteLeftEngine);
}

void setupEngines() {
  pinMode(RIGHT_FRONT_ENGINE, OUTPUT);
  pinMode(LEFT_FRONT_ENGINE, OUTPUT);
  pinMode(RIGHT_BACK_ENGINE, OUTPUT);
  pinMode(LEFT_BACK_ENGINE, OUTPUT);
}
