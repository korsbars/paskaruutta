//***********************************************************
//*  Extruder Driver by J. Koljonen & M. Oksanen            *
//*                                                         *
//*                                                         *
//***********************************************************

#include "ServoMod.h"

#define teensyLEDPin 13
#define filamentSensePin 23
#define VREF (3.300)
#define ADCMAX (65535)
#define updatePeriod 100000 //microseconds

boolean stringComplete = false;
byte bufferLength = 0;
long lastUpdateTime = 0;



void setup() {
  pinMode(teensyLEDPin, OUTPUT);      // LED to show Teensy is alive
  pinMode(filamentSensePin, INPUT);
  digitalWriteFast(teensyLEDPin, HIGH);
  analogReadRes(16);          // Set ADC resolution to this many bits
  analogReadAveraging(16);    // Average this many readings
  
 
  Serial.begin(9600);
  while (!Serial.dtr()) ;  // Wait for user to start the serial comms
  
  delay(250);
  Serial.println("INFO: Filament Extruder Driver v1.0");
  Serial.println("INFO: Initialization finished");
  Serial.println("INFO: # Syntax #");
}

void loop() {
  // If input data is available, SerialComm file handles it and writes stringComplete
  if (Serial.available()) serialRead();
  
  if (stringComplete) {
    //if (code_seen('F')) disableMotors();
    //if (getBlock('A', j0sp)) {}
    stringComplete = false;
    bufferLength = 0; 
  }

  if (micros() > (lastUpdateTime + updatePeriod)) {
    float filamentDiameter = (VREF/ADCMAX)*analogRead(filamentSensePin);
    Serial.print("MEAS: ");
    Serial.println(filamentDiameter);
    lastUpdateTime = micros();
  }
}

