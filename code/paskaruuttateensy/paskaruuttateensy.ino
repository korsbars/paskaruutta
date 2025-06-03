// Use v1.2.1 of newLiquidCrystal_I2C!!!

#include <Wire.h>
#include "LiquidCrystal_I2C.h"
//#include <AccelStepper.h>
#include <FrequencyTimer2.h>

// i2c-osoite kannattaa selvittää ennen kiinteää asennusta laitteeseen...
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define motorOnButton 1
#define heatOnButton 0
#define filamentPullerOnButton 2
#define filamentPullerAutomaticButton 3
#define spoolerOnButton 4
//#define spoolerAutomaticButton 5
#define spoolerAutomaticButton 22
#define ledPin 13

#define pullerOff 0
#define pullerOn 1
#define pullerAuto 2

#define pullerConstantSpeed 5

#define heaterSSRPin 14
#define motorFetRelayPin 11

#define filamentWidthADCPin 9
float filamentADCVal = 0;

#define stepperDriverEnablePin 20
#define stepperDriverDirPin 21
//#define stepperDriverStepPin 22
#define stepperDriverStepPin 5
// tuota yllä olevaa stepperDriverStepPinniä ei ny koodi tarvi mihinkää mutta onpahan dokumentoitu

#define pullerStepPerUnit 1000

#define buttonUpdatePeriod 200
#define pullerUpdatePeriod 250
#define lcdUpdatePeriod 1000

//AccelStepper pullerMotor(1, stepperDriverStepPin, stepperDriverDirPin);

unsigned long lastButtonUpdate = 0;
unsigned long lastPullerUpdate = 0;
unsigned long lastLcdUpdate = 0;
int pullerStatus = 0;
int ledStatus = 0;

int motorSpeedCommand = 0;

float diaSp = 1.75;
float diaKp = 3000.0;
float diaKi = 1.0;
float diaKd = 0.0;
float diaErrSum = 0.0;
float lastDiaErr = 0.0;

boolean stringComplete = false;
byte bufferLength = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  ledStatus=1;
  pinMode(motorOnButton, INPUT_PULLUP);
  pinMode(heatOnButton, INPUT_PULLUP);
  pinMode(filamentPullerOnButton, INPUT_PULLUP);
  pinMode(filamentPullerAutomaticButton, INPUT_PULLUP);
  pinMode(spoolerOnButton, INPUT_PULLUP);
  pinMode(spoolerAutomaticButton, INPUT_PULLUP);

  pinMode(stepperDriverEnablePin, OUTPUT);
  digitalWrite(stepperDriverEnablePin, HIGH); // active low
  //pinMode(stepperDriverStepPin, OUTPUT);
  pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
  pinMode(stepperDriverDirPin, OUTPUT);
  digitalWrite(stepperDriverDirPin, HIGH);

  pinMode(heaterSSRPin, OUTPUT);
  pinMode(motorFetRelayPin, OUTPUT);
  digitalWrite(heaterSSRPin, LOW);
  digitalWrite(motorFetRelayPin, LOW);

  // analog read stuff
  analogReference(DEFAULT);
  analogReadResolution(12);
  analogReadAveraging(32);

  //pullerMotor.setMaxSpeed(12000);
  //pullerMotor.setAcceleration(10);
  //pullerMotor.setSpeed(0);
  FrequencyTimer2::setPeriod(0);
  FrequencyTimer2::disable();
  
  lcd.begin(20, 4);
  lcd.home();

  //attachInterrupt(downButton, isrDownButton, FALLING);
  //attachInterrupt(upButton, isrUpButton, FALLING);

  lcd.setCursor(0, 0);
  lcd.print("Heat ");
  lcd.setCursor(7, 0);
  lcd.print("Dist: ");
  lcd.setCursor(0, 2);
  lcd.print("Width: ");

  Serial.begin(115200);
  delay(200);
  Serial.println("FilamentPuller controller 1.0 initialized");
}


void loop() {
  if (Serial.available()) serialRead();

  if (stringComplete) {
    if (getBlock('S', diaSp))
      diaSp /= 100.0;
    if (getBlock('P', diaKp))
      diaKp /= 100.0;
    if (getBlock('I', diaKi))
      diaKi /= 100.0;
    if (getBlock('D', diaKd))
      diaKd /= 100.0;

    // Jotain on tullut sisään...
    // Serialista pitäisi lukea
    // (protokolla varmaan kaikissa luokkaa diaSp*100 eli tässä päässä jaetaan sadalla ja tallennetaan floattiin. Siten saadaan inttinä portin läpi esim 1.75
    //if (getBlock('S', tempDiaSp)) {diaSp = tempDiaSp/100.0;}
    // diaSp
    // diaKp
    // diaKi
    // diaKd

    // Jos serialista tulee vaikka kirjain Q (query), postaa sinne noi em. arvot
    if (code_seen('Q')){
      Serial.print("PID P:");
      Serial.print(diaKp);
      Serial.print(" I:");
      Serial.print(diaKi);
      Serial.print(" D:");
      Serial.print(diaKd);
    }
  }

  if (millis() > (lastButtonUpdate + buttonUpdatePeriod)) {
    readButtons();
    lastButtonUpdate = millis();
  }

  if (millis() > (lastLcdUpdate + lcdUpdatePeriod)) {
    writeLcd();
    lastLcdUpdate = millis();
    if(ledStatus) {
      ledStatus = 0;
      digitalWrite(ledPin, LOW);
    } else {
      ledStatus=1;
      digitalWrite(ledPin, HIGH);
    }
  }

  if (millis() > (lastPullerUpdate + pullerUpdatePeriod)) {
    if (pullerStatus == pullerAuto) {
      runPullerSpeedControl();
    }
    lastPullerUpdate = millis();
  }

  //pullerMotor.runSpeed();
}

void writeLcd(void) {
  // Filamentwidthluku, adc luku
  //lcd.setCursor(7, 2);
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  float width = (float)3.3*analogRead(filamentWidthADCPin)/(float)4095;
//  width=width*1000;
//  lcd.setCursor(0,2);
//  lcd.print((String)width);
  
  String res = "";
  if (width < 0) {
    res = "0";
  } else if (width < 1) {
    res = "0.";
  } else if (width < 2) {
    res = "1.";
    width = width - 1.0;
  } else if (width < 3) {
    res = "2.";
    width = width - 2.0;
  } else if (width < 4) {
    res = "3.";
    width = width - 3.0;
  }

  width *= 1000;
  int dec = (int)width;
 
  lcd.print(res);
  lcd.print(dec);
  
  //lcd.print((String)(3.3 * analogRead(filamentWidthADCPin) / 4095));

  // Give motor time to run..
  // pullerMotor.runSpeed();

  // Ekstruuderin kuljettu matka
  lcd.setCursor(13, 0);
  // FrequencyTimer2 ei osaa laskea steppejä.
  //lcd.print(pullerMotor.currentPosition() / pullerStepPerUnit);

  // säätimen tietoa näytölle
  lcd.setCursor(0, 3);
  lcd.print((String)(diaErrSum));
  lcd.print(" ");
  lcd.print((int) motorSpeedCommand);
  lcd.print(" ");
}

void runPullerSpeedControl(void) {


  float filamentDia = 3.3 * analogRead(filamentWidthADCPin) / 4095;
  float diaErr = filamentDia - diaSp;
  
  diaErrSum += (diaErr * ((float)pullerUpdatePeriod/500.0));
  diaErrSum = constrain(diaErrSum, -1000, 1000);

  float dErr = (diaErr - lastDiaErr) / pullerUpdatePeriod;
  dErr = constrain(dErr, -50, 50);
  lastDiaErr = diaErr;

  motorSpeedCommand = constrain(diaSp * diaKp + diaErrSum * diaKi + dErr * diaKd, 0, 20000);
  
  //pullerMotor.setSpeed(motorSpeedCommand);
  FrequencyTimer2::setPeriod((int)((1000000.0/(float)motorSpeedCommand))+0.5);
}

void setPuller(int state) {
  switch (state) {
    case pullerOff:
      if (pullerStatus != pullerOff) {
        // Disable puller
        digitalWriteFast(stepperDriverEnablePin, HIGH); // active low
        FrequencyTimer2::setPeriod(0);
        FrequencyTimer2::disable();
        //pullerMotor.setSpeed(0);
        pullerStatus = pullerOff;
      }
      break;
    case pullerOn:
      if (pullerStatus != pullerOn) {
        // Enable puller
        digitalWriteFast(stepperDriverEnablePin, LOW); // active low
        //pullerMotor.setSpeed(pullerStepPerUnit * pullerConstantSpeed);
        // TESTATAAN 500US PERIODI -> ABOUT 10 RPM
        FrequencyTimer2::setPeriod((int)((1000000.0/(float)(pullerStepPerUnit * pullerConstantSpeed)+0.5)));

        FrequencyTimer2::enable();

        pullerStatus = pullerOn;
      }
      break;
    case pullerAuto:
      if (pullerStatus != pullerAuto) {
        // Enable puller, let PID set velocity
        FrequencyTimer2::enable();
        digitalWriteFast(stepperDriverEnablePin, LOW); // active low
        pullerStatus = pullerAuto;
      }
      break;
    default:
      break;
  }
}

void readButtons(void) {
  if (digitalReadFast(motorOnButton) == HIGH) {
    // motor fet relay off
    digitalWriteFast(motorFetRelayPin, LOW);
  } else {
    // motor fet relay off
    digitalWriteFast(motorFetRelayPin, HIGH);
  }

  if (digitalReadFast(heatOnButton) == HIGH) {
    // turn heat SSR off
    digitalWriteFast(heaterSSRPin, LOW);
    lcd.setCursor(5, 0);
    lcd.print("0");
  } else {
    // turn heat SSR on
    digitalWriteFast(heaterSSRPin, HIGH);
    lcd.setCursor(5, 0);
    lcd.print("1");
  }

  if ((digitalReadFast(filamentPullerOnButton) == HIGH) && (digitalReadFast(filamentPullerAutomaticButton) == HIGH)) {
    // disable puller
    diaErrSum = 0.0;
    setPuller(pullerOff);
  }

  if (digitalReadFast(filamentPullerOnButton) == LOW) {
    // enable puller
    setPuller(pullerOn);
  }

  if (digitalReadFast(filamentPullerAutomaticButton) == LOW) {
    // enable puller auto
    setPuller(pullerAuto);
  }

  /* if (digitalReadFast(spoolerOnButton) == HIGH) {
    //Nothing
    } else {
    //Nothing
    }

    if (digitalReadFast(spoolerAutomaticButton) == HIGH) {
    //Nothing
    } else {
    //Nothing
    } */
}

