#include <LiquidCrystal.h>
#include "DFR_Key.h"

//Pin assignments for SainSmart LCD Keypad Shield
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); 


//Hall blue goes to pin 2
//Motor PWM runs on pin 3

//Hall brown = VCC
//Hall green = GND

DFR_Key keypad;

int localKey = 0;
String keyString = "";

double third_revolutions;
double rps;
double revs;
double count_rpm;
double set_rps;
unsigned long timeold;
                 
void setup() 
{ 
  pinMode(2, INPUT_PULLUP);
  pinMode(3, OUTPUT);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("FilamentExtruder");
  lcd.setCursor(0, 1);
  lcd.print("v 0.1");
  delay(2500);

  keypad.setRate(20);
  
  attachInterrupt(0, rpm_fun, FALLING);
  third_revolutions = 0;
  rps = 0;
  revs = 0;
  set_rps = 10.0;
  count_rpm = 0;
  timeold = 0;
  
  
  
  lcd.clear();
}

void loop() { 

  if (count_rpm >= 10) { 
    //Update RPM every 10 counts, increase this for better RPM resolution,
    //decrease for faster update
    rps = (count_rpm*1000)/((millis() - timeold));
    timeold = millis();
    count_rpm = 0;
    lcd.setCursor(0,1);
    lcd.print("RPS: ");
    lcd.print(rps,2);
  }

  //Display where we are going..
  lcd.setCursor(0,0);
  lcd.print("Turns: ");
  lcd.print(revs,1);
  
  //Display the current angle of extruder puller motor  
  lcd.setCursor(15,1);
  if (third_revolutions < 1) lcd.print("|");
  else if (third_revolutions < 2) lcd.print("/");
  else if (third_revolutions <= 3) lcd.print("-");
  
  //pulse motor with 150/255 PWM
  analogWrite(3, 150);

  delay(100);  
  
  
// Key polling and setting RPS according to it  
//  lcd.setCursor(5,1);
//  lcd.print(set_rps,1);
//  
//  //Poll key
//  localKey = keypad.getKey();
//  if (localKey == -1) set_rps = set_rps + 0.1;
//  else if (localKey == -1) set_rps = set_rps - 0.1;
}

//Each rotation, this interrupt function is run thrice
void rpm_fun() {
  //Calculate added revolutions (gearing 1/29.75)
  third_revolutions = third_revolutions + 1.0/75.0;
  if (third_revolutions > 3) third_revolutions = 0;
  
  //Count for RPM calculation
  revs = revs + 1.0/(75.0*3);
  count_rpm = count_rpm + 1.0/(75.0*3);
 }
