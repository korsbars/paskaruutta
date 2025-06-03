

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin0 = A0;  
const int analogInPin1 = A1;
const int analogOutPin0 = 9;
const int analogOutPin1 = 10;

int sensor0Value, sensor1Value = 0;        // value read from the pot
int output0Value, output1Value = 0;        // value output to the PWM (analog out)

void setup() {
  analogReference(INTERNAL);
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
}

void loop() {
  // read the analog in value:
  sensor0Value = analogRead(analogInPin0);     
  sensor1Value = analogRead(analogInPin1);   
  // map it to the range of the analog out:
  output0Value = map(sensor0Value, 0, 1023, 0, 255);  
  output1Value = map(sensor1Value, 0, 1023, 0, 255); 
  // change the analog out value:
  analogWrite(analogOutPin0, output0Value);  
  analogWrite(analogOutPin1, output1Value);  

  // print the results to the serial monitor:
  Serial.print("Sensor 0 = " );                       
  Serial.print(sensor0Value);     
  Serial.print(" Sensor 1 = " );                       
  Serial.print(sensor1Value); 
  Serial.print("\t Output 0 = ");      
  Serial.print(output0Value);   
  Serial.print(" Output 1 = ");      
  Serial.println(output1Value);  

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);                     
}
