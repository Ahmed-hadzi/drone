#include <Servo.h>
 
Servo ESC;     // create servo object to control the ESC
 
const int pwmPin =  1;      // PWM pin
 
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in Âµs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in Âµs
 
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT); 
 
 
  delay(2000);
  digitalWrite(13, HIGH);  
  delay(1000);
  digitalWrite(13, LOW);
  ESC.attach(pwmPin, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
 
  delay(5000);
 
  digitalWrite(13, HIGH);
  /*Serial.println("Calibratee: HIGH");
  ESC.writeMicroseconds(MAX_PULSE_LENGTH);
  delay(2000);*/
   Serial.println("Move HIGH");
  ESC.writeMicroseconds(1800);
 
  delay(5000);
 
  digitalWrite(13, LOW);
  Serial.println("Move LOW");
  ESC.writeMicroseconds(1200);
  /*delay(2000);
  Serial.println("Calibrate: LOW");
  ESC.writeMicroseconds(MIN_PULSE_LENGTH);*/
 
 
 
  delay(5000);
 
  digitalWrite(13, HIGH);
  Serial.println("Stop");
  ESC.writeMicroseconds(1500);   
}
 
void loop() {
 
}
