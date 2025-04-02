#include <Servo.h>
#define campin 12

Servo camservo;

void setup() {
  // put your setup code here, to run once:
  pinMode(12, OUTPUT);
  camservo.attach(campin, 500, 2500);
  camservo.writeMicroseconds(500);
  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
  for(int  i = 2500; i>500;i=i-10){
    camservo.writeMicroseconds(i);
    delay(50);
  }

}
