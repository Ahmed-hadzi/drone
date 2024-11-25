#include <Servo.h>

Servo esc;
int readChannel(int channelInput){
  int ch = pulseIn(channelInput, HIGH, 30000);
  return constrain(ch, 1000, 2000);
}
#define pin 1
int motor = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(16, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  esc.attach(pin, 1000, 2000);
  //analogWriteFrequency(pin, 250);
  //analogWriteResolution(12);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor = readChannel(16);
  Serial.println(motor);
  if(motor>1100){
    digitalWrite(13,HIGH);
  } else{
    digitalWrite(13,LOW);
  }
  esc.writeMicroseconds(motor);
}
