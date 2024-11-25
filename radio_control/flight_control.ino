#include <Servo.h>

#define CH1 3
#define CH2 5
#define CH3 6
#define CH4 9

#define throttle CH1

Servo servo;

int ch1val, ch2val, ch3val, ch4val;

int readChannel(int channelInput){
  int ch = pulseIn(channelInput, HIGH, 30000);
  return constrain(ch, 1000, 2000);
}


void setup(){
  Serial.begin(115200);
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);

  servo.attach(10, 1000, 2000);
}

void loop() {
  ch1val = readChannel(CH1);
  ch2val = readChannel(CH2);
  ch3val = readChannel(CH3);
  ch4val = readChannel(CH4);
  
  servo.writeMicroseconds(ch3val);

  Serial.print("Ch1: ");
  Serial.print(ch1val);
  Serial.print(" Ch2: ");
  Serial.print(ch2val);
  Serial.print(" Ch3: ");
  Serial.print(ch3val);
  Serial.print(" Ch4: ");
  Serial.println(ch4val);

}