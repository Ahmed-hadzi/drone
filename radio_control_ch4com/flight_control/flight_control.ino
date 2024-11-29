#include <Servo.h>

#define CH1 14
#define CH2 15
#define CH3 16
#define CH4 17

bool ch4comode = false;
bool comexc = false;

Servo servo;

int ch1val, ch2val, ch3val, ch4val;

int readChannel(int channelInput){
  int ch = pulseIn(channelInput, HIGH, 30000);
  return constrain(ch, 1000, 2000);
}


void setup(){
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);

  ch4val = readChannel(CH4);
  while(ch4val<1400){
    digitalWrite(13, HIGH);
    ch4val = readChannel(CH4);
    delay(100);
  }

  for(int i = 0; i<5; i++){
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }

  servo.attach(10, 1000, 2000);
}

void loop() {
  ch1val = readChannel(CH1);
  ch2val = readChannel(CH2);
  ch3val = readChannel(CH3);
  ch4val = readChannel(CH4);

  if((ch4val>1200) && (ch4val<1300) && ch4comode && !comexc){
    Serial.println(" COMMAND EXECUTED ");
    comexc = true;
    delay(3000);
  }

  if((ch4val==1000) && !comexc){
    ch4comode = true;
        digitalWrite(13, HIGH);
  } else{
    ch4comode = false;
        digitalWrite(13, LOW);
  }

  if(ch4val>1400){
    ch4val = 2*(ch4val-1500)+1000;
  }
  
  servo.writeMicroseconds(ch3val);

  Serial.print("Ch1: ");
  Serial.print(ch1val);
  Serial.print(" Ch2: ");
  Serial.print(ch2val);
  Serial.print(" Ch3: ");
  Serial.print(ch3val);
  Serial.print(" Ch4: ");
  Serial.print(ch4val);
  if(ch4comode){
    Serial.print(" Command mode ");
  }
  Serial.println("");

}
