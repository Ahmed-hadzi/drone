#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#define i2c_Address 0x3c

// DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// CHANNELS
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

  // DISPLAY SETUP
  display.begin(i2c_Address, true);
  display.display();
  delay(250);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 27);
  display.println("LOADING CHANNELS");
  display.setCursor(0, 54);
  display.println("Check battery level!");
  display.display();
  delay(1000);
  
  pinMode(13, OUTPUT);
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setCursor(0, 27);
  display.println("THROTTLE CHECK?");
  display.display();
    
  while(ch3val < 1020 || ch3val > 1050){
    
    ch3val = readChannel(CH3);
    delay(4);
  }
  
  display.clearDisplay();
  display.setCursor(0, 27);
  display.println("DISENGAGE DROP...");
  display.display();

  ch4val = readChannel(CH4);
  while(ch4val<1400){
    digitalWrite(13, HIGH);
    ch4val = readChannel(CH4);
    delay(100);
  }

  display.clearDisplay();
  display.setCursor(0, 27);
  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.println("Flight ready");
  display.display();

  for(int i = 0; i<5; i++){
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }

  servo.attach(10, 1000, 2000);
  display.clearDisplay();
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
