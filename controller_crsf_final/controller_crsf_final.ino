#include <Wire.h> //gyro
#include <Servo.h> //Camera servo
#include <Adafruit_GFX.h> //Display
#include <Adafruit_SH110X.h>  //Display
#include <AlfredoCRSF.h> //CRSF communication

// DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // QT-PY / XIAO
#define i2c_Address 0x3c
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int serial_counter = 0;

// If battery <30%
bool lowbat = false;

// Arming-disarming variables
bool armed = false;
int armCounter = 0;
int displayCounter = 500;

// ESC calibration variables
bool esc_calibrated = false;
bool esc_calibrating = false;
int esc_counter = 0;

// Receiver communication
AlfredoCRSF crsf;

// Camera movement system
#define camPin 12
int desiredCamAngle = 0;
int actualCamAngle=0;
Servo camServo;

// TRIM
int TrimPitch = 0;
int TrimRoll = 0;
bool trim_set = false;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float ReceiverValue[]={0,0,0,0};

float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed = 0;
float BatteryDefault = 3800; //mAh value of BATTERY
uint32_t LoopTimer;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0,0,0};

// CALIBRATE
float PRateRoll = 0.6; float PRatePitch = PRateRoll; float PRateYaw = 2; // 0.6
float IRateRoll = 3.55; float IRatePitch = IRateRoll; float IRateYaw = 13; // 3.5
float DRateRoll = 0.03; float DRatePitch = DRateRoll; float DRateYaw = 0; // 0.03

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;
float Kalman1DOutput[] = {0,0};
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;

float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;

float PAngleRoll=2; float PAnglePitch = PAngleRoll;
float IAngleRoll = 0; float IAnglePitch = IAngleRoll;
float DAngleRoll = 0; float DAnglePitch = DAngleRoll;



void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement){
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004*0.004*4*4;
  float KalmanGain = KalmanUncertainty * 1/(1*KalmanUncertainty+3*3);
  KalmanState=KalmanState+KalmanGain*(KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain)*KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState;
  Kalman1DOutput[1]=KalmanUncertainty;
}

void battery_voltage(void){
  Voltage=(float)analogRead(15)/63.33;
  Current=(float)analogRead(21)*0.4;
}

void gyro_signal(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;
  
  AccX = (float)AccXLSB/4096 - 0.11; // X CALIBRATION
  AccY = (float)AccYLSB/4096 + 0.01; // Y CALIBRATION
  AccZ = (float)AccZLSB/4096 - 0.19; // Z CALIBRATION

  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch = -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}


void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm){
  float Pterm=P*Error;
  float Iterm = PrevIterm+1*(Error+PrevError)*0.004/2;
  if(Iterm>400) Iterm = 400;
  if(Iterm<-400) Iterm = -400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput = Pterm+Iterm+Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  if(PIDOutput<-400) PIDOutput = -400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(){
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0;PrevItermRatePitch=0;PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}

void moveCamera(){
  desiredCamAngle = constrain(crsf.getChannel(6), 1000, 2000);

  if(abs(desiredCamAngle-actualCamAngle)<20){
    return;
  } else{
  actualCamAngle = desiredCamAngle;
  if(desiredCamAngle < 950){ // NO SIGNAL
    desiredCamAngle = 1500;
  } else
  if(desiredCamAngle >= 950){ // Normal signal
    desiredCamAngle=map(desiredCamAngle, 1000, 2000, 1000, 2000);
  }
  camServo.writeMicroseconds(desiredCamAngle);
  }
}

static void TelemetryBattery(float voltage, float current, float capacity, float remaining)
{
  crsf_sensor_battery_t crsfBatt = { 0 };
  crsfBatt.voltage = htobe16((uint16_t)(voltage * 10.0));   //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;   //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);                //percent
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}

void setup(){
  Serial.begin(115200);

  Serial2.begin(400000);
  crsf.begin(Serial2);

  // DISPLAY SETUP
  display.begin(i2c_Address, true);
  display.display();
  delay(250);
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 27);
  display.println("LOADING CHANNELS");
  display.setCursor(0, 54);
  display.println("Preparing for flight");
  display.display();
  delay(1000);

  pinMode(15, INPUT);
  pinMode(21, INPUT);

  camServo.attach(camPin, 1000, 2000);

  pinMode(camPin, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  display.clearDisplay();

  battery_voltage();
  if(Voltage>8.3){
    BatteryAtStart=BatteryDefault;
  } else if (Voltage<7.5){
    lowbat=true;
    BatteryAtStart=30/100*BatteryDefault;
  } else{
    BatteryAtStart=(82*Voltage-580)/100*BatteryDefault;
  }
  TelemetryBattery(Voltage, 0, BatteryDefault, BatteryAtStart);

  display.clearDisplay();
  if(lowbat){
    display.setCursor(0, 9);
    display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    display.println("BATTERY LOW");
  }
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setCursor(0, 27);
  display.println("CALIBRATING...");
  display.setCursor(0, 54);
  display.print("BATTERY: ");
  display.print(Voltage);
  display.println("V");
  display.display();

  for(RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++){
    gyro_signal();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;

  analogWriteFrequency(1, 250);
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);
  analogWriteResolution(12);

  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

  display.clearDisplay();
  if(lowbat){
    display.setCursor(0, 9);
    display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    display.println("BATTERY LOW");
  }
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setCursor(0, 27);
  display.println("THROTTLE CHECK?");
  display.setCursor(0, 36);
  display.println("Flight mode ready...");
  display.setCursor(0, 54);
  display.print("BATTERY: ");
  display.print(Voltage);
  display.println("V");
  display.display();

  crsf.update();
  int throttlecheck = crsf.getChannel(3);
  while(throttlecheck < 1450 || throttlecheck > 1550){
    crsf.update();
    throttlecheck = crsf.getChannel(3);
    Serial.println(throttlecheck);
    delay(4);
  }

  display.clearDisplay();
  if(lowbat){
    display.setCursor(0, 9);
    display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    display.println("BATTERY LOW");
  }
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setCursor(0, 27);
  display.println("DISENGAGE DROP...");
  display.setCursor(0, 54);
  display.print("BATTERY: ");
  display.print(Voltage);
  display.println("V");
  display.display();

  crsf.update();
  int dropcheck = crsf.getChannel(5);
  while(dropcheck<1400){
    crsf.update();
    digitalWrite(13, HIGH);
    dropcheck = constrain(crsf.getChannel(5), 1000, 2000);
    delay(100);
  }
  digitalWrite(13, LOW);

  if(!lowbat){
    digitalWrite(5, LOW);
  }

  display.clearDisplay();
  if(lowbat){
    display.setCursor(0, 9);
    display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    display.println("BATTERY LOW");
  }
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setCursor(0, 27);
  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.println("FLIGHT MODE");
  display.setCursor(0, 54);
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.print("BATTERY: ");
  display.print(Voltage);
  display.println("V");
  display.display();
  
  LoopTimer = micros();

  delay(1000);
  display.clearDisplay();
  display.display();

}

void loop() {
  gyro_signal();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

  DesiredAngleRoll=0.10*(ReceiverValue[0]-1500);
  DesiredAnglePitch = 0.10*(ReceiverValue[1]-1500);

  if((DesiredAngleRoll<1) && (DesiredAngleRoll>-1)){
    DesiredAngleRoll=0;
  }

  if((DesiredAnglePitch<1) && (DesiredAnglePitch>-1)){
    DesiredAnglePitch=0;
  }

  crsf.update();

  ReceiverValue[0] = constrain(crsf.getChannel(1), 1000, 2000) + TrimRoll;
  ReceiverValue[1] = constrain(crsf.getChannel(2), 1000, 2000) + TrimPitch;
  ReceiverValue[2] = constrain(crsf.getChannel(3), 1000, 2000);
  ReceiverValue[3] = constrain(crsf.getChannel(4), 1000, 2000);
  
  InputThrottle=ReceiverValue[2];
  DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
  if((DesiredRateYaw<1)&&(DesiredRateYaw>-1)){
    DesiredRateYaw = 0;
  }

  ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
  ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;

  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll=PIDReturn[0];
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];
  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch=PIDReturn[0];
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];

  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll=PIDReturn[0];
  PrevErrorRateRoll=PIDReturn[1];
  PrevItermRateRoll=PIDReturn[2];
  
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch=PIDReturn[0];
  PrevErrorRatePitch=PIDReturn[1];
  PrevItermRatePitch=PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw=PIDReturn[0];
  PrevErrorRateYaw=PIDReturn[1];
  PrevItermRateYaw=PIDReturn[2];

  if(InputThrottle > 1800) InputThrottle = 1800;

  MotorInput1=1.024*(InputThrottle-InputRoll-InputPitch-InputYaw);
  MotorInput2=1.024*(InputThrottle-InputRoll+InputPitch+InputYaw);
  MotorInput3=1.024*(InputThrottle+InputRoll+InputPitch-InputYaw);
  MotorInput4=1.024*(InputThrottle+InputRoll-InputPitch+InputYaw);

  if(MotorInput1 > 2000) MotorInput1 = 1999;
  if(MotorInput2 > 2000) MotorInput2 = 1999;
  if(MotorInput3 > 2000) MotorInput3 = 1999;
  if(MotorInput4 > 2000) MotorInput4 = 1999;
  
  int ThrottleIdle=1180;

  if(MotorInput1 < ThrottleIdle) MotorInput1=ThrottleIdle;
  if(MotorInput2 < ThrottleIdle) MotorInput2=ThrottleIdle;
  if(MotorInput3 < ThrottleIdle) MotorInput3=ThrottleIdle;
  if(MotorInput4 < ThrottleIdle) MotorInput4=ThrottleIdle;

  int ThrottleCutOff=1000;
  if(ReceiverValue[2]<1050){
    MotorInput1=ThrottleCutOff;
    MotorInput2=ThrottleCutOff;
    MotorInput3=ThrottleCutOff;
    MotorInput4=ThrottleCutOff;
    reset_pid();
  }

  // DRONE IS NOT ARMED
  if(!armed){
    //Avoiding display slowdown
    if(displayCounter>=500){
    display.clearDisplay();
    display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    display.setCursor(0, 27);
    display.println("DISARMED");
    display.setCursor(0, 54);
    display.setTextColor(SH110X_WHITE, SH110X_BLACK);
    if(lowbat){
      display.print("LOW BATTERY ");
      display.println("");
    } else{
      display.print("BATTERY: ");
      display.print(BatteryRemaining);
      display.println("%");
    }
    display.display();
    displayCounter=0;
    }
    displayCounter++;
    if((InputThrottle<1050) && (DesiredRateYaw > 47) && (!esc_calibrating)){
      armCounter++;
    } else{
      armCounter=0;
    }
    if(armCounter>200){
      armed=true;
      esc_calibrated = true;
      display.clearDisplay();
      display.display();
      displayCounter=500;
      armCounter=0;
    }

    if((ReceiverValue[2]>1990) && (DesiredRateYaw < -47) && (!esc_calibrated) && (!esc_calibrating)){
      esc_counter++;
    } else if(!esc_calibrating){
      esc_counter=0;
    }

    if(esc_counter>200){
      esc_calibrating = true;
      displayCounter=500;
      esc_counter=0;
    }

    // TRIM SETTING MODE
    while (crsf.getChannel(7)>1700){
      crsf.update();
      
      if(((crsf.getChannel(1) < 1550) && (crsf.getChannel(1)>1450)) && ((crsf.getChannel(2) < 1550) && (crsf.getChannel(2)>1450))){
        trim_set = false;
      }
      
      if((!trim_set)&&(crsf.getChannel(1)>1900)){
        TrimRoll+=10;
        trim_set = true;
      }
      if((!trim_set)&&(crsf.getChannel(1)<1100)){
        TrimRoll-=10;
        trim_set = true;
      }
      if((!trim_set)&&(crsf.getChannel(2)>1900)){
        TrimPitch+=10;
        trim_set = true;
      }
      if((!trim_set)&&(crsf.getChannel(2)<1100)){
        TrimPitch-=10;
        trim_set = true;
      }

      if(displayCounter>=500){
         display.clearDisplay();
         display.setTextColor(SH110X_WHITE, SH110X_BLACK);
         display.setCursor(0, 0);
         display.print("Pitch Trim: ");
         display.println(TrimPitch);
         display.setCursor(0, 9);
         display.print("Roll Trim: ");
         display.println(TrimRoll);
         display.setTextColor(SH110X_BLACK, SH110X_WHITE);
         display.setCursor(0, 27);
         display.println("DISARMED");
         display.setCursor(0, 54);
         display.setTextColor(SH110X_WHITE, SH110X_BLACK);
       if(lowbat){
         display.print("LOW BATTERY ");
         display.println("");
       } else{
          display.print("BATTERY: ");
          display.print(BatteryRemaining);
          display.println("%");
       }
          display.display();
          displayCounter=0;
       }
       displayCounter++;
    }
  }

  // DRONE IS IN ESC CALIBRATION MODE
  if(esc_calibrating && !esc_calibrated){
    if(displayCounter>=500){
      display.clearDisplay();
      display.setTextColor(SH110X_WHITE, SH110X_BLACK);
      display.setCursor(0, 27);
      display.println("ESC CALIBRATION");
      display.setCursor(0, 54);
      display.setTextColor(SH110X_WHITE, SH110X_BLACK);
      if(lowbat){
        display.print("LOW BATTERY ");
        display.println("");
      } else{
        display.print("BATTERY: ");
        display.print(BatteryRemaining);
        display.println("%");
      }
      display.display();
      displayCounter=0;
    }
    displayCounter++;
    analogWrite(1, ReceiverValue[2]);
    analogWrite(2, ReceiverValue[2]);
    analogWrite(3, ReceiverValue[2]);
    analogWrite(4, ReceiverValue[2]);
    if((ReceiverValue[2]<1050) && (DesiredRateYaw > 47)){
      esc_counter++;
      Serial.println(esc_counter);
    } else{
      esc_counter=0;
    }
    Serial.print(ReceiverValue[2]);
    Serial.print("      ");
    Serial.println(DesiredRateYaw);
    if(esc_counter>200){
      esc_calibrating = false;
      esc_calibrated = true;
      displayCounter=500;
      esc_counter=0;
      display.clearDisplay();
      display.display();
    }
  }
  
  // DRONE IS ARMED
  if(armed){
    analogWrite(1, MotorInput1);
    analogWrite(2, MotorInput2);
    analogWrite(3, MotorInput3);
    analogWrite(4, MotorInput4);
    if((ReceiverValue[2]<1050) && (DesiredRateYaw < -47)){
      armCounter++;
    } else{
      armCounter=0;
    }
    if(armCounter>200){
      armed=false;
      armCounter=0;
    }
  }
  moveCamera();

  battery_voltage();
  CurrentConsumed=Current*1000*0.004/3600+CurrentConsumed;
  BatteryRemaining=(BatteryAtStart-CurrentConsumed)/BatteryDefault*100;
  if(BatteryRemaining<=30){
    lowbat=true;
    digitalWrite(5, HIGH);
  }
  else{
    digitalWrite(5, LOW);
    lowbat = false;
  }
  TelemetryBattery(Voltage, Current, BatteryDefault, BatteryRemaining);

  /*display.clearDisplay();
  display.setCursor(0, 54);
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.print("BATTERY: ");
  display.print(BatteryRemaining);
  display.println("%");
  display.display();*/

  while(micros() - LoopTimer < 4000);
  LoopTimer=micros();

  /*Serial.print("Motor_1:");
  Serial.print(MotorInput1);
  Serial.print(" Motor_2:");
  Serial.print(MotorInput2);
  Serial.print(" Motor_3:");
  Serial.print(MotorInput3);
  Serial.print(" Motor_4:");
  Serial.println(MotorInput4);*/

  /*Serial.print("X: ");
  Serial.print(AccX);
  Serial.print(" | Y: ");
  Serial.print(AccY);
  Serial.print(" | Z: ");
  Serial.println(AccZ);*/

  /*Serial.print("Roll:");
  Serial.print(ReceiverValue[0]);
  Serial.print(" | Pitch:");
  Serial.println(ReceiverValue[1]);*/
  //printChannels();

  Serial.println(serial_counter);
  serial_counter++;
}

void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 14; ChannelNum++)
  {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}
