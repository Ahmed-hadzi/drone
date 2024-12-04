#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#define i2c_Address 0x3c

// DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool ch4comode = false;
bool comexc = false;

int channelPins[4]={14,15,16,17}; // Channel pinout in order: {CH1, CH2, CH3, CH4}

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
float PRateRoll = 0.6; float PRatePitch = PRateRoll; float PRateYaw = 2;

float IRateRoll = 3.5; float IRatePitch = IRateRoll; float IRateYaw = 12;
float DRateRoll = 0.03; float DRatePitch = DRateRoll; float DRateYaw = 0;
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


int ch1val, ch2val, ch3val, ch4val;


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
  Voltage = (float)analogRead(15)/62;
  Current=(float)analogRead(21)*0.089;
}

void read_receiver(void){
  for(int i = 0; i<4; i++){
    ReceiverValue[i] = readChannel(channelPins[i]);
  }
}

int readChannel(int channelInput){
  int ch = pulseIn(channelInput, HIGH, 30000);
  return constrain(ch, 1000, 2000);
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
  
  AccX = (float)AccXLSB/4096 - 0.04; // X CALIBRATION
  AccY = (float)AccYLSB/4096 + 0.02; // Y CALIBRATION
  AccZ = (float)AccZLSB/4096 - 0.07; // Z CALIBRATION

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
  display.println("Preparing for flight");
  display.display();
  delay(1000);

  for(int i = 0; i<4; i++){
    pinMode(channelPins[i], INPUT);
  }

  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  display.clearDisplay();
  display.setCursor(0, 27);
  display.println("CALIBRATING...");
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

  battery_voltage();
  if(Voltage>8.3){
    digitalWrite(5, LOW);
    BatteryAtStart=BatteryDefault;
  } else if (Voltage<7.5){
    BatteryAtStart=30/100*BatteryDefault;
  } else{
    digitalWrite(5, LOW);
    BatteryAtStart=(82*Voltage-580)/100*BatteryDefault;
  }

  display.clearDisplay();
  display.setCursor(0, 27);
  display.println("THROTTLE CHECK?");
  display.setCursor(0, 54);
  display.print("BATTERY: ");
  display.println(Voltage);
  display.display();

  while(ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050){
    read_receiver();
    delay(4);
  }

  display.clearDisplay();
  display.setCursor(0, 27);
  display.println("DISENGAGE DROP...");
  display.setCursor(0, 54);
  display.print("BATTERY: ");
  display.println(Voltage);
  display.display();

  read_receiver();
  while(ReceiverValue[4]<1400){
    digitalWrite(13, HIGH);
    read_receiver();
    delay(100);
  }

  display.clearDisplay();
  display.setCursor(0, 27);
  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.println("Flight ready");
  display.setCursor(0, 54);
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.print("BATTERY: ");
  display.println(Voltage);
  display.display();
  
  LoopTimer = micros();

  delay(1000);
  display.clearDisplay();

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

  read_receiver();

  DesiredAngleRoll=0.10*(ReceiverValue[0]-1500);
  DesiredAnglePitch = 0.10*(ReceiverValue[1]-1500);

  InputThrottle=ReceiverValue[3];
  if((ReceiverValue[4]>1200) && (ReceiverValue[4]<1300) && ch4comode && !comexc){
    Serial.println(" COMMAND EXECUTED ");
    comexc = true;
    // ACTIVATE COMMAND
    DesiredRateYaw=0;
  } else{
    DesiredRateYaw=0.15*((2*(ReceiverValue[4]-1500)+1000)-1500); // Normal yaw rate
  }

  if((ReceiverValue[4]==1000) && !comexc){
    ch4comode = true;
        digitalWrite(13, HIGH);
  } else{
    ch4comode = false;
        digitalWrite(13, LOW);
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

  analogWrite(1, MotorInput1);
  analogWrite(2, MotorInput2);
  analogWrite(3, MotorInput3);
  analogWrite(4, MotorInput4);

  battery_voltage();
  CurrentConsumed=Current*1000*0.004/3600+CurrentConsumed;
  BatteryRemaining=(BatteryAtStart-CurrentConsumed)/BatteryDefault*100;
  if(BatteryRemaining<=30) digitalWrite(5, HIGH);
  else digitalWrite(5, LOW);

  while(micros() - LoopTimer < 4000);
  LoopTimer=micros();


  Serial.print(" Motor 1: ");
  Serial.print(MotorInput1);
  Serial.print(" Motor 2: ");
  Serial.print(MotorInput2);
  Serial.print(" Motor 3: ");
  Serial.print(MotorInput3);
  Serial.print(" Motor 4: ");
  Serial.println(MotorInput4);

}
