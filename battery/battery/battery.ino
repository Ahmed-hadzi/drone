float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed = 0;
float BatteryDefault = 3800; //mAh value of BATTERY

void battery_voltage(void){
  Voltage=(float)analogRead(A1)/40.23; //63.33
  Current=(float)analogRead(A5);
}

void setup() {
  Serial.begin(9600);

  delay(200);
  battery_voltage();
  if(Voltage>8.3){
    BatteryAtStart=BatteryDefault;
  } else if (Voltage<7.5){
    BatteryAtStart=30/100*BatteryDefault;
  } else{
    BatteryAtStart=(82*Voltage-580)/100*BatteryDefault;
  }
  Serial.print("VOLTAGE: ");
  Serial.println(Voltage);
  Serial.print("BATTERY AT START: ");
  Serial.println(BatteryAtStart);
  delay(1000);
}

void loop() {

  battery_voltage();
  CurrentConsumed=Current*1000*0.004/3600+CurrentConsumed;
  BatteryRemaining=(BatteryAtStart-CurrentConsumed)/BatteryDefault*100;
  if(BatteryRemaining<=30){
    Serial.println("LOWBAT");
  }

  Serial.print(Current);
  Serial.print(" A |   ");
  Serial.print(Voltage);
  Serial.print(" V |   REM: ");
  Serial.print(BatteryRemaining);
  Serial.print(" % |   CON: ");
  Serial.print(CurrentConsumed);
  Serial.println(" mAh");
  delay(4);
}
