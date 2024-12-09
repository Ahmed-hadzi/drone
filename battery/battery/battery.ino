float Voltage;

void battery_voltage(){
    Voltage=(float)analogRead(15)/33.325; // CUSTOM BATTERY DIVIDER
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  battery_voltage();
  Serial.println(Voltage);
  delay(50);
}
