const int pwmInputPin = A3;    // Analog pin used as digital input
const int pwmOutputPin = 3;    // Digital pin with PWM capability

int channelPins[4]={A3,A4,A5,A6};
float ReceiverValue[]={0,0,0,0};

int readChannel(int channelInput){
  int ch = pulseIn(channelInput, HIGH, 30000);
  return constrain(ch, 1000, 2000);
}

void read_receiver(void){
  for(int i = 0; i<4; i++){
    ReceiverValue[i] = readChannel(channelPins[i]);
  }
}

void setup() {
  for(int i = 0; i<4; i++){
    pinMode(channelPins[i], INPUT);
  }
  pinMode(pwmOutputPin, OUTPUT);

  Serial.begin(115200);
  Serial.println("BEGINNING");
}

void loop() {
  read_receiver();
  Serial.println(ReceiverValue[0]);
  analogWrite(3, ReceiverValue[0]);
  
}
