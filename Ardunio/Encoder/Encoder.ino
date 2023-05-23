int motorIn1 = 4;
int motorIn2 = 7;
int motorEnA = 3;

int encoder = 11;

volatile unsigned int counter;
int rpm;

void setup() {
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorEnA, OUTPUT);
  pinMode(encoder, INPUT);

  digitalWrite(encoder, HIGH);
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  analogWrite(motorEnA, 1000);

  attachInterrupt(0,countpulse,RISING);
  Serial.begin (9600);
  
}

void countpulse(){
        counter++;
}

void loop() {
  static uint32_t previousMillis;
  if (millis() - previousMillis >= 1000) {
            rpm = (counter/20)*60;   
            Serial.println(rpm);       
            counter = 0;
            previousMillis += 1000;
  }
    delay(1);
}