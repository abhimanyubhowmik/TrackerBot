int trigPin = 12;
int echoPin = 11;
int pingTravelTime;


void setup() {
pinMode(trigPin,OUTPUT);
pinMode(echoPin,INPUT);
Serial.begin(9600);
}

void loop() {
digitalWrite(trigPin,LOW);
delayMicroseconds(10);
digitalWrite(trigPin,HIGH);
delayMicroseconds(10);
digitalWrite(trigPin,LOW);
pingTravelTime = pulseIn(echoPin ,HIGH);
delay(25);
Serial.println(pingTravelTime);
}
