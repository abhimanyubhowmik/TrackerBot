#define ENA 3
#define in1 4
#define in2 7

#define ENB 5
#define in3 9
#define in4 8

#define sensorLeft 10
#define sensorRight 11

unsigned long start_time = 0;
unsigned long end_time = 0;
int stepsLeft=0;
int stepsRight = 0;
float steps_old_left=0;
float steps_old_right=0;
float temp1=0;
float temp2=0;
float rps_left=0;
float rps_right=0;

void setup() {
  pinMode(ENA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);

  pinMode(ENB,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);

  Serial.begin (9600);
  pinMode(sensorLeft,INPUT_PULLUP);
  pinMode(sensorRight,INPUT_PULLUP);


}



void loop() {

  start_time=millis();
  end_time=start_time+1000;

  while(millis()<end_time) {
   if(digitalRead(sensorLeft)) {
    stepsLeft=stepsLeft+1; 
    while(digitalRead(sensorLeft));
   }
   if(digitalRead(sensorRight)) {
      stepsRight=stepsRight+1; 
      while(digitalRead(sensorRight));
   }
 }


    temp1=stepsLeft-steps_old_left;
    steps_old_left=stepsLeft;
    rps_left=(temp1/20);

    temp2=stepsRight-steps_old_right;
    steps_old_right=stepsRight;
    rps_right=(temp2/20);
    Serial.print("Left:");Serial.print(rps_left);Serial.print(",");
    Serial.print("Right:");Serial.println(rps_right);

  // put your main code here, to run repeatedly:
  analogWrite(ENA,1000);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);

  analogWrite(ENB,1000);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  // delay(10000);

  // analogWrite(ENA,500);
  // digitalWrite(in1,HIGH);
  // digitalWrite(in2,LOW);

  // analogWrite(ENB,500);
  // digitalWrite(in3,HIGH);
  // digitalWrite(in4,LOW);
  // delay(10000);

  }

