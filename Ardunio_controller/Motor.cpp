#include "Arduino.h"
#include "Motor.h"


Motor::Motor(int plus, int minus, int EN, int enc) {
  pinMode(plus,OUTPUT);
  pinMode(minus,OUTPUT);
  pinMode(EN,OUTPUT);
  pinMode(enc,INPUT_PULLUP);
  Motor::plus = plus;
  Motor::minus = minus;
  Motor::EN = EN;
  Motor::enc = enc;
}

void Motor::leftRotate(int value) {
  if(value>=0){
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
//    Serial.println("called");
//    Serial.println(plus);
    int out = map(value, 0, 100, 0, 475);
    analogWrite(EN,out);
    digitalWrite(plus,HIGH);
    digitalWrite(minus,LOW);
  }else{
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
    int out = map(value, 0, -100, 0, 450);
    analogWrite(EN,out);
    digitalWrite(plus,LOW);
    digitalWrite(minus,HIGH);
  }
}

void Motor::rightRotate(int value) {
  if(value>=0){
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
//    Serial.println("called");
//    Serial.println(plus);
    int out = map(value, 0, 100, 0, 300);
    analogWrite(EN,out);
    digitalWrite(plus,HIGH);
    digitalWrite(minus,LOW);
  }else{
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
    int out = map(value, 0, -100, 0, 300);
    analogWrite(EN,out);
    digitalWrite(plus,LOW);
    digitalWrite(minus,HIGH);
  }
}
