#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  public:
    //Constructor - Plus and Minus are the Motor output / en_a and en_b are the encoder inputs
    Motor(int plus, int minus, int EN, int enc);
    //Spin the motor with a percentage value
    void leftRotate(int value);
    void rightRotate(int value);
    //Motor Outputs - plus is one direction and minus is the other
    int plus;
    int minus;
    int EN;
    //Encoder Inputs
    int enc;
};

#endif
