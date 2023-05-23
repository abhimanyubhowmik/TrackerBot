#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "NewPing.h"

#define trigPinFront 6
#define echoPinFront 5
#define trigPinBack 11
#define echoPinBack 10

#define maxDistance 400

NewPing sonarFront(trigPinFront, echoPinFront, maxDistance);
NewPing sonarBack(trigPinBack, echoPinBack, maxDistance);

#define screenWidth 128
#define screenHight 32

Adafruit_SSD1306 display(screenWidth,screenHight);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

}

void loop() {
  // put your main code here, to run repeatedly:
  long distanceFront, distanceBack;
  distanceFront = sonarFront.ping_cm();
  distanceBack = sonarBack.ping_cm();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("Front: ");
  display.print(distanceFront);
  display.setCursor(0,8);
  display.print("Back: ");
  display.print(distanceBack);
  display.setCursor(0,24);
  display.print("IP: 192.168.210.20");
  display.display();


}
