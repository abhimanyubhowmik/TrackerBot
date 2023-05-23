#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define screenWidth 128
#define screenHight 32

Adafruit_SSD1306 display(screenWidth,screenHight);



void setup() {
  // put your setup code here, to run once:
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

}

void loop() {
  // put your main code here, to run repeatedly:
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(SSD1306_WHITE);
display.setCursor(0,0);
display.print("Hello World");
display.display();
}
