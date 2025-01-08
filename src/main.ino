/*
  Blink

  Flashes a LED every second, repeatedly.

  The ESP32-C3 SuperMini has an on-board LED you can control. 
  It is attached to digital pin 8. 
  LED_BUILTIN is set to the correct LED pin.

  This example code is in the public domain.

  Adapted from:
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

#include <ADXL345.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

ADXL345 adxl;

// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, clock=4, data=3);
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 4, /* data=*/ 3);   // ESP32 Thing, HW I2C with pin remapping

// the LED is at pin  GPIO8
#define LED_BUILTIN 8

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  u8g2.begin();
}

// the loop function runs over and over again forever
void loop() {
    static uint32_t counter = 0;


    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);	// choose a suitable font
    u8g2.drawStr(0,10,"Hello World!");	// write something to the internal memory
    u8g2.drawStr(0,30,"Counter: ");	// write something to the internal memory
    u8g2.setCursor(80,30);
    u8g2.print(counter++);	// write something to the internal memory
    u8g2.sendBuffer();					// transfer internal memory to the display

    digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (LOW is the voltage level)
    delay(50);                      // wait 50 ms
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off by making the voltage HIGH
    delay(1950);                      // wait 950 ms
}
