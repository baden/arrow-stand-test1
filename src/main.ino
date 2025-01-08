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
// #include "LowPower.h"

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

ADXL345 adxl;

#define OLED_HARDWARE_I2C 1

#define INTERRUPTPIN 2

// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, clock=4, data=3);
// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 4, /* data=*/ 3);   // ESP32 Thing, HW I2C with pin remapping

#if defined(OLED_HARDWARE_I2C) && (OLED_HARDWARE_I2C == 1)
// Hardware I2C
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#else
// Software I2C
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /*clock*/ 4, /*data*/ 3, /*reset*/U8X8_PIN_NONE);
#endif

// the LED is at pin  GPIO8
#define LED_BUILTIN 8

volatile unsigned trigged = 0;

void wakeUp();
void setup_accelerometer();

// the setup function runs once when you press reset or power the board
void setup() {
    #if defined(OLED_HARDWARE_I2C) && (OLED_HARDWARE_I2C == 1)
        Wire.begin( 3, 4 );
    #endif
    u8g2.begin();

    pinMode(INTERRUPTPIN, INPUT);

    setup_accelerometer();

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
}

void sleep();

// the loop function runs over and over again forever
void loop() {
    static uint32_t counter = 0;

    //Boring accelerometer stuff
    int x, y, z;
    adxl.readXYZ(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z

    double xyz[3];
    double ax, ay, az;
    adxl.getAcceleration(xyz);
    ax = xyz[0];
    ay = xyz[1];
    az = xyz[2];

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);	// choose a suitable font
    u8g2.setCursor(0, 12);
    /*u8g2.print(": ");*/ u8g2.print(ax);
    // u8g2.setCursor(20, 20);
    u8g2.print(":"); u8g2.print(ay);
    // u8g2.setCursor(40, 20);
    u8g2.print(":"); u8g2.print(az);
    u8g2.setCursor(0, 26);
    u8g2.print(trigged);

    if(adxl.triggered(adxl.getInterruptSource(), ADXL345_ACTIVITY)){
        u8g2.print("+");
    } else {
        u8g2.print("-");
    }
    // if(trigged) {
    //     u8g2.print("TRIG");
    //     trigged = 0;
    // } else {
    //     u8g2.print("NTRG");
    // }

    // u8g2.drawStr(0,10,"Hello_World!");	// write something to the internal memory
    // u8g2.drawStr(0,30,"Counter: ");	// write something to the internal memory
    // u8g2.setCursor(80,30);
    // u8g2.print(counter++);	// write something to the internal memory
    u8g2.sendBuffer();					// transfer internal memory to the display


    // if(adxl.triggered(adxl.getInterruptSource(), ADXL345_INACTIVITY)){
    //     sleep();
    // }

    digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (LOW is the voltage level)
    delay(50);                      // wait 50 ms
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off by making the voltage HIGH
    delay(50);                      // wait 950 ms
}

void setup_accelerometer()
{
    adxl.powerOn();
    adxl.setLowPower(true);

    //set activity/ inactivity thresholds (0-255)
    // adxl.setActivityThreshold(75); //62.5mg per increment
    // adxl.setInactivityThreshold(75); //62.5mg per increment
    // Активность будет всё что больше "80"
    // adxl.setActivityThreshold(80);
    // Отсутствие активности - когда меньше "18"
    // adxl.setInactivityThreshold(18);

    adxl.setActivityThreshold(18);
    adxl.setInactivityThreshold(17);
    // Если нет активности - через 3 сек засыпаем
    adxl.setTimeInactivity(3); // how many seconds of no activity is inactive?

    //look of activity movement on this axes - 1 == on; 0 == off
    adxl.setActivityX(1);
    adxl.setActivityY(1);
    adxl.setActivityZ(1);

    //look of inactivity movement on this axes - 1 == on; 0 == off
    adxl.setInactivityX(1);
    adxl.setInactivityY(1);
    adxl.setInactivityZ(1);

    //look of tap movement on this axes - 1 == on; 0 == off
    // adxl.setTapDetectionOnX(0);
    // adxl.setTapDetectionOnY(0);
    // adxl.setTapDetectionOnZ(1);

    //set values for what is a tap, and what is a double tap (0-255)
    // adxl.setTapThreshold(50); //62.5mg per increment
    // adxl.setTapDuration(15); //625us per increment
    // adxl.setDoubleTapLatency(80); //1.25ms per increment
    // adxl.setDoubleTapWindow(200); //1.25ms per increment

    //set values for what is considered freefall (0-255)
    // adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
    // adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment

    //setting all interrupts to take place on int pin 1
    //I had issues with int pin 2, was unable to reset it
    // adxl.setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN);
    // adxl.setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN);
    // adxl.setInterruptMapping(ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN);
    adxl.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN);

#if 0
    // Но сейчас нам это не нужно - ещё не спим
    adxl.setInterrupt(ADXL345_INT_ACTIVITY_BIT, false);

    // При остутствии активности датчик подаст плюс на свой первый вывод
    adxl.setInterruptMapping(ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN);

    // Как раз это сейчас актуально
    adxl.setInterrupt(ADXL345_INT_INACTIVITY_BIT, true);
#endif
    //register interrupt actions - 1 == on; 0 == off
    // adxl.setInterrupt(ADXL345_INT_SINGLE_TAP_BIT, 1);
    // adxl.setInterrupt(ADXL345_INT_DOUBLE_TAP_BIT, 1);
    // adxl.setInterrupt(ADXL345_INT_FREE_FALL_BIT,  1);
    adxl.setInterrupt(ADXL345_INT_ACTIVITY_BIT,   1);
    // adxl.setInterrupt(ADXL345_INT_INACTIVITY_BIT, 1);
}

void wakeUp()
{
    trigged++;
}


void sleep()
{
    // Отсутсвие активности не интересует
    adxl.setInterrupt(ADXL345_INT_INACTIVITY_BIT, false);
    // При активности датчик подаст плюс на свой первый вывод
    adxl.setInterrupt(ADXL345_INT_ACTIVITY_BIT, true);
    // Ставим будильник - когда будет плюс на втором выводе

    // TODO: Спим.

    attachInterrupt(digitalPinToInterrupt(INTERRUPTPIN), wakeUp, RISING);
    // attachInterrupt(0, wakeUp, HIGH);

    // Засыпаем, выключаем что можно
    // LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    // delay(5000);
    while(!adxl.triggered(adxl.getInterruptSource(), ADXL345_ACTIVITY)){
        // sleep();
        delay(100);
    }

    detachInterrupt(digitalPinToInterrupt(INTERRUPTPIN));
    // detachInterrupt(0);

    // Активность не интересует
    adxl.setInterrupt(ADXL345_INT_ACTIVITY_BIT, false);
    // Отсутсвие активности сейчас актуально
    adxl.setInterrupt(ADXL345_INT_INACTIVITY_BIT, true);
}
