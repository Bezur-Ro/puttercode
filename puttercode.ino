#include <avr/wdt.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <IRremote.h>
#include "Servo.h"
#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"
#include <elapsedMillis.h>
#include "FastLED.h"
#include "SR04.h"

#define VOL_MEASURE_PIN A3

#define TIMEOUT 200

#define PIN_RBGLED 4
#define NUM_LEDS 1

#define TRIG_PIN 13
#define ECHO_PIN 12

#define up 70     //   up
#define down 21   //  down
#define LEFT 68   // left
#define RIGHT 67  // right
#define ok 64     // ok grabber
#define code1 22  // 1
#define code2 25  // 2
#define code3 13  // 3
#define code4 12  // 4
#define code5 24  // 5
#define code6 94  // 6
#define code7 8   // 7
#define code8 28  // 8
#define code9 90  // 9
#define code0 80  // 0
#define pnd 74    // #
#define astr 66   // *


// define IO pin
#define PWMA 5  // Controls power to right motor
#define PWMB 6  // Controls power to left motor
#define AIN 7   // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define BIN 8   // Controls direction of LEFT motor, HIGH = FORWARD, LOW = REVERSE
#define STBY 3  // Place H-Bridge in standby if LOW, Run if HIGH

#define modeSwitch 2  // Mode Switch input


uint32_t
Color(uint8_t r, uint8_t g, uint8_t b) {
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}






uint32_t period = 10000L;

unsigned long timestamp = 0;
unsigned int interval = 60000;  // one min
unsigned long vol_measure_time = 0;

static bool isStarted = false;
static uint32_t startTime;

long dist;
long distance;

float yaw;   //angle robot is facing
float temp;  // mpu6050 temp
float lpos;  // variable for saving robots last position
float deltaA = 0;
float lower;  // upper range for turning function
float upper;  // lower range for turning function

int tol = 2.5;  // angular tolerance for turning function
int POW = 67;   // motor power
int tpow = 67;  //motor power when turning
int turning = 1;
int speed = 1;
int targetpos = 15;  // wanted angle
int RECV_PIN = 9;    //

MPU6050 mpu6050(Wire);

CRGB leds[NUM_LEDS];

elapsedMillis timeElapsed;

IRrecv irrecv(RECV_PIN);

decode_results results;

Servo grabber;

Servo head;

SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);

void setup() {
  Application_FunctionSet.ApplicationFunctionSet_Init();
  pinMode(PWMA, OUTPUT);  //set IO pin mode OUTPUT
  pinMode(PWMB, OUTPUT);
  pinMode(BIN, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  digitalWrite(STBY, HIGH);  //Enable Motors to run   // Fully on
  digitalWrite(PWMA, LOW);   // digitalWrite(PWMA, HIGH);  // Fully on
  Serial.begin(9600);
  Wire.begin();
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(127);
  FastLED.showColor(Color(0, 0, 0));
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  irrecv.enableIRIn();
  voltageInit();
  grabber.attach(11);  // attach the grabber servo
  head.attach(10);
  grabber.write(90);  // stop the grabber servo
  head.write(100);
}


//main loop
void loop() {
  //output();
  FastLED.showColor(Color(0, 0, 127));
  delay(200);
  while (1 == 1) {
    //Voltage_Measure();
    if (IrReceiver.decode()) {
      uint16_t command = IrReceiver.decodedIRData.command;
      switch (command) {
          //case up:
          //code
          ///  fwrd();
          //  break;
          //case down:
          //code
          // rev();
          // break;
        case LEFT:
          //code
          left(targetpos);
          break;
        case RIGHT:
          //code
          right(targetpos);
          break;
        case ok:
          //code
          FastLED.showColor(Color(255, 0, 0));

          if (turning == 1) {
            FastLED.setBrightness(64);
            targetpos += 30;
            turning += 1;
          } else if (turning == 2) {
            FastLED.setBrightness(127);
            targetpos += 30;
            turning += 1;
          } else if (turning == 3) {
            FastLED.setBrightness(193);
            targetpos += 30;
            turning += 1;
          } else if (turning == 4) {
            FastLED.setBrightness(255);
            targetpos += 30;
            turning += 1;
          } else {
            FastLED.setBrightness(64);
            turning = 1;
            targetpos = 15;
          }
          delay(200);
          FastLED.showColor(Color(0, 0, 127));
          FastLED.setBrightness(127);
          break;
        case code1:
          // code
          hole1();
          break;
        case code2:
          //code
          hole2();
          break;
        case code3:
          //code
          hole3();
          break;
        case code4:
          // code
          hole4();
          break;
        case code5:
          //code
          hole5();
          break;
        case code6:
          //code
          break;
        case code7:
          // code
          distance = sr04.Distance();  //get distance in cm
          while (distance >= 20) {     // drive forward until an object is 20cm infront of robot
            fwrd();
          }
          stp();
          break;
        case code8:
          //code
          FastLED.showColor(Color(255, 0, 255));

          if (speed == 1) {
            FastLED.setBrightness(64);
            POW = 64;
            speed += 1;
          } else if (speed == 2) {
            FastLED.setBrightness(127);
            POW = 127;
            speed += 1;
          } else if (speed == 3) {
            FastLED.setBrightness(193);
            POW = 191;
            speed += 1;
          } else if (speed == 4) {
            FastLED.setBrightness(255);
            POW = 255;
            speed += 1;
          } else {
            FastLED.setBrightness(64);
            speed = 1;
            POW = 64;
          }

          delay(200);
          FastLED.showColor(Color(0, 0, 127));
          FastLED.setBrightness(127);
          break;
        case code9:
          //code
          interval = 10000;                                                  // 10 seconds
          for (uint32_t tStart = millis(); (millis() - tStart) < period;) {  // track line for "interval" amount of time
            delay(10);
            Application_FunctionSet.ApplicationFunctionSet_Tracking();
            Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
          }


          break;
        case code0:
          // code
          break;
        case pnd:
          //code
          gripperclose();
          break;
        case astr:
          //code
          gripperopen();
          break;
        default:
          //default code if button isnt above
          break;
      }

      if (command == up) {
        fwrd();
        timestamp = millis();
        command = 0;
      }

      if (command == up) {
        timestamp = millis();
        command = 0;
      }
      if (command == down) {
        rev();
        timestamp = millis();
        command = 0;
      }

      if (command == down) {
        timestamp = millis();
        command = 0;
      }
      delay(100);
      irrecv.resume();  // Receive the next value
    }
    if (millis() - timestamp > TIMEOUT) {
      stp();
    }
  }
}


void gripperopen() {
  grabber.write(120);  // rotate the motor counter-clockwise
  delay(750);
  grabber.write(94);  // stop the motor
}

void gripperclose() {
  grabber.write(45);  // rotate the motor clockwise
  delay(850);
  grabber.write(94);  // stop the motor
}

void voltageInit() {
  pinMode(VOL_MEASURE_PIN, INPUT);
}

void Voltage_Measure() {
  if (millis() - vol_measure_time > 1000)  //Measured every 1000 milliseconds
  {
    vol_measure_time = millis();
    float voltage = (analogRead(VOL_MEASURE_PIN) * 5) * ((10 + 1.5) / 1.5) / 1024;  //Read voltage value
                                                                                    //float voltage = (analogRead(VOL_MEASURE_PIN) * 0.0375);
    voltage = voltage + (voltage * 0.08);
    Serial.print("Current voltage value : ");
    Serial.println(voltage);
    if (voltage > 7.8) {
      Serial.println("The battery is fully charged");
      FastLED.showColor(Color(0, 255, 0));
    } else {
      Serial.println("Low battery");
      FastLED.showColor(Color(255, 0, 0));
      delay(100);
      FastLED.showColor(Color(0, 0, 0));
    }
  }
}





void output() {
  mpu6050.update();
  temp = (mpu6050.getTemp() * (9 / 5)) + 32;  // get temp in F
  yaw = mpu6050.getAngleZ();
  // Serial.print("\tangleZ : ");
  //Serial.println(mpu6050.getAngleZ());
  Serial.print("\tyaw : ");
  Serial.println(yaw);
  Serial.print("\deltaA : ");
  Serial.println(deltaA);
}

void left(int tpos) {
  //output();
  deltaA = (yaw + tpos);
  while (deltaA > yaw) {
    lft();
    output();
  }
  stp();
}

void right(int tpos) {
  output();
  deltaA = (yaw - tpos);
  while (deltaA < yaw) {
    rht();
    output();
  }
  stp();
}

void rht() {
  digitalWrite(AIN, LOW);   //Right wheels Reverse direction
  digitalWrite(BIN, HIGH);  // Left wheels Forward direction
  analogWrite(PWMA, tpow);  // Right wheels power
  analogWrite(PWMB, tpow);  // Left wheels power
}

void lft() {
  digitalWrite(AIN, HIGH);  //Right wheels Forward direction
  digitalWrite(BIN, LOW);   // Left wheels Reverse direction
  analogWrite(PWMA, tpow);  // Right wheels power
  analogWrite(PWMB, tpow);  // Left wheels power
}

void fwrd() {
  digitalWrite(AIN, HIGH);  //Right wheels Forward direction
  digitalWrite(BIN, HIGH);  // Left wheels Forward direction
  analogWrite(PWMA, POW);   // Right wheels power
  analogWrite(PWMB, POW);   // Left wheels power
}

void rev() {
  digitalWrite(AIN, LOW);  //Right wheels Forward direction
  digitalWrite(BIN, LOW);  // Left wheels Forward direction
  analogWrite(PWMA, POW);  // Right wheels power
  analogWrite(PWMB, POW);  // Left wheels power
}
void stp() {
  digitalWrite(PWMA, LOW);  // No power on Right
  digitalWrite(PWMB, LOW);  // No power on Left
}



void myColor() {
  int r = 255, g = 0, b = 0;
  for (int i = 0; i < 255; i++) {
    FastLED.showColor(Color(r, g, b));
    r -= 1;
    g += 1;
    delay(10);
  }
  r = 0, g = 0, b = 255;

  for (int i = 0; i < 255; i++) {
    FastLED.showColor(Color(r, g, b));
    r += 1;
    b -= 1;
    delay(10);
  }
  r = 0, g = 255, b = 0;

  for (int i = 0; i < 255; i++) {
    FastLED.showColor(Color(r, g, b));
    g -= 1;
    b += 1;
    delay(10);
  }
  r = 0, g = 0, b = 0;
}

// Function - accepts the time in milli-Seconds to go into standby for
void stopTime(int mS) {
  digitalWrite(STBY, LOW);   // Go into standby
  delay(mS);                 //  Wait defined time
  digitalWrite(STBY, HIGH);  //  Come out of standby
}
