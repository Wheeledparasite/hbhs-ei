/*
 * Author: Lucas Ruebsamen
 * Date: December 28, 2021
 * 
 * Version 1.1 - Updated Jan 14, 2022
 * Version 1.4 - Updated Feb 9, 2022
 * 
 * Changelog:
 * 1.0 - Original Version using Differential Steering
 * 1.1 - Remove reverse ability
 * 1.2 - Interrupt to count wheel rotation sensors (hall effect sensor)
 * 1.3 - Implement PID motor control
 * 1.4 - 500ms Timer2 interrupt to average wheel encoder counts.
 * 
 * Notes: Differential steering control using two BTS7960 motor drivers
 *        driving two 250W Scooter motors.
 * 
 * Differential Steering Library: https://github.com/edumardo/DifferentialSteering
 * LCD Library: https://github.com/mrkaleArduinoLib/LiquidCrystal_I2C
 * PID Lbirary: https://github.com/imax9000/Arduino-PID-Library
 */


/*
 * Timer0 - used for millis() micros() delay()… and is on pin 5, 6
 * Timer1 - 16bit timer is on pin 9, 10
 * Timer2 - 8bit timer is on pin 3, 11
 * 
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//#include <PID_v2.h>
#include "QuickPID.h"
#include <MsTimer2.h>
#include "DifferentialSteering.h"

//#include "BTS7960.h"

#define MSDEBOUNCE 20
#define DEBUG
#define PIDCONT

// PWM PINS on NANO: 3,5,6,9,10,11
// INTERRUPT PINS ON NANO: 2,3

// WHEEL SENSORS
const uint8_t L_SEN = 2;
const uint8_t R_SEN = 3;

// RIGHT MOTOR - PWM using Timer0
// R_DIR = 0 forward, 1 back
// R_PWM = speed (0,255)
const uint8_t R_DIR = 4;
const uint8_t R_PWM = 5;
//const uint8_t RR_PWM = 6;

// LEFT MOTOR - PWM using Timer1
// L_DIR = 0 forward, 1 back
// L_PWM = speed (0,255)
const uint8_t L_DIR = 8;
const uint8_t L_PWM = 9;
//const uint8_t LR_PWM = 10;

// JOYSTICK
const uint8_t VRX_PIN = A0;
const uint8_t VRY_PIN = A1;
const uint8_t SW_PIN = 7;

// differential steering
const int brakeThreshold = 0;
const int fPivYLimit = 32;  // 0 - 127 (A greater value will assign more of the joystick's range to pivoting) - default 32
DifferentialSteering DiffSteer;

//BTS7960 motorLeft(L_DIR, LL_PWM, LR_PWM);
//BTS7960 motorRight(R_DIR, RL_PWM, RR_PWM);

float LeftSetpoint, LeftInput, LeftOutput;
float RightSetpoint, RightInput, RightOutput;
float Kp = 2, Ki = 25, Kd = 0.01;

QuickPID rightPID(&RightInput, &RightOutput, &RightSetpoint);//,PID::P_On::Measurement);
QuickPID leftPID(&LeftInput, &LeftOutput, &LeftSetpoint);//,PID::P_On::Measurement);
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

const unsigned int sampleTime = 256;
unsigned long lastMillis = 0;

volatile uint16_t leftCount  = 0;
volatile uint16_t rightCount = 0;
volatile uint16_t avgLeft = 0;
volatile uint16_t avgRight = 0;

volatile unsigned long previousRightMillis,previousLeftMillis;

void MotorRight(int dir, int pwr);
void MotorLeft(int dir, int pwr);

void count_right_isr() {
  if (millis() - previousRightMillis >= MSDEBOUNCE) {
    //rightCount = (uint16_t) min(255, millis() - previousRightMillis);
    rightCount++;
    previousRightMillis = millis();
  }
}

void count_left_isr() {
  if (millis() - previousLeftMillis >= MSDEBOUNCE) {
    //leftCount = (uint16_t) min(255, millis() - previousLeftMillis);
    leftCount++;
    previousLeftMillis = millis();
  }
}

void timer2_isr() {
  avgLeft = (uint16_t) (avgLeft + leftCount)/2.0;
  avgRight = (uint16_t) (avgRight + rightCount)/2.0;
  leftCount = 0;
  rightCount = 0;
}

void setup()
{
  //motorLeft.Disable();
  //motorRight.Disable();


  // setup pins for speed sensors
  pinMode(R_SEN, INPUT);
  pinMode(L_SEN, INPUT);
  
  // setup pins for analog joystick
  pinMode(VRX_PIN, INPUT);    // analog joystick x-axis
  pinMode(VRY_PIN, INPUT);    // analog joystick y-axis
  pinMode(SW_PIN, INPUT);     // analog joystick push-button

  // setup pins for motor drivers
  pinMode(R_DIR, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  //pinMode(RL_PWM, OUTPUT);
  //pinMode(RR_PWM, OUTPUT);

  //disable motors at startup
  digitalWrite(R_DIR, LOW);
  digitalWrite(L_DIR, LOW);
  analogWrite(R_DIR, 0);
  analogWrite(L_DIR, 0);

  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.clear();
  lcd.print("GoKart v1.4");
  lcd.setCursor(0,1);
  lcd.print("Lucas Ruebsamen");
  delay(2500);
  lcd.clear();
  DiffSteer.begin(fPivYLimit);

  attachInterrupt(digitalPinToInterrupt(R_SEN),count_right_isr,RISING);
  attachInterrupt(digitalPinToInterrupt(L_SEN),count_left_isr,RISING);
  leftPID.SetTunings(Kp,Ki,Kd);
  leftPID.SetMode(leftPID.Control::automatic);
  rightPID.SetTunings(Kp,Ki,Kd);
  rightPID.SetMode(rightPID.Control::automatic);

  MsTimer2::set(500,timer2_isr);
  MsTimer2::start();

#ifdef DEBUG
  Serial.begin(57600);
#endif
}

unsigned short consecutiveError = 0;
int computedLeftPow=0,computedRightPow=0;

// 0 = 0v to 1023 = 5v
// we need 0 - 127 
void loop()
{
  int xValue = map(analogRead(VRX_PIN), 0, 1023, -127, 127);
  int yValue = map(analogRead(VRY_PIN), 0, 1023, 0, 127);
  char sVal[5];

  // differential steering library
  // https://github.com/edumardo/DifferentialSteering
  DiffSteer.computeMotors(xValue,yValue);

#ifdef PIDCONT
  //leftPID.Setpoint(round(map(max(0,DiffSteer.computedLeftMotor()),0,127,255,30)));
  //rightPID.Setpoint(round(map(max(0,DiffSteer.computedRightMotor()),0,127,255,30)));
  LeftSetpoint = round(map(max(0,DiffSteer.computedLeftMotor()),0,127,0,18));
  RightSetpoint = round(map(max(0,DiffSteer.computedRightMotor()),0,127,0,18));
  LeftInput = (float)avgLeft;
  RightInput = (float)avgRight;
  leftPID.Compute();
  rightPID.Compute();
  if (LeftInput <= 0)
    MotorLeft(0,0);
  else
    MotorLeft(0, LeftOutput);
  if (RightInput <= 0)
    MotorRight(0, 0);
  else
    MotorRight(0, RightOutput);
#else
  // Drive LEFT motor
  int motVal = DiffSteer.computedLeftMotor();
  if (motVal > brakeThreshold) {
     
    MotorLeft(0, motVal*2);  // Move Forward
  // } else if (motVal < -brakeThreshold) {
  //  motorLeft.TurnRight(-motVal*2); 
  } else {
    MotorLeft(0,0);
  }
  // Drive RIGHT motor
  motVal = DiffSteer.computedRightMotor();
  if (motVal > brakeThreshold) {
    MotorRight(0, motVal*2); // Move Forward
  // } else if (motVal < -brakeThreshold) {
  //  motorRight.TurnRight(-motVal*2); 
  } else {
    MotorRight(0,0);
  }
#endif

  // HANDLE LCD UPDATES HERE
  if (millis() - lastMillis >= sampleTime) {
#ifdef DEBUG
    Serial.print("LMotor: ");
    Serial.print(DiffSteer.computedLeftMotor());
    Serial.print("\t| RMotor: ");
    Serial.println(DiffSteer.computedRightMotor());
    Serial.print("LSetpt: ");
    Serial.print(LeftSetpoint);
    Serial.print("\t| RSetpt: ");
    Serial.println(RightSetpoint);
    Serial.print("LInput: ");
    Serial.print(LeftInput);
    Serial.print("\t| RInput: ");
    Serial.println(RightInput);
    Serial.print("LOutput: ");
    Serial.print(LeftOutput);
    Serial.print("\t| ROutput: ");
    Serial.println(RightOutput);
    Serial.println();
#endif
    // 500ms has elapsed since last lcd update, time to update again
    lcd.setCursor(0,0); // col 0, row 0
    lcd.print("Vel: ");
    float rpm = ((avgLeft+avgRight)/2.0 * 30);
    float vel = (rpm/(5280/((10.5*PI)/12)))*60;
    sprintf(sVal,"%03d", (int)vel);// (int)rightPID.GetSetpoint());//*30);
    lcd.printstr(sVal);
    lcd.print(" mpH");
/*
    lcd.setCursor(0,1); // col 0, row 1
    lcd.print("X: ");
    //lcd.print(xValue);
    sprintf(sVal,"%+04d", (int)xValue);
    lcd.printstr(sVal);

    lcd.setCursor(8,1);   // col 8, row 1
    lcd.print("Y: ");
    sprintf(sVal,"%+04d", (int)yValue);
    lcd.printstr(sVal);
*/
    lcd.setCursor(0,1); // col 0, row 1
    lcd.print("L: ");
    //lcd.print(xValue);
    sprintf(sVal,"%+04d", (int)avgLeft);
    lcd.printstr(sVal);

    lcd.setCursor(8,1);   // col 8, row 1
    lcd.print("R: ");
    sprintf(sVal,"%+04d", (int)avgRight);
    lcd.printstr(sVal);

    lastMillis = millis();
  }
}

void MotorLeft(int dir, int pwr) {
  digitalWrite(L_DIR, dir);
  analogWrite(L_PWM, pwr);
}

void MotorRight(int dir, int pwr) {
  digitalWrite(R_DIR, dir);
  analogWrite(R_PWM, pwr);
}
