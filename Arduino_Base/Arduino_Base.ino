/* 
Author : Krzysztof Dudziak
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define GO_FORWARD 1
#define GO_BACKWARD 2
#define TURN_LEFT 1
#define TURN_RIGHT 2

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motorFR = AFMS.getMotor(1); //create a motor instance
Adafruit_DCMotor *motorFL = AFMS.getMotor(2);
Adafruit_DCMotor *motorBL = AFMS.getMotor(3);
Adafruit_DCMotor *motorBR = AFMS.getMotor(4);

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void moveStraight(int moveDirection, int driveTimems, int maxSpeed)
{
  uint8_t i;
  if (moveDirection == GO_FORWARD)
  {
    motorFR->run(FORWARD); //set direction of motor
    motorFL->run(FORWARD);
    motorBR->run(BACKWARD);
    motorBL->run(BACKWARD);
  }
  else if (moveDirection == GO_BACKWARD)
  {
    motorFR->run(BACKWARD);
    motorFL->run(BACKWARD);
    motorBR->run(FORWARD);
    motorBL->run(FORWARD);
  }
  
  for (i=0; i<maxSpeed; i++) 
  {
    motorFR->setSpeed(i); //motor speed
    motorFL->setSpeed(i);
    motorBR->setSpeed(i);
    motorBL->setSpeed(i);  
    delay(10);
  }
  delay(driveTimems);
  for (i=maxSpeed; i!=0; i--) 
  {
    motorFR->setSpeed(i);  
    motorFL->setSpeed(i);
    motorBR->setSpeed(i);
    motorBL->setSpeed(i); 
    delay(10);
  }
  motorFR->run(RELEASE); //realease motor
  motorFL->run(RELEASE);
  motorBR->run(RELEASE);
  motorBL->run(RELEASE);
}

void turn(int turnDirection, int turnTimems, int turnSpeed) //turning is tankwise
{
  if (turnDirection == TURN_LEFT)
  {
    motorFR->run(FORWARD);
    motorFL->run(BACKWARD);
    motorBR->run(BACKWARD);
    motorBL->run(FORWARD);
  }
  else if (turnDirection == TURN_RIGHT)
  {
    motorFR->run(BACKWARD);
    motorFL->run(FORWARD);
    motorBR->run(FORWARD);
    motorBL->run(BACKWARD);
  }

  motorFR->setSpeed(turnSpeed);  
  motorFL->setSpeed(turnSpeed);
  motorBR->setSpeed(turnSpeed);
  motorBL->setSpeed(turnSpeed);

  delay(turnTimems);

  motorFR->run(RELEASE);
  motorFL->run(RELEASE);
  motorBR->run(RELEASE);
  motorBL->run(RELEASE);
}

void serialEvent() //serial port data receive event function
{
  while (SerialUSB.available()) 
  {
    // get the new byte:
    char inChar = (char)SerialUSB.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    stringComplete = true;
  }
}

void setup() {
  SerialUSB.begin(9600);           // set up SerialUSB library at 9600 bps
  SerialUSB.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
}

void loop() {
  
  if (SerialUSB.available()) serialEvent(); //call event function
  if (stringComplete) {
    SerialUSB.println("received");
   
    if (inputString == "f")
    {
      SerialUSB.println("going forward");
      moveStraight(GO_FORWARD,1000, 128);
    }
    else if (inputString == "b")
    {
      SerialUSB.println("going backward");
      moveStraight(GO_BACKWARD,1000, 128);
    }
    else if (inputString == "l")
    {
      SerialUSB.println("turning left");
      turn(TURN_LEFT, 1000, 64);
    }
    else if (inputString == "r")
    {
      SerialUSB.println("turning right");
      turn(TURN_RIGHT, 1000, 64);
    }
     // clear the string:
    inputString = "";
    stringComplete = false;
  }
}
