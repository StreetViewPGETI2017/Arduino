/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define GO_FORWARD 1
#define GO_BACKWARD 2
#define TURN_LEFT 1
#define TURN_RIGHT 2

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motorFR = AFMS.getMotor(1);
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
    motorFR->run(FORWARD);
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
    motorFR->setSpeed(i);
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
  motorFR->run(RELEASE);
  motorFL->run(RELEASE);
  motorBR->run(RELEASE);
  motorBL->run(RELEASE);
}

void turn(int turnDirection, int turnTimems, int turnSpeed)
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

void serialEvent() {
  while (SerialUSB.available()) {
    // get the new byte:
    char inChar = (char)SerialUSB.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    stringComplete = true;
    /*if (inChar == '\n') 
    {
      stringComplete = true;
    }*/
  }
}

void setup() {
  SerialUSB.begin(9600);           // set up SerialUSB library at 9600 bps
  SerialUSB.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  /*myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);*/
}

void loop() {
  
  if (SerialUSB.available()) serialEvent();
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
