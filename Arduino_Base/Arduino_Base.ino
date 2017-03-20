/* 
Author : Krzysztof Dudziak
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

#define GO_FORWARD 1
#define GO_BACKWARD 2
#define TURN_LEFT 1
#define TURN_RIGHT 2
#define SERVO_PIN 9

#define I2C_ADDRESS 5
#define I2C_SEND_BYTES 10

#define SERVO_START_POS 50
#define SERVO_END_POS 165

#define TEST_SERIAL 1

#define trigPinForward 2
#define echoPinForward 3
#define trigPinLeft 4
#define echoPinLeft 5
#define trigPinRight 6
#define echoPinRight 7

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motorFR = AFMS.getMotor(1); //create a motor instance
Adafruit_DCMotor *motorFL = AFMS.getMotor(2);
Adafruit_DCMotor *motorBL = AFMS.getMotor(3);
Adafruit_DCMotor *motorBR = AFMS.getMotor(4);

Servo cameraServo; // create servo object to control a servo

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int servoTurn = 5;
volatile char charI2C;
volatile boolean charI2Ccomplete = false;

void waitForRaspberry()
{
  delay(500);
  /*
  while(1) //waits for Raspberry confirmation
  {
    if(charI2Ccomplete &&  charI2C == 'w')break;
    else delay(10);
  }
  charI2C = ' ';
  charI2Ccomplete = false;*/
}
int sendToRaspberry()
{
  //code for sending to Raspberry
  return 1;
}

String getFromRaspberry()
{
  //code for receiving from Raspberry
  if(charI2Ccomplete)
  {
    charI2Ccomplete = false;
    return String(charI2C);
  }
  return "";
}

long* readSonicData(){
 //long forwardDistance, leftDistance, rightDistance;

 long distance[3];
 distance[0] = readSonicForward();
 distance[1] = readSonicLeft();
 distance[2] = readSonicRight();
  
 return distance;
}

long readSonicForward(){
  long time, distance;
 
  digitalWrite(trigPinForward, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinForward, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinForward, LOW);
 
  time = pulseIn(echoPinForward, HIGH);
  distance = time / 58;  // to get cm 

  return distance;
}

long readSonicRight(){
  long time, distance;
 
  digitalWrite(trigPinRight, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight, LOW);
 
  time = pulseIn(echoPinRight, HIGH);
  distance = time / 58;  // to get cm 

  return distance;
}

long readSonicLeft(){
  long time, distance;
 
  digitalWrite(trigPinLeft, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);
 
  time = pulseIn(echoPinLeft, HIGH);
  distance = time / 58;  // to get cm 

  return distance;
}

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

void rotateCamera() //rotates camera
{
  for (int i = 0; i < 8; ++i)
  {
    cameraServo.write(100);
    waitForRaspberry();
    cameraServo.write(95);
    waitForRaspberry();
  }
  for (int i = 0; i < 8; ++i)
  {
    cameraServo.write(90);
    waitForRaspberry();
    cameraServo.write(95);
    waitForRaspberry();
  }
}

void serialEvent() //serial port data receive event function
{
  while (SerialUSB.available()) 
  {
    // get the new byte:
    char inChar = (char)SerialUSB.read();
    // add it to the inputString:
    inputString += inChar;
    stringComplete = true; //one char working version
  }
}
void receiveEventI2C(int howMany)//I2C port data receive, howMany - the number of bytes read from the master
{
  while ( Wire.available()) // loop through all but the last
  { 
    charI2C = Wire.read(); // receive byte as a character
  }
  charI2Ccomplete = true; 
}
void requestEventI2C() {
  Wire.write("hello ");
}

void setup() {
  SerialUSB.begin(9600);           // set up SerialUSB library at 9600 bps
  SerialUSB.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  cameraServo.attach(SERVO_PIN);
  cameraServo.write(SERVO_START_POS);

 /* //I2C:
  Serial.begin(9600);           // start serial at 9600 bps
  Wire.begin(I2C_ADDRESS); //join to i2c with I2C_ADDRESS
  Wire.onReceive(receiveEventI2C); // register event
  //Wire.onRequest(requestEventI2C); */

  pinMode(trigPinForward, OUTPUT);    // settings for sonic sensors
  pinMode(echoPinForward, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  //your own initializattion code
}

void loop() {
#ifdef TEST_SERIAL
  if (SerialUSB.available()) serialEvent(); //call event function
#endif
#ifndef TEST_SERIAL
  inputString = getFromRaspberry();
#endif
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
      turn(TURN_LEFT, 1000, 128);
    }
    else if (inputString == "r")
    {
      SerialUSB.println("turning right");
      turn(TURN_RIGHT, 1000, 128);
    }
    else if (inputString == "p")
    {
      //SerialUSB.println(readSonicLeft());
      SerialUSB.println("camera rotation");
      rotateCamera();
    }
    else if (inputString == "s")
    {
      SerialUSB.println("Ultra Sonic Sensor read: ");
      SerialUSB.println(readSonicLeft());
      SerialUSB.println(readSonicForward());
      SerialUSB.println(readSonicRight());
    }
     // clear the string:
    inputString = "";
    stringComplete = false;
  }
}
