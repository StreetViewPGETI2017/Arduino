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
#define WAIT_TIME 10000

#define SERVO_START_POS 50
#define SERVO_END_POS 165

#define trigPinForward 2
#define echoPinForward 3
#define trigPinLeft 4
#define echoPinLeft 5
#define trigPinRight 6
#define echoPinRight 7

#define END_OF_MESSAGE "$"
#define RX_SIZE 3
#define TX_SIZE 20

String dataFromUSB[RX_SIZE] = {"", "", ""};; //0 - code, 1-argument, 2-argument
volatile bool received = false;
struct dataToUSB {
  //counts how many rows is filled:
  int rowsIMU = 0;
  int rowsEncoder = 0;
  int rowsSonar = 0;
  //data storage:
  int imu[TX_SIZE][3];//accelerometer,magnetometer,gyroscope
  int sonar[TX_SIZE][3];//forward,left,right
  int encoder[TX_SIZE][3];//only two encoders, but function accepts three
};
dataToUSB data;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motorFR = AFMS.getMotor(1); //create a motor instance
Adafruit_DCMotor *motorFL = AFMS.getMotor(2);
Adafruit_DCMotor *motorBL = AFMS.getMotor(3);
Adafruit_DCMotor *motorBR = AFMS.getMotor(4);

Servo cameraServo; // create servo object to control a servo

int servoTurn = 5;

long* readSonicData() {
  //long forwardDistance, leftDistance, rightDistance;

  long distance[3];
  distance[0] = readSonicForward();
  distance[1] = readSonicLeft();
  distance[2] = readSonicRight();

  return distance;
}

long readSonicForward() {
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

long readSonicRight() {
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

long readSonicLeft() {
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

  for (i = 0; i < maxSpeed; i++)
  {
    motorFR->setSpeed(i); //motor speed
    motorFL->setSpeed(i);
    motorBR->setSpeed(i);
    motorBL->setSpeed(i);
    delay(10);
  }
  delay(driveTimems);
  for (i = maxSpeed; i != 0; i--)
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
  int pos;

  for (pos = cameraServo.read(); pos + servoTurn <= SERVO_END_POS; pos = cameraServo.read()) //take pictures
  {
    cameraServo.write(pos + servoTurn);
    SerialUSB.println(cameraServo.read());
    waitForRaspberry(WAIT_TIME);
  }

  servoTurn = -servoTurn;

  for (pos = cameraServo.read(); pos + servoTurn >= SERVO_START_POS; pos = cameraServo.read()) //return back
  {
    cameraServo.write(pos + servoTurn);
    SerialUSB.println(cameraServo.read());
    delay(500); //perhaps should be replaced with waiting for Raspberry
  }

  servoTurn = -servoTurn;
  /*for (int i = 0; i < 10; ++i)
    {
    cameraServo.write(0);
    delay(100);
    cameraServo.write(78);
    delay(500);
    }*/

}

void setup() {
  SerialUSB.begin(9600);           // set up SerialUSB library at 9600 bps
  SerialUSB.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  cameraServo.attach(SERVO_PIN);
  cameraServo.write(SERVO_START_POS);

  pinMode(trigPinForward, OUTPUT);    // settings for sonic sensors
  pinMode(echoPinForward, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  //your own initializattion code
}

void loop() {
  if (SerialUSB.available()) recieveUSB(); //call event function
  if (received) {
    String command = dataFromUSB[0];
    int argument = 0;//distance or angle
    int confirmationArgument = 0;//distance or angle, changed on true value of move or rotation

    if (command == "f")           //forward
    {
      argument = dataFromUSB[1].toInt();//distance
      moveStraight(GO_FORWARD, 1000, 128);
      confirmationArgument = 1; //test value
    }
    else if (command  == "b")      //backward
    {
      argument = dataFromUSB[1].toInt();//distance
      moveStraight(GO_BACKWARD, 1000, 128);
      confirmationArgument = 1; //test value
    }
    else if (command == "l")       //left
    {
      argument = dataFromUSB[1].toInt();//angle
      turn(TURN_LEFT, 1000, 64);
      confirmationArgument = 2; //test value
    }
    else if (command  == "r")      //right
    {
      argument = dataFromUSB[1].toInt();//angle
      turn(TURN_RIGHT, 1000, 64);
      confirmationArgument = 2; //test value
    }
    else if (command  == "p")     //camera rotation
    {
      rotateCamera();
    }
    else if (command == "i")      //IMU data sending
    {
      sendUSB(data.imu, 3, data.rowsIMU);
      memset(data.imu, 0, sizeof(data.imu)); //clear data
      data.rowsIMU = 0;
    }
    else if (command == "e")    //encoders data sending
    {
      sendUSB(data.encoder, 2, data.rowsEncoder);
      memset(data.encoder, 0, sizeof(data.encoder)); //clear data
      data.rowsEncoder = 0;
    }
    else if (command == "s")    //sonars data sending
    {
      sendUSB(data.sonar, 3, data.rowsSonar);
      memset(data.sonar, 0, sizeof(data.sonar)); //clear data
      data.rowsSonar = 0;
    }
    sendConfirmation(command, confirmationArgument);
    clearDataFromUSB();
  }
}
