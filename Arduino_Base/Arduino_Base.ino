/*
  Author : Krzysztof Dudziak
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
//#include <RedBot.h>
//#include <Encoder.h>
#include <PID_v1.h>


#define GO_FORWARD 1
#define GO_BACKWARD 2
#define TURN_LEFT 3
#define TURN_RIGHT 4
#define SERVO_PIN 9

#define WAIT_TIME 10000 //how much time should wait for Raspberry(for receive command 'w')

#define trigPinForward 2
#define echoPinForward 3
#define trigPinLeft 6
#define echoPinLeft 7
#define trigPinRight 4
#define echoPinRight 5

#define END_OF_MESSAGE 'E'
#define RX_SIZE 10//how large is buffor of receiving messages from USB




//#define IMU_V5

// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the right
// and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the left
// and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

//#include <Wire.h>

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

#define M_X_MIN -1000
#define M_Y_MIN -1000
#define M_Z_MIN -1000
#define M_X_MAX +1000
#define M_Y_MAX +1000
#define M_Z_MAX +1000

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

//CONTROL DEFINITIONS:
#define WHEEL_RADIUS 5.5 //radius in cm
#define ENCODER_POLES 8
#define fixedErrorPID 4.5 //error between given value and recived on which computing stops
#define Kp 10
#define Ki 5
#define Kd 0.003
#define maxPIDsteps 5000 // after cross over of this value robot stops,
#define MAX_VELOCITY 200 // total max is 255

float G_Dt = 0.02;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0; //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; //Omega Integrator
float Omega[3] = {0, 0, 0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] = {
  {
    1, 0, 0
  }
  , {
    0, 1, 0
  }
  , {
    0, 0, 1
  }
};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; //Gyros here


float Temporary_Matrix[3][3] = {
  {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
};

struct dataFromUSB //structure for reading USB command, arguments
{
  String command = "";
  int argument = 0 ;
  bool received = 0;
};
dataFromUSB fromUSB;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motorFR = AFMS.getMotor(1); //create a motor instance
Adafruit_DCMotor *motorFL = AFMS.getMotor(2);
Adafruit_DCMotor *motorBL = AFMS.getMotor(3);
Adafruit_DCMotor *motorBR = AFMS.getMotor(4);

Servo cameraServo; // create servo object to control a servo

int encoderPinL = 10;
int encoderPinR = 11;

volatile int lCount = 0;
volatile int rCount = 0;
double distanceDrivenL = 0;
double distanceDrivenR = 0;

int previousStateEncL = LOW;
int previousStateEncR = LOW;


//RedBotEncoder encoder = RedBotEncoder(A1, A2);  //left encoder, right encoder;
/*Encoder myEnc(10, 11);

  long oldPosition  = -999;
  unsigned long oldTime = 0;

  double countsPerRevolution = 48.0; // encoder counts per revolution of the motor shaft
  double gearRatio = 75.0; // the gear ratio of the motor is 75:1*/

int servoTurn = 5;

void readIMU() {
  if ((millis() - timer) >= 20) // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer = millis();
    if (timer > timer_old)
    {
      G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;

    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter = 0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***
    //for (int i = 0; i < 100; ++i)
    //printdata();
  }
}

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

int detectObstacle(int moveDirection, int distance) //obstacle detection
{
  int counter = 0;
  for (int j = 0; j < 10; ++j)
    if (moveDirection == GO_FORWARD && readSonicForward() < distance)
      ++counter;
    else if (moveDirection == TURN_LEFT && readSonicLeft() < distance)
      ++counter;
    else if (moveDirection == TURN_RIGHT && readSonicRight() < distance)
      ++counter;
  if (counter > 5 )
  {
    SerialUSB.println("obstacle");
    return 1;
  }
  return 0;
}

void tickL() //left encoder
{
  lCount++;
  distanceDrivenL =  2 * 3.14159 * WHEEL_RADIUS * lCount * 1 / ENCODER_POLES;
}

void tickR() // right encoder
{
  rCount++;
  distanceDrivenR =  2 * 3.14159 * WHEEL_RADIUS * rCount * 1 / ENCODER_POLES;

}

/*void getEncoderPosition(){

  //Read encoder counts
  long newPosition = myEnc.read();
  Serial.print("Current encoder count: ");
  Serial.println(newPosition);

  //Get current time
  unsigned long newTime = millis();

  //Get the time elapsed between the previous and current measurement
  long timeElapsed = (newTime-oldTime)/1000;
  Serial.print("Time elapsed (seconds): ");
  Serial.println(timeElapsed);
  oldTime = newTime;

  if (newPosition != oldPosition) { // if the motor moved
    Serial.print("Current output shaft rotations per second: ");
    Serial.println(((newPosition-oldPosition)/(countsPerRevolution*gearRatio))/timeElapsed);
    oldPosition = newPosition;
  }
  }*/



void moveStraight(int moveDirection, double distance)
{
  rCount = 0;
  lCount = 0;
  double velocity = 0;
  int stepsMade = 0;
  PID myPID(&distanceDrivenL, &velocity, &distance, Kp, Ki, Kd, DIRECT);
  myPID.SetMode(AUTOMATIC);

  float goFR = 1, goFL = 1, goBR = 1, goBL = 1; //used to correct path if encoders have different values

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

  while (abs(distance - distanceDrivenL) >= fixedErrorPID && stepsMade < maxPIDsteps)
  {
    myPID.Compute();
    if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY; //velocity limitation
    if (detectObstacle(moveDirection, 30)) break; //evading of obstacles
    if (abs(lCount - rCount) > 1) //prevents of different positions left and right wheel, max error is one position
    {
      if (lCount > rCount) //go more to left side
      {
        goFR = 1; goFL = 0.8; goBR = 1; goBL = 0.8;
      }
      else if (lCount < rCount) //go more to right side
      {
        goFR = 0.8; goFL = 1; goBR = 0.8; goBL = 1;
      }
    }
    else //same power for all motors
    {
      goFR = 1, goFL = 1, goBR = 1, goBL = 1;
    }

    motorFR->setSpeed(goFR * velocity); //motor speed
    motorFL->setSpeed(goFL * velocity);
    motorBR->setSpeed(goBR * velocity);
    motorBL->setSpeed(goBL * velocity);
    stepsMade++;

    //data is send in every loop
    //s(sonar_forward,sonar_right,sonar_left)t(distance_traveled)E
    String stringToSend = "s(" + String(readSonicForward()) +
                          "," + String(readSonicRight()) +
                          "," + String(readSonicLeft()) + ")" +
                          "t(" + String(distanceDrivenL) + ")" + END_OF_MESSAGE;
    SerialUSB.println(stringToSend);
  }
  motorFR->setSpeed(0); //motor speed
  motorFL->setSpeed(0);
  motorBR->setSpeed(0);
  motorBL->setSpeed(0);

  motorFR->run(RELEASE); //realease motor
  motorFL->run(RELEASE);
  motorBR->run(RELEASE);
  motorBL->run(RELEASE);
}

void turn(int turnDirection, int turnTimems, int turnSpeed) //turning is tankwise
{
  if (detectObstacle(turnDirection, 20))
    return;
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

  for ( int i = 0; i < 20; ++i)
  {
    readIMU();
    printdata();
  }

  float wejscier = ToDeg(roll);
  float wejsciep = ToDeg(pitch);
  float wejsciey = ToDeg(yaw);

  for (int i = 0; i < 10; ++i)
  {
    for ( int i = 0; i < 20; ++i)
    {
      readIMU();
    }
    SerialUSB.println(ToDeg(roll));
    SerialUSB.println(ToDeg(pitch));
    SerialUSB.println(ToDeg(yaw));
    SerialUSB.println(" ");
  }
  delay(turnTimems / 10);

  float wyjscier = ToDeg(roll);
  float wyjsciep = ToDeg(pitch);
  float wyjsciey = ToDeg(yaw);

  SerialUSB.println(wyjscier - wejscier);
  SerialUSB.println(wyjsciep - wejsciep);
  SerialUSB.println(wyjsciey - wejsciey);

  motorFR->run(RELEASE);
  motorFL->run(RELEASE);
  motorBR->run(RELEASE);
  motorBL->run(RELEASE);

}

float turn(int turnDirection, float angle) //turning is tankwise
{
  float correction = 1.7;
  angle /= correction;
  if (detectObstacle(turnDirection, 20))
    return -1;
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

  for ( int i = 0; i < 20; ++i)
  {
    readIMU();
    //printdata();
  }

  float wejscier = ToDeg(roll);
  float wejsciep = ToDeg(pitch);
  float wejsciey = ToDeg(yaw);

  motorFR->setSpeed(128);
  motorFL->setSpeed(128);
  motorBR->setSpeed(128);
  motorBL->setSpeed(128);


  while ( abs(wejsciey - ToDeg(yaw)) < abs(angle))
  {
    for ( int i = 0; i < 20; ++i)
    {
      readIMU();
    }

    //s(sonar_forward,sonar_right,sonar_left)t(turn_in_degrees)E
    String stringToSend = "s(" + String(readSonicForward()) +
                          "," + String(readSonicRight()) +
                          "," + String(readSonicLeft()) + ")" +
                          "t(" + String(ToDeg(yaw)) + ")" + END_OF_MESSAGE;
    SerialUSB.println(stringToSend);
  }
  motorFR->setSpeed(0);
  motorFL->setSpeed(0);
  motorBR->setSpeed(0);
  motorBL->setSpeed(0);

  float wyjscier = ToDeg(roll);
  float wyjsciep = ToDeg(pitch);
  float wyjsciey = ToDeg(yaw);

  SerialUSB.print("degrees: ");
  SerialUSB.println(correction * abs(wyjsciey - wejsciey));

  motorFR->run(RELEASE);
  motorFL->run(RELEASE);
  motorBR->run(RELEASE);
  motorBL->run(RELEASE);
  return (abs(wyjsciey - wejsciey));
}

void rotateCamera(int cameraDirection) //rotates camera
{

  /*for (int i = 0; i < 10; ++i)
    {
    cameraServo.write(97);
    //waitForRaspberry();
    delay(500);
    cameraServo.write(95.5);
    for (int j = 0; j <20; ++j)
    {
        readIMU();
        printdata();
    }
    printdata();
    delay(500);
    }
    for (int i = 0; i < 10; ++i)
    {
    cameraServo.write(94);
    //waitForRaspberry();
    delay(500);
    cameraServo.write(95.5);
    for (int j = 0; j <20; ++j)
    {
        readIMU();
        printdata();
    }
    delay(500);
    }*/
  //cameraStep = 1.5;
  //SerialUSB.println(cameraDirection);
  cameraServo.write(95 + cameraDirection * 1.5);
  //waitForRaspberry();
  delay(500 + (-1 + cameraDirection) * 50);
  cameraServo.write(95);
  for (int j = 0; j < 20; ++j)
  {
    readIMU();
    printdata();
  }
  delay(500);
}

void setup() {
  SerialUSB.begin(9600);           // set up SerialUSB library at 9600 bps

  SerialUSB.println("start");

  AFMS.begin();  // create with the default frequency 1.6KHz
  cameraServo.attach(SERVO_PIN);
  cameraServo.write(95);

  pinMode(trigPinForward, OUTPUT);    // settings for sonic sensors
  pinMode(echoPinForward, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);

  pinMode(encoderPinL, INPUT);
  pinMode(encoderPinR, INPUT);
  attachInterrupt(encoderPinL, tickL, CHANGE);
  attachInterrupt(encoderPinR, tickR, CHANGE);

  //Serial.begin(115200);
  pinMode (STATUS_LED, OUTPUT); // Status LED

  SerialUSB.println("test1");
  I2C_Init();
  SerialUSB.println("test2");

  digitalWrite(STATUS_LED, LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();
  SerialUSB.println("test3");
  delay(20);

  for (int i = 0; i < 32; i++) // We take some readings...
  {
    Read_Gyro();
    Read_Accel();
    SerialUSB.println("test4");
    for (int y = 0; y < 6; y++) // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
  }

  for (int y = 0; y < 6; y++)
    AN_OFFSET[y] = AN_OFFSET[y] / 32;

  AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for (int y = 0; y < 6; y++)
    SerialUSB.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED, HIGH);

  timer = millis();
  delay(20);
  counter = 0;
  //your own initializattion code
}

void loop() {
  for ( int i = 0; i < 20; ++i)
  {
    readIMU();
  }
  //readIMU();
  if (SerialUSB.available()) recieveUSB(); //call event function
  if (fromUSB.received) {
    String argument = "0";//distance or angle or values from sensor
    if (fromUSB.command == "f")           //forward
    {
      moveStraight(GO_FORWARD, fromUSB.argument);
      argument = String(distanceDrivenL);
      distanceDrivenL = 0;
    }
    else if (fromUSB.command  == "b")      //backward
    {
      moveStraight(GO_BACKWARD, fromUSB.argument);
      argument = String(distanceDrivenL);
      distanceDrivenL = 0;
    }
    else if (fromUSB.command == "l")       //left
    {
      argument = String(turn(TURN_LEFT, fromUSB.argument));//turn left, get value and assign
    }
    else if (fromUSB.command  == "r")      //right
    {
      argument = String(turn(TURN_RIGHT, fromUSB.argument));//turn right, get value and assign
    }
    else if (fromUSB.command  == "p")     //camera rotation right
    {
      //rotateCamera(1);
      turn(TURN_RIGHT, 20.0);
    }
    else if (fromUSB.command  == "q")     //camera rotation left
    {
      //rotateCamera(-1);
      turn(TURN_LEFT, 20.0);
    }
    else if (fromUSB.command == "i")      //IMU data sending
    {
      for ( int i = 0; i < 20; ++i)//reads 20 times for true value
      {
        readIMU();
      }
      //imu degrees data
      //i(yaw_degrees)E
      argument  = "(" + String(ToDeg(yaw)) + ")";
    }
    else if (fromUSB.command == "e")    //encoders data sending
    {
      //encoder data
      //e(encoder_right,encoder_left)E
      argument  = "(" + String(rCount) +
                  "," + String(lCount) + ")";
    }
    else if (fromUSB.command == "s")    //sonars data sending
    {
      //sonar data
      //s(sonar_forward,sonar_right,sonar_left)E
      argument = "(" + String(readSonicForward()) +
                 "," + String(readSonicRight()) +
                 "," + String(readSonicLeft()) + ")";
    }
    sendConfirmation(fromUSB.command, argument);
    clearDataFromUSB();
  }
}


