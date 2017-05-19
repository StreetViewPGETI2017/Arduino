#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
//#include <RedBot.h>
//#include <Encoder.h>
#include <Stepper.h>
#include <RunningMedian.h>


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

#define steps 32   
#define moveSteps 32 * 64  //2048  

// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
//#define IMU_V5

// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the right
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the left
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>

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

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

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
#define OUTPUTMODE 0

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

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

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

//CONTROL/MOVE DEFINITIONS:
#define ENCODER_POLES 8
#define MAX_VELOCITY 200 // total max is 255
#define distanceForTick 4.69 //wheel round / encoder poles
#define SEND_INFO_LOOP false

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

Stepper stepper(steps, 8, 10, 9, 11); // PINS TO CHANGE!
int waitingSteps;

int encoderPinL = 0; //10
int encoderPinR = 1; //11

volatile int lCount = 0;
volatile int rCount = 0;
volatile double distanceDrivenL = 0;
volatile double distanceDrivenR = 0;

int previousStateEncL = LOW;
int previousStateEncR = LOW;


//RedBotEncoder encoder = RedBotEncoder(A1, A2);  //left encoder, right encoder;
/*Encoder myEnc(10, 11);

  long oldPosition  = -999;
  unsigned long oldTime = 0;

  double countsPerRevolution = 48.0; // encoder counts per revolution of the motor shaft
  double gearRatio = 75.0; // the gear ratio of the motor is 75:1*/

int servoTurn = 5;

long* readSonicData() {
  //long forwardDistance, leftDistance, rightDistance;

  long distance[3];
  distance[0] = readSonicForward();
  distance[1] = readSonicLeft();
  distance[2] = readSonicRight();

  return distance;
}

void tickL() //left encoder
{
  lCount++;
  distanceDrivenL = lCount * distanceForTick;
}

void tickR() // right encoder
{
  rCount++;
//  distanceDrivenR =  rCount * distanceForTick;
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
  distanceDrivenL = 0;
  distanceDrivenR = 0;
  double velocity = 0;
  bool defaultSpeed = true;
  bool changeSpeed = true;

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
  velocity = MAX_VELOCITY;
  while (distance - distanceDrivenL >= distanceForTick)
  {
    if (detectObstacle(moveDirection, 30)) break; //evading of obstacles
    if (changeSpeed) //prevents from setting speed in every loop
    {
      motorFR->setSpeed(goFR * velocity); //motor speed
      motorFL->setSpeed(goFL * velocity);
      motorBR->setSpeed(goBR * velocity);
      motorBL->setSpeed(goBL * velocity);
      changeSpeed = false;
    }

    if (abs(lCount - rCount) > 1) //prevents of different positions left and right wheel, max error is one position
    {

      changeSpeed = true;
      defaultSpeed = false;

      if (lCount > rCount) //go more to right side
      {
        goFR = 1; goFL = 0.8; goBR = 1; goBL = 0.8;
      }
      else if (lCount < rCount) //go more to left side
      {
        goFR = 0.8; goFL = 1; goBR = 0.8; goBL = 1;
      }
    }
    else if (!defaultSpeed) //same power for all motors if isn't set
    {
      defaultSpeed = true;
      changeSpeed = true;
      goFR = 1, goFL = 1, goBR = 1, goBL = 1;
    }

    if (SEND_INFO_LOOP && velocity != 0)
    {
      //data is send in every loop
      //s(sonar_forward,sonar_right,sonar_left)t(distance_traveled)E
      String stringToSend = "s(" + String(readSonicForward()) +
                            "," + String(readSonicRight()) +
                            "," + String(readSonicLeft()) + ")" +
                            "t(" + String(distanceDrivenL) + ")" + END_OF_MESSAGE;
      SerialUSB.println(stringToSend); 
    }
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

float turn(int turnDirection, int turnTimems, int turnSpeed) //turning is tankwise
{
  /*rCount = 0;
    lCount = 0;*/
  if (detectObstacle(turnDirection, 20))
    return 0.0;
  if (turnDirection == TURN_LEFT)
  {
    turnTimems *= 1.25;
    motorFR->run(FORWARD);
    motorFL->run(BACKWARD);
    motorBR->run(BACKWARD);
    motorBL->run(FORWARD);

  }
  else if (turnDirection == TURN_RIGHT)
  {
    turnTimems *= 1.25;
    motorFR->run(BACKWARD);
    motorFL->run(FORWARD);
    motorBR->run(FORWARD);
    motorBL->run(BACKWARD);

  }

  motorFR->setSpeed(turnSpeed);
  motorFL->setSpeed(turnSpeed);
  motorBR->setSpeed(turnSpeed);
  motorBL->setSpeed(turnSpeed);

  /*for ( int i = 0; i < 20; ++i)
    {
    readIMU();
    //printdata();
    }

    float wejscier = ToDeg(roll);
    float wejsciep = ToDeg(pitch);
    float wejsciey = ToDeg(yaw);*/

  for (int i = 0; i < 10; ++i)
  {
    /*for ( int i = 0; i < 20; ++i)
      {
      readIMU();
      }
      SerialUSB.println(ToDeg(roll));
      SerialUSB.println(ToDeg(pitch));
      SerialUSB.println(ToDeg(yaw));
      SerialUSB.println(" ");*/
    delay(turnTimems / 10);
  }

  /*float wyjscier = ToDeg(roll);
    float wyjsciep = ToDeg(pitch);
    float wyjsciey = ToDeg(yaw);

    SerialUSB.println(wyjscier - wejscier);
    SerialUSB.println(wyjsciep - wejsciep);
    SerialUSB.println(wyjsciey - wejsciey);*/

  motorFR->run(RELEASE);
  motorFL->run(RELEASE);
  motorBR->run(RELEASE);
  motorBL->run(RELEASE);
  return 27.0;

}

float turn(int turnDirection, float angle) //turning is tankwise
{
  //float correction = 1.7;
  float correction = 1.0;
  
  float copyAngle = angle / correction;
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
  float wyjsciey = ToDeg(yaw);

  int speed  = 128;
  motorFR->setSpeed(speed);
  motorFL->setSpeed(speed);
  motorBR->setSpeed(speed);
  motorBL->setSpeed(speed);

  while ( abs(wyjsciey - wejsciey) < abs(copyAngle))
  {
    
    for ( int i = 0; i < 10; ++i)
    {
      readIMU();
    }
    wyjsciey = ToDeg(yaw);
    //printdata();


    //s(sonar_forward,sonar_right,sonar_left)t(turn_in_degrees)E
    /*String stringToSend = "s(" + String(readSonicForward()) +
                          "," + String(readSonicRight()) +
                          "," + String(readSonicLeft()) + ")" +
                          "t(" + String(correction * abs(wyjsciey - wejsciey)) + ")" + END_OF_MESSAGE;*/
    String stringToSend = "s(" + String(wyjsciey) +
                          "," + String(wejsciey) +
                          "t(" + String(correction * abs(wyjsciey - wejsciey)) + ")" + END_OF_MESSAGE;
    SerialUSB.println(stringToSend);

    if ((wyjsciey - wejsciey) > 180)
      wyjsciey -= 360;
    else if ((wyjsciey - wejsciey) < -180)
      wyjsciey += 360;
  }
  motorFR->setSpeed(0);
  motorFL->setSpeed(0);
  motorBR->setSpeed(0);
  motorBL->setSpeed(0);

  /*float wyjscier = ToDeg(roll);
  float wyjsciep = ToDeg(pitch);
  // float wyjsciey = ToDeg(yaw);*/

  //SerialUSB.print("degrees: ");
  // SerialUSB.println(correction * abs(wyjsciey - wejsciey));

  motorFR->run(RELEASE);
  motorFL->run(RELEASE);
  motorBR->run(RELEASE);
  motorBL->run(RELEASE);
  return (correction * abs(wyjsciey - wejsciey));
}

float rotateCamera(int cameraDirection,int camSpeed) //rotates camera
{
    waitingSteps = cameraDirection * moveSteps / 16;
    stepper.setSpeed(camSpeed); //0-255  
    stepper.step(waitingSteps);
    //delay(4000);

  return 22.5;
}

void calibrate()
{
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  //SerialUSB.println("test3");
  delay(20);

  for (int i = 0; i < 32; i++) // We take some readings...
  {
    Read_Gyro();
    Read_Accel();
    //SerialUSB.println("test4");
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
}

void setup() {
  SerialUSB.begin(115200);           // set up SerialUSB library at 9600 bps

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

  initIMU();

  
}

void loop() {
  
  //readIMU();
  if (SerialUSB.available()) recieveUSB(); //call event function
  if (fromUSB.received) {
    String argument = "0";//distance or angle or values from sensor
    int integerArg = 0 ;
    if (fromUSB.command == "f")           //forward
    {
      moveStraight(GO_FORWARD, fromUSB.argument);
      argument = String(distanceDrivenL);
      integerArg = (int)distanceDrivenL;
      distanceDrivenL = 0;
    }
    else if (fromUSB.command  == "b")      //backward
    {
      moveStraight(GO_BACKWARD, fromUSB.argument);
      argument = String(distanceDrivenL);
      integerArg = (int)distanceDrivenL;
      distanceDrivenL = 0;
    }
    else if (fromUSB.command == "l")       //left
    {
      argument = String(turn(TURN_LEFT, fromUSB.argument));//turn left, get value and assign
      /*String(turn(TURN_LEFT, 550, 128));
      argument = String(30.0);
      integerArg = 30;*/
    }
    else if (fromUSB.command  == "r")      //right
    {
      argument = String(turn(TURN_RIGHT, fromUSB.argument));//turn right, get value and assign
      /*String(turn(TURN_RIGHT, 570, 128));
      argument = String(30.0);
      integerArg = 30;*/
    }
    else if (fromUSB.command  == "p")     //camera rotation right
    {
      argument = String(rotateCamera(1,fromUSB.argument));
      //turn(TURN_RIGHT, 20.0);
      //String(turn(TURN_RIGHT, 570, 128));
      //argument = String(22.5);
    }
    else if (fromUSB.command  == "q")     //camera rotation left
    {
      argument = String(rotateCamera(-1,fromUSB.argument));
      //turn(TURN_LEFT, 20.0);
      //String(turn(TURN_LEFT, 550, 128));
      //argument = String(22.5);
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
      int samplesNumber = 5;
      RunningMedian samplesForward = RunningMedian(samplesNumber );
      RunningMedian samplesRight = RunningMedian(samplesNumber );
      RunningMedian samplesLeft = RunningMedian(samplesNumber );
      for(int i = 0 ;i < samplesNumber ; i++)
      {
        samplesForward.add(readSonicForward());
        samplesRight.add(readSonicRight());
        samplesLeft.add(readSonicLeft());
      }
      argument = "(" + String(samplesForward.getMedian()) +
                 "," + String(samplesRight.getMedian()) +
                 "," + String(samplesLeft.getMedian()) + ")";

      //s(sonar_forward,sonar_right,sonar_left)E
      /*argument = "(" + String(readSonicForward()) +
                 "," + String(readSonicRight()) +
                 "," + String(readSonicLeft()) + ")";*/
    }
    else if (fromUSB.command == "c")
    {
      calibrate();
      argument = "Calibrated";
    }

    sendConfirmation(fromUSB.command, String(argument));
    clearDataFromUSB();
  }
}

