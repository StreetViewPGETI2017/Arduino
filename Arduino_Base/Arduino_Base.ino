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

#define trigPinForward 2
#define echoPinForward 3
#define trigPinLeft 6
#define echoPinLeft 7
#define trigPinRight 4
#define echoPinRight 5

#define END_OF_MESSAGE 'E'
#define RX_SIZE 10
#define TX_SIZE 20

#define IMU_V5

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

struct dataFromUSB //structure for reading USB command, arguments
{
  String command = "";
  int argument = 0 ;
  bool received = 0;
};
dataFromUSB fromUSB;

struct DataToUSB {
  //counts how many rows is filled:
  int rowsIMU = 0;
  int rowsEncoder = 0;
  int rowsSonar = 0;
  //data storage:
  int imu[TX_SIZE][3];//accelerometer,magnetometer,gyroscope
  int imuRoll;
  int imuPitch;
  int imuYaw;
  int sonar[TX_SIZE][3];//forward,left,right
  int encoder[TX_SIZE][3];//only two encoders, but function accepts three
};
DataToUSB data;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motorFR = AFMS.getMotor(1); //create a motor instance
Adafruit_DCMotor *motorFL = AFMS.getMotor(2);
Adafruit_DCMotor *motorBL = AFMS.getMotor(3);
Adafruit_DCMotor *motorBR = AFMS.getMotor(4);

Servo cameraServo; // create servo object to control a servo

int servoTurn = 5;


void readIMU(){
  if((millis()-timer)>=20)  // Main loop runs at 50Hz
    {
      counter++;
      timer_old = timer;
      timer=millis();
      if (timer>timer_old)
      {
        G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
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
        counter=0;
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
    cameraServo.write(95.5+cameraDirection*1.5);
    //waitForRaspberry();
    delay(500+(-1+cameraDirection)*50);
    cameraServo.write(95.5);
    for (int j = 0; j <20; ++j)
    {
        readIMU();
        printdata();
    }
    delay(500);
}

void setup() {
  SerialUSB.begin(9600);           // set up SerialUSB library at 9600 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  cameraServo.attach(SERVO_PIN);
  cameraServo.write(95.5);

  pinMode(trigPinForward, OUTPUT);    // settings for sonic sensors
  pinMode(echoPinForward, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);

  //Serial.begin(115200);
  pinMode (STATUS_LED,OUTPUT);  // Status LED

  I2C_Init();

  digitalWrite(STATUS_LED,LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    SerialUSB.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED,HIGH);

  timer=millis();
  delay(20);
  counter=0;
  //your own initializattion code
}

void loop() {
  readIMU();
  if (SerialUSB.available()) recieveUSB(); //call event function
  if (fromUSB.received) {
    int confirmationArgument = 0;//distance or angle, changed on true value of move or rotation
    if (fromUSB.command == "f")           //forward
    {
      moveStraight(GO_FORWARD, 1000, 172);
      confirmationArgument = fromUSB.argument; //test value
    }
    else if (fromUSB.command  == "b")      //backward
    {
      moveStraight(GO_BACKWARD, 1000, 172);
      confirmationArgument = fromUSB.argument;; //test value
    }
    else if (fromUSB.command == "l")       //left
    {
      turn(TURN_LEFT, 1000, 128);
      confirmationArgument = fromUSB.argument;; //test value
    }
    else if (fromUSB.command  == "r")      //right
    {
      turn(TURN_RIGHT, 1000, 128);
      confirmationArgument = fromUSB.argument;; //test value
    }
    else if (fromUSB.command  == "p")     //camera rotation right
    {
      rotateCamera(1);
    }
    else if (fromUSB.command  == "q")     //camera rotation left
    {
      rotateCamera(-1);
    }
    else if (fromUSB.command == "i")      //IMU data sending
    {
     // readIMU();
      /*sendUSB(data.imu, 3, data.rowsIMU);
      memset(data.imu, 0, sizeof(data.imu)); //clear data
      data.rowsIMU = 0;*/
      SerialUSB.println(ToDeg(roll));
      SerialUSB.println(ToDeg(pitch));
      SerialUSB.println(ToDeg(yaw));
    }
    else if (fromUSB.command == "e")    //encoders data sending
    {
      sendUSB(data.encoder, 2, data.rowsEncoder);
      memset(data.encoder, 0, sizeof(data.encoder)); //clear data
      data.rowsEncoder = 0;
    }
    else if (fromUSB.command == "s")    //sonars data sending
    {
      SerialUSB.print("Forward: ");
      SerialUSB.println(readSonicForward());
      SerialUSB.print("Right: ");
      SerialUSB.println(readSonicRight());
      SerialUSB.print("Left: ");
      SerialUSB.println(readSonicLeft());
      /*sendUSB(data.sonar, 3, data.rowsSonar);
      memset(data.sonar, 0, sizeof(data.sonar)); //clear data
      data.rowsSonar = 0;*/
    }
    sendConfirmation(fromUSB.command, confirmationArgument);
    clearDataFromUSB();
  }
}

