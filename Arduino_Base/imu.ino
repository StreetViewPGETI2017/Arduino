

void initIMU()
{
  /*
   * This function:
   * 1. Initializes I2C
   * 2. Initializes accelererometer
   * 3. Initializes compass
   * 4. Initializes gyroscope
   * 5. Calculates sensor offset
   * 6. Starts timer
   * 7. The overall result is fully initialized IMU
   */
  pinMode (STATUS_LED,OUTPUT);  // Status LED

  I2C_Init();

  SerialUSB.println("Pololu MinIMU-9 + Arduino AHRS");

  digitalWrite(STATUS_LED,LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  //offset calculation
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

  //SerialUSB.println("Offset:");
  for(int y=0; y<6; y++)
    SerialUSB.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED,HIGH);

  timer=millis();
  delay(20);
  counter=0;
}

float readIMU()
{
  /*
   * This function:
   * 1. Sets up frequency and time intervals for IMU readings
   * 2. Runs DCM algorithm
   * 3. Normalizes data, eliminates drift effect and calculates euler angles 
   * 4. Returns yaw angle (around z axis)
   */
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

    //printdata();
    return ToDeg(yaw);
  }
}

