long readSonicForward() {
  /*
   * This function:
   * 1. Reads data from forward sonic sensor
   * 2. It converts it to centimeteres and returns the result
   */
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
  /*
   * This function:
   * 1. Reads data from right sonic sensor
   * 2. It converts it to centimeteres and returns the value
   */
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
  /*
   * This function:
   * 1. Reads data from left sonic sensor
   * 2. It converts it to centimeteres and returns the value
   */
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
  /*
   * This function
   * 1. Detects obstacles using sonic sensors. It is an emergency function that prevents robot from crashing into walls.
   * 2. Returns 1 when obstacle was detected and 0 when not.
   */
  int counter = 0;
  for (int j = 0; j < 10; ++j)
    if (moveDirection == GO_FORWARD &&  readSonicForward() < distance)
      ++counter;
    else if (moveDirection == TURN_LEFT && readSonicLeft() < distance)
      ++counter;
    else if (moveDirection == TURN_RIGHT && readSonicRight() < distance)
      ++counter;
  if (counter > 5 )
  {
    // SerialUSB.println("obstacle");
    return 1;
  }
  return 0;
}

