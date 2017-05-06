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

