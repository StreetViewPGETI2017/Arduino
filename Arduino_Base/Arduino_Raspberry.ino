void recieveUSB() //USB data receive event function
{
  char readChar;
  int i = 0;
  bool minus = false;//if argument is negative
  int power = 0, argument = 0;
  while (i < RX_SIZE) //waits to end of message or end of space
  {
    readChar = ' ';
    if (SerialUSB.available()) readChar = SerialUSB.read();
    if (readChar == END_OF_MESSAGE)break;
    else if (readChar != ' ')
    {
      int ascii = (int)readChar;
      if (ascii == 45)minus = true; //if char is '-'
      else if (ascii >= 48 && ascii <= 57) //if char is 0-9
      {
        ascii -= 48;//set as normal number
        argument += ascii * pow(10, power); //calculate positions of base 10 number
        power += 1;
      }
      else fromUSB.command = readChar;
      i++;
    }
  }
  if (minus)argument *= -1; //negative number
    fromUSB.argument = argument;
  fromUSB.received = true;
}
void sendUSB(int data[][3], int cells, int rows)
{
  //code for sending to Raspberry
  for (int i = 0; i < TX_SIZE && i < rows; i++) //send all rows
  {
    for (int j = 0; j < cells; j++) //send all cells in row
      SerialUSB.println(data[i][j]);
  }
  SerialUSB.println(END_OF_MESSAGE);
}
bool waitForRaspberry(int waitTime)
{
  int wait = 0;
  while (wait != waitTime) //waits for Raspberry confirmation
  {
    if (fromUSB.received &&  fromUSB.command == "w")
    {
      clearDataFromUSB();
      return 0;//success
    }
    else
    {
      wait++;
      delay(1);
    }
  }
  return 1;//failure
}
void clearDataFromUSB()
{
  fromUSB.command = ' ';
  fromUSB.argument = 0;
  fromUSB.received = false;
}
void sendConfirmation(String command, int argument)//confirms execution of command
{
  SerialUSB.println(command);
  SerialUSB.println(argument);
  SerialUSB.println(END_OF_MESSAGE);
}


