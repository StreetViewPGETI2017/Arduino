void recieveUSB() //USB data receive event function
{
  /*
   * This function:
   * 1. Reads data from USB in correct format
   */
  char readChar;
  String numberStr = "";
  int i = 0;

  while (i < RX_SIZE) //waits to end of message or end of space
  {
    readChar = ' ';
    if (SerialUSB.available())
    {
      readChar = SerialUSB.read();
      i++;
    }
    if (readChar == END_OF_MESSAGE)break;
    else if (readChar != ' ')
    {
      int ascii = (int)readChar;
      if ((ascii >= 48 && ascii <= 57) || ascii == 45 ) //if char is 0-9 or minus
        numberStr += readChar;//connect number parts made from single chars
      else fromUSB.command = readChar;
    }
  }
  fromUSB.argument = numberStr.toInt();
  fromUSB.received = true;
}
bool waitForRaspberry(int waitTime)
{
  /*
   * This function:
   * 1. Waits for Raspberry's confirmation
   * 2. It returns 0 if Raspberry confirmed action and 1 if not
   */
  int wait = 0;
  while (wait != waitTime)
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
  /*
   * This function:
   * 1. Clears data from USB
   */
  fromUSB.command = ' ';
  fromUSB.argument = 0;
  fromUSB.received = false;
}
void sendConfirmation(String command, String argument)//confirms execution of command
{
  /*
   * This function:
   * 1. Sends confirmaton that the action requested by Raspberry was successful or not
   */
  SerialUSB.println(command);
  SerialUSB.println(argument);
  SerialUSB.println(END_OF_MESSAGE);
}


