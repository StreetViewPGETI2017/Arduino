void recieveUSB() //USB data receive event function
{
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
void sendConfirmation(String command, String argument)//confirms execution of command
{
  String allToSend = command + argument + END_OF_MESSAGE;
  SerialUSB.println(allToSend);
}


