void recieveUSB() //USB data receive event function
{
  char inString;
  for (int i = 0; i < RX_SIZE; i++)
  {
    inString = SerialUSB.read();
    if (inString == 'x')i--;
    else{
       if (inString == END_OF_MESSAGE)break;  
       dataFromUSB[i] = inString;
    }
    inString = 'x';
  }
  received = true;
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
    if (received &&  dataFromUSB[0] == "w")
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
  for (int i = 0; i < RX_SIZE; i++)
    dataFromUSB[i] = "";
  received = false;
}
void sendConfirmation(String command, int argument)//confirms execution of command
{
  SerialUSB.println(command);
  SerialUSB.println(argument);
  SerialUSB.println(END_OF_MESSAGE);
}

