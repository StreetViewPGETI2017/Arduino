void recieveUSB() //USB data receive event function
{
  String inString = "";
  for (int i=0; SerialUSB.available() && i < RX_SIZE; i++) 
  {
    inString = String(SerialUSB.read());
    if(inString == END_OF_MESSAGE)break;
    dataFromUSB[i] = inString;
  }
  received = true; 
}
void sendUSB(String data[][5],int cellsRow, int rows)
{
  //code for sending to Raspberry
  for(int i=0; i<TX_SIZE && i < rows;i++)//send all rows
  {
    for(int j=0; j<cellsRow;j++) //send all cells in row
       SerialUSB.println(data[i][j]);
  }
  SerialUSB.println(END_OF_MESSAGE);
}
bool waitForRaspberry(int waitTime)
{
  int wait = 0;
  while(wait != waitTime) //waits for Raspberry confirmation
  {
    if(received &&  dataFromUSB[0] == "w")
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
  for(int i=0; i<RX_SIZE; i++)
  dataFromUSB[i] = "";
  received = false;
}
void sendConfirmation(String command, int argument)//confirms about execution of command
{
   SerialUSB.println(command);
   SerialUSB.println(argument);
   SerialUSB.println(END_OF_MESSAGE);
}

