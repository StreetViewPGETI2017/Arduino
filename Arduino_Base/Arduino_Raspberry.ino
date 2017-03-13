
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
void sendUSB(String data[])
{
  //code for sending to Raspberry
  for(int i=0; i<TX_SIZE && i < dataCounter;i++)
  {
    SerialUSB.println(data[i]);
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

