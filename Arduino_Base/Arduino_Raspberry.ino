
void recieveUSB() //USB data receive event function
{
  while (SerialUSB.available()) 
  {
    // get the new byte:
    char inChar = (char)SerialUSB.read();
    // add it to the inputString:
    inputString += inChar;
    stringComplete = true; //one char working version
  }
}
int sendUSB()
{
  //code for sending to Raspberry
  return 1;
}
bool waitForRaspberry(int waitTime)
{
  int wait = 0;
  while(wait != waitTime) //waits for Raspberry confirmation default 10 seconds
  {
    if(stringComplete &&  inputString == "w")
    {
      inputString = "";
      stringComplete = false;
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

