void printdata(void)
{   
      SerialUSB.print("!");

      #if PRINT_EULER == 1
      SerialUSB.print("ANG:");
      SerialUSB.print(ToDeg(roll));
      SerialUSB.print(",");
      SerialUSB.print(ToDeg(pitch));
      SerialUSB.print(",");
      SerialUSB.print(ToDeg(yaw));
      #endif      
      #if PRINT_ANALOGS==1
      SerialUSB.print(",AN:");
      SerialUSB.print(AN[0]);  //(int)read_adc(0)
      SerialUSB.print(",");
      SerialUSB.print(AN[1]);
      SerialUSB.print(",");
      SerialUSB.print(AN[2]);  
      SerialUSB.print(",");
      SerialUSB.print(AN[3]);
      SerialUSB.print (",");
      SerialUSB.print(AN[4]);
      SerialUSB.print (",");
      SerialUSB.print(AN[5]);
      SerialUSB.print(",");
      SerialUSB.print(c_magnetom_x);
      SerialUSB.print (",");
      SerialUSB.print(c_magnetom_y);
      SerialUSB.print (",");
      SerialUSB.print(c_magnetom_z);
      #endif
      #if PRINT_DCM == 1
      SerialUSB.print (",DCM:");
      SerialUSB.print(DCM_Matrix[0][0]);
      SerialUSB.print (",");
      SerialUSB.print(DCM_Matrix[0][1]);
      SerialUSB.print (",");
      SerialUSB.print(DCM_Matrix[0][2]);
      SerialUSB.print (",");
      SerialUSB.print(DCM_Matrix[1][0]);
      SerialUSB.print (",");
      SerialUSB.print(DCM_Matrix[1][1]);
      SerialUSB.print (",");
      SerialUSB.print(DCM_Matrix[1][2]);
      SerialUSB.print (",");
      SerialUSB.print(DCM_Matrix[2][0]);
      SerialUSB.print (",");
      SerialUSB.print(DCM_Matrix[2][1]);
      SerialUSB.print (",");
      SerialUSB.print(DCM_Matrix[2][2]);
      #endif
      SerialUSB.println();
      
}

/*long convert_to_dec(float x)
{
  return x*10000000;
}*/
