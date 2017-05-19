/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011-2016 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/

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

