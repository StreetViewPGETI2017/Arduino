
float Vector_Dot_Product(float vector1[3],float vector2[3])
{
  /*
   * This function:
   * 1. Computes the dot product of two vectors and returns it
   */
  float op=0;
  
  for(int c=0; c<3; c++)
  {
  op+=vector1[c]*vector2[c];
  }
  
  return op; 
}

void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
  /*
   * This function:
   * 1. Computes the cross product of two vectors
   */
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
  /*
   * This function:
   * 1. Multiplies the vector by a scalar
   */
  for(int c=0; c<3; c++)
  {
   vectorOut[c]=vectorIn[c]*scale2; 
  }
}

void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
  /*
   * This function:
   * 1. Adds two vectors
   */
  for(int c=0; c<3; c++)
  {
     vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }
}



