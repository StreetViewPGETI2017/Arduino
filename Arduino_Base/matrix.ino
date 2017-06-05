
void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3])
/*
 * This function:
 * 1. Multiplies two 3x3 matrixes
 */
{
  for(int x = 0; x < 3; x++)
  {
    for(int y = 0; y < 3; y++)
    {
      mat[x][y] = 0;
      
      for(int w = 0; w < 3; w++)
      {
        mat[x][y] += a[x][w] * b[w][y];
      } 
    }
  }
}


