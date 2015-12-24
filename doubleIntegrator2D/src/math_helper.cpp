#include"math_helper.h"

void printMatrix(double Mat[], int n, int m)
{
  for(int i = 0; i < n; i++) {
    for(int j = 0; j < m; j++) {
      cout << Mat[i*m+j] << " ";
    }
    cout << endl;
  }
}

void scalarMatMulti(double Mat[], double scalar, int n, int m)
{
  for(int i = 0; i < n; i++) {
    for(int j = 0; j < m; j++) {
      Mat[i*m+j] = Mat[i*m+j] * scalar;
    }
  }
}

void matrixSum(double Mat1[], double Mat2[], double MatSum[], int n, int m)
{
  for(int i = 0; i < n; i++) {
    for(int j = 0; j < m; j++) {
      MatSum[i*m+j] = Mat1[i*m+j] + Mat2[i*m+j];
    }
  }
}

