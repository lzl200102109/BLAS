/*
 * math_helper.h
 *
 *  Created on: Dec 22, 2015
 *      Author: zhilong
 */

#ifndef MATH_HELPER_H_
#define MATH_HELPER_H_

#include <iostream>
using namespace std;


const int N = 4;
const int M = 2;

void printMatrix(double Mat[], int n, int m);
void scalarMatMulti(double Mat[], double scalar, int n, int m);
void matrixSum(double Mat1[], double Mat2[], double MatSum[], int n, int m);

#endif /* MATH_HELPER_H_ */
