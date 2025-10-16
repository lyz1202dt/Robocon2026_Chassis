#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <stdio.h>
#include <math.h>
#include <arm_math.h>


#define M 8
#define N 3

int solve_linear_system_8x3(const float A_in[M][N], const float b_in[M][1], float x_out[N][1]);


#endif