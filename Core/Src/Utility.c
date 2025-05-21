/*
 * Utility.c
 *
 *  Created on: May 20, 2025
 *      Author: nino
 */
#include "Utility.h"

////////////////////////////////////////////////////////////////////////////////
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
////////////////////////////////////////////////////////////////////////////////
void matrixMultiply(uint8_t aRows, uint8_t aCols_bRows, uint8_t bCols, int16_t matrixC[], int16_t matrixA[], int16_t matrixB[])
{
    uint8_t i, j, k;

    for (i = 0; i < aRows * bCols; i++)
    {
        matrixC[i] = 0.0;
    }

    for (i = 0; i < aRows; i++)
    {
        for (j = 0; j < aCols_bRows; j++)
        {
            for (k = 0;  k < bCols; k++)
            {
                matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
            }
        }
    }
}
