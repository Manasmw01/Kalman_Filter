// Copyright (c) 2011-2023 Columbia University, System Level Design Group
// SPDX-License-Identifier: Apache-2.0

#ifndef __DATATYPES__
#define __DATATYPES__

// #define PRINT_STATEMENTS
#define GOLDEN_OP
#include "ac_int.h"
#include "ac_fixed.h"
#include "ac_std_float.h"
#include "ac_math.h"
#include "kalman_specs.hpp"


#define N const_mat_dim
#define FPDATA_WL DATA_WIDTH
// #define FPDATA_IL 23
#define FPDATA_IL DATA_WIDTH/2

typedef ac_int<DMA_WIDTH> DMA_WORD;
typedef ac_int<FPDATA_WL> FPDATA_WORD;
typedef ac_fixed<FPDATA_WL, FPDATA_IL> FPDATA;

inline void int2fx(const FPDATA_WORD& in, FPDATA& out)
{ out.set_slc(0,in.slc<FPDATA_WL>(0)); }

inline void fx2int(const FPDATA& in, FPDATA_WORD& out)
{ out.set_slc(0,in.slc<FPDATA_WL>(0)); }


#ifdef PRINT_STATEMENTS
inline void print_matrix(FPDATA matrixx[N][N], uint32_t kalman_mat_dim)
{
    for (int i = 0; i < kalman_mat_dim; i++)
        for (int j = 0; j < kalman_mat_dim; j++)
            std::cout << std::setw(20) << matrixx[i][j] << ((j == kalman_mat_dim - 1) ? "\n": "\t");
    std::cout << "\n";
}

inline void print_vector(FPDATA vec[N] , uint32_t kalman_mat_dim)
{
    for (int i = 0; i < kalman_mat_dim; i++)
            std::cout << std::setw(20) << vec[i] << "\t";
    std::cout << "\n";
}
#endif
inline void gauss_inverse(FPDATA A[N][N], FPDATA inverse[N][N], uint32_t kalman_mat_dim) {
    // Augmenting the matrix A with identity matrix of same dimensions
    // FPDATA augmented[N][2*N];
    FPDATA augmented[kalman_mat_dim][2*kalman_mat_dim];
        // #ifdef PRINT_STATEMENTS
        // std::cout << "A: " << "\n";
        // print_matrix(A, kalman_mat_dim);
        // #endif

    for (int i = 0; i < kalman_mat_dim; i++) {
        for (int j = 0; j < kalman_mat_dim; j++) {
            augmented[i][j] = A[i][j];
            // Set the corresponding element in the augmented matrix to 1 if on the diagonal, 0 otherwise
            // augmented[i][j + N] = (i == j) ? 1.0 : 0.0;

            if(i == j)
                augmented[i][j + kalman_mat_dim] = 1;                
            else
                augmented[i][j + kalman_mat_dim] = 0;                
        }
    }
    // for (int i = 0; i < kalman_mat_dim; i++)
    //     for (int j = 0; j < 2*kalman_mat_dim; j++)
    //         std::cout << augmented[i][j] << ((j == 2*kalman_mat_dim - 1) ? "\n": "\t");
    // std::cout << "\n";

    // Applying Gauss-Jordan elimination
    for (int i = 0; i < kalman_mat_dim; i++) 
    {
        // Find the pivot row and swap
        int pivot_row = i;
        for (int j = i + 1; j < kalman_mat_dim; j++) 
        {
            if (augmented[j][i] > augmented[pivot_row][i]) 
            {
                pivot_row = j;
            }
        }
        // Swap rows i and pivot_row
        if (pivot_row != i) 
        {
            for (int k = 0; k < 2*kalman_mat_dim; k++) 
            {
                FPDATA temp = augmented[i][k];
                // augmented[i][k] = augmented[pivot_row][k];
                FPDATA temp2 = augmented[pivot_row][k];
                augmented[i][k] = temp2;
                augmented[pivot_row][k] = temp;
            }

        }

        // Make the diagonal elements 1
        FPDATA pivot = augmented[i][i];
        for (int k = 0; k < 2*kalman_mat_dim; k++) 
        {
            FPDATA temp = augmented[i][k] / pivot;
            augmented[i][k] = temp;
        }

        // Make other elements in the column 0
        for (int j = 0; j < kalman_mat_dim; j++) 
        {
            if (j != i) 
            {
                FPDATA factor = augmented[j][i];
                for (int k = 0; k < 2*kalman_mat_dim; k++) {
                    // std::cout << i << "\t" << j << "\t" << k << "\n";
                    // augmented[j][k] -= factor * augmented[i][k];
                    FPDATA temp = factor * augmented[i][k];
                    FPDATA temp2 = augmented[j][k] - temp;
                    augmented[j][k] = temp2;
                }
            }
        }
    }

    // Copy the inverse from the augmented matrix
    for (int i = 0; i < kalman_mat_dim; i++) 
    {
        for (int j = 0; j < kalman_mat_dim; j++) 
        {
            inverse[i][j] = augmented[i][j + kalman_mat_dim];
        }
    }
}


inline void multiplyMatrices(FPDATA A[N][N], FPDATA B[N][N], FPDATA result[N][N], uint32_t kalman_mat_dim) {
    for (int i = 0; i < kalman_mat_dim; i++) {
        for (int j = 0; j < kalman_mat_dim; j++) {
            result[i][j] = 0;
            for (int k = 0; k < kalman_mat_dim; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


inline void multiplyMatrixVector(FPDATA A[N][N], FPDATA vector[N], FPDATA result[N], uint32_t kalman_mat_dim) {
    for (int i = 0; i < kalman_mat_dim; i++) {
        result[i] = 0;
        for (int j = 0; j < kalman_mat_dim; j++) {
            result[i] += A[i][j] * vector[j];
        }
    }
}

inline void transposeMatrix(FPDATA matrix[N][N], FPDATA result[N][N], uint32_t kalman_mat_dim) {
    for (int i = 0; i < kalman_mat_dim; i++) {
        for (int j = 0; j < kalman_mat_dim; j++) {
            result[j][i] = matrix[i][j];
        }
    }
}

inline void addMatrices(FPDATA A[N][N], FPDATA B[N][N], FPDATA result[N][N], uint32_t kalman_mat_dim) {
    for (int i = 0; i < kalman_mat_dim; i++) {
        for (int j = 0; j < kalman_mat_dim; j++) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}

inline void subtractMatrices(FPDATA A[N][N], FPDATA B[N][N], FPDATA result[N][N], uint32_t kalman_mat_dim) {
    for (int i = 0; i < kalman_mat_dim; i++) {
        for (int j = 0; j < kalman_mat_dim; j++) {
            result[i][j] = A[i][j] - B[i][j];
        }
    }
}

inline void copymat(FPDATA A[N][N], FPDATA result[N][N], uint32_t kalman_mat_dim) {
    for (int i = 0; i < kalman_mat_dim; i++) {
        for (int j = 0; j < kalman_mat_dim; j++) {
            result[i][j] = A[i][j];
        }
    }
}

inline void subtractVectors(FPDATA vector1[N], FPDATA vector2[N], FPDATA result[N], uint32_t kalman_mat_dim) {
    for (int i = 0; i < kalman_mat_dim; i++) {
        result[i] = vector1[i] - vector2[i];
    }
}

inline void addVectors(FPDATA vector1[N], FPDATA vector2[N], FPDATA result[N], uint32_t kalman_mat_dim) {
    for (int i = 0; i < kalman_mat_dim; i++) {
        result[i] = vector1[i] + vector2[i];
    }
}



#endif
