#ifndef __INITDATA__
#define __INITDATA__

#include "testbench.hpp"
#include "ac_math/ac_random.h"
#include <mc_connections.h>
#include <mc_scverify.h>


#include "csv_time.h"
#include "csv_x_acc.h"
#include "csv_x_gps.h"
#include "csv_y_acc.h"
#include "csv_y_gps.h"


uint32_t offset_timestamp = 0;
float dt = 0.01;

    float  phi[matrix_dim][matrix_dim] = {
        {1, dt, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, dt},
        {0, 0, 0, 1}
    };
    // float  phi[matrix_dim][matrix_dim] = {
    //     {0, 1, 2, 3},
    //     {4, 5, 6, 7},
    //     {8, 9, 10, 11},
    //     {12, 13, 14, 15}
    // };


    float Q[matrix_dim][matrix_dim] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
  
    float  H[matrix_dim][matrix_dim] = {
        {1, 0, 0, 0},
        {1, 0, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 0}
    };

    float  R[matrix_dim][matrix_dim] = {
        {25, 0, 0, 0},
        {0, 400, 0, 0},
        {0, 0, 25, 0},
        {0, 0, 0, 400}
    };  

    float  Pp[matrix_dim][matrix_dim] = {
        {200, 0, 0, 0},
        {0, 200, 0, 0},
        {0, 0, 200, 0},
        {0, 0, 0, 200}
    };  

#endif