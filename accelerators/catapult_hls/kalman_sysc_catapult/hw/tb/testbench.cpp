//Copyright (c) 2011-2023 Columbia University, System Level Design Group
//SPDX-License-Identifier: Apache-2.0

#include "testbench.hpp"
#include "ac_math/ac_random.h"
#include <mc_connections.h>
#include <mc_scverify.h>

#include "data_init.h"
std::ofstream ofs;
std::ifstream ifs;

#define ERROR_THRESHOLD 0.1
int err=0;
FPDATA data;
float float_data;
FPDATA Pp_Final[matrix_dim][matrix_dim];


#ifdef GOLDEN_OP
inline void print_matrix_golden(float matrixx[N][N], uint32_t kalman_mat_rows)
{
    for (int i = 0; i < kalman_mat_rows; i++)
        for (int j = 0; j < kalman_mat_rows; j++)
            std::cout << std::setw(20) << matrixx[i][j] << ((j == kalman_mat_rows - 1) ? "\n": "\t");
    std::cout << "\n";
}

inline void print_vector_golden(float vec[N] , uint32_t kalman_mat_rows)
{
    for (int i = 0; i < kalman_mat_rows; i++)
            std::cout << std::setw(20) << vec[i] << "\t";
    std::cout << "\n";
}
inline void gauss_inverse_golden(float A[N][N], float inverse[N][N], uint32_t kalman_mat_rows) {
    // Augmenting the matrix A with identity matrix of same dimensions
    float augmented[N][2*N];
    for (int i = 0; i < kalman_mat_rows; i++) {
        for (int j = 0; j < kalman_mat_rows; j++) {
            augmented[i][j] = A[i][j];
            // Set the corresponding element in the augmented matrix to 1 if on the diagonal, 0 otherwise
            // augmented[i][j + N] = (i == j) ? 1.0 : 0.0;

            if(i == j)
                augmented[i][j + kalman_mat_rows] = 1;                
            else
                augmented[i][j + kalman_mat_rows] = 0;                
        }
    }

    // Applying Gauss-Jordan elimination
    for (int i = 0; i < kalman_mat_rows; i++) {
        // Find the pivot row and swap
        int pivot_row = i;
        for (int j = i + 1; j < kalman_mat_rows; j++) 
        {
            if (augmented[j][i] > augmented[pivot_row][i]) 
            {
                pivot_row = j;
            }
        }
        // Swap rows i and pivot_row
        if (pivot_row != i) 
        {
            for (int k = 0; k < 2*kalman_mat_rows; k++) 
            {
                float temp = augmented[i][k];
                // augmented[i][k] = augmented[pivot_row][k];
                float temp2 = augmented[pivot_row][k];
                augmented[i][k] = temp2;
                augmented[pivot_row][k] = temp;
            }
        }

        // Make the diagonal elements 1
        float pivot = augmented[i][i];
        for (int k = 0; k < 2*kalman_mat_rows; k++) 
        {
            float temp = augmented[i][k] / pivot;
            augmented[i][k] = temp;
        }

        // Make other elements in the column 0
        for (int j = 0; j < kalman_mat_rows; j++) 
        {
            if (j != i) 
            {
                float factor = augmented[j][i];
                for (int k = 0; k < 2*kalman_mat_rows; k++) {
                    // augmented[j][k] -= factor * augmented[i][k];
                    float temp = factor * augmented[i][k];
                    float temp2 = augmented[j][k] - temp;
                    augmented[j][k] = temp2;
                }
            }
        }
    }

    // Copy the inverse from the augmented matrix
    for (int i = 0; i < kalman_mat_rows; i++) 
    {
        for (int j = 0; j < kalman_mat_rows; j++) 
        {
            inverse[i][j] = augmented[i][j + kalman_mat_rows];
        }
    }
}


inline void multiplyMatrices_golden(float A[N][N], float B[N][N], float result[N][N], uint32_t kalman_mat_rows) {
    for (int i = 0; i < kalman_mat_rows; i++) {
        for (int j = 0; j < kalman_mat_rows; j++) {
            result[i][j] = 0;
            for (int k = 0; k < kalman_mat_rows; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


inline void multiplyMatrixVector_golden(float A[N][N], float vector[N], float result[N], uint32_t kalman_mat_rows) {
    for (int i = 0; i < kalman_mat_rows; i++) {
        result[i] = 0;
        for (int j = 0; j < kalman_mat_rows; j++) {
            result[i] += A[i][j] * vector[j];
        }
    }
}

inline void transposeMatrix_golden(float matrix[N][N], float result[N][N], uint32_t kalman_mat_rows) {
    for (int i = 0; i < kalman_mat_rows; i++) {
        for (int j = 0; j < kalman_mat_rows; j++) {
            result[j][i] = matrix[i][j];
        }
    }
}

inline void addMatrices_golden(float A[N][N], float B[N][N], float result[N][N], uint32_t kalman_mat_rows) {
    for (int i = 0; i < kalman_mat_rows; i++) {
        for (int j = 0; j < kalman_mat_rows; j++) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}

inline void subtractMatrices_golden(float A[N][N], float B[N][N], float result[N][N], uint32_t kalman_mat_rows) {
    for (int i = 0; i < kalman_mat_rows; i++) {
        for (int j = 0; j < kalman_mat_rows; j++) {
            result[i][j] = A[i][j] - B[i][j];
        }
    }
}

inline void copymat_golden(float A[N][N], float result[N][N], uint32_t kalman_mat_rows) {
    for (int i = 0; i < kalman_mat_rows; i++) {
        for (int j = 0; j < kalman_mat_rows; j++) {
            result[i][j] = A[i][j];
        }
    }
}

inline void subtractVectors_golden(float vector1[N], float vector2[N], float result[N], uint32_t kalman_mat_rows) {
    for (int i = 0; i < kalman_mat_rows; i++) {
        result[i] = vector1[i] - vector2[i];
    }
}

inline void addVectors_golden(float vector1[N], float vector2[N], float result[N], uint32_t kalman_mat_rows) {
    for (int i = 0; i < kalman_mat_rows; i++) {
        result[i] = vector1[i] + vector2[i];
    }
}
#endif


void testbench::proc()
{
    conf_info.Reset();
    mac_n = 1;
    mac_vec = 100;
    mac_len = 64;
    kalman_iters = num_iterations; // Number of readings taken
    kalman_mat_rows = matrix_dim; // Number of num_iterations (matrix_dim: [X_GPS(i); X_pos(i); Y_GPS(i); Y_pos(i)])
    kalman_mat_cols = matrix_dim; // Number of num_iterations (matrix_dim: [X_GPS(i); X_pos(i); Y_GPS(i); Y_pos(i)])

    phi_base_address = 0;
    Q_base_address = phi_base_address + (matrix_dim*matrix_dim);
    H_base_address = Q_base_address + (matrix_dim*matrix_dim);
    R_base_address = H_base_address + (matrix_dim*matrix_dim);
    Pp_base_address = R_base_address + (matrix_dim*matrix_dim);
    constant_matrices_size = Pp_base_address + (matrix_dim*matrix_dim);

    measurement_vecs_base_address = constant_matrices_size;

    input_vecs_total_size = measurement_vecs_base_address + matrix_dim*num_iterations;
    output_size_per_iter = matrix_dim + matrix_dim*matrix_dim;
    output_total_size = output_size_per_iter*num_iterations;

    wait();

    CCS_LOG("--------------------------------");
    CCS_LOG("ESP - KALMAN [Catapult HLS SystemC]");
    CCS_LOG("--------------------------------");

#if (DMA_WORD_PER_BEAT == 0)
    in_words_adj = input_vecs_total_size;
    out_words_adj = output_total_size;
#else
    // in_words_adj = round_up(mac_len*mac_vec, DMA_WORD_PER_BEAT);
    // out_words_adj = round_up(mac_vec, DMA_WORD_PER_BEAT);
    in_words_adj = round_up(input_vecs_total_size, DMA_WORD_PER_BEAT);
    out_words_adj = round_up(output_total_size, DMA_WORD_PER_BEAT);
#endif

    in_size = in_words_adj * (mac_n);
    out_size = out_words_adj * (mac_n);

    CCS_LOG( "  - DMA width: "<< DMA_WIDTH);
    CCS_LOG( "  - DATA width: "<< DATA_WIDTH);
    CCS_LOG("--------------------------------");


    in = new ac_int<DATA_WIDTH,false>[in_size];
    in_float = new float[in_size];
    gold= new ac_int<DATA_WIDTH,false>[out_size];
    gold_float = new float[out_size];
    master_array = new float[in_size];
    golden_array = new float[out_size];
    
    partition();
    std::cout << "Partition completed\n";

    print_variables();
    std::cout << "Print variables completed\n";

    single_input_array(); // Updates in_float by merging all the inputs
    std::cout << "Single input array completed\n";

    load_data(in_float, input_vecs_total_size); // Writes data in mem[i]
    std::cout << "Load datafloat done\n";

#ifdef GOLDEN_OP
    compute_golden();
#endif
    do_config();
    std::cout << "Do config done\n";

    dump_memory();
    std::cout << "Dump memory completed\n";

    validate();

    
    sc_stop();
    wait();
}

void testbench::load_data(float *inn, uint32_t inn_size)
{
    for (uint32_t i = 0; i < inn_size / DMA_WORD_PER_BEAT; i++)  {
        ac_int<DMA_WIDTH> data_bv;
        for (int wordd = 0; wordd < DMA_WORD_PER_BEAT; wordd++)
        {
            ac_ieee_float32 data = inn[i* DMA_WORD_PER_BEAT + wordd];

            FPDATA fpdata=data.convert_to_ac_fixed<FPDATA_WL,FPDATA_IL,true,AC_TRN, AC_WRAP>();
            FPDATA_WORD fpdata_word;
            fpdata_word.set_slc(0,fpdata.slc<FPDATA_WL>(0));
            data_bv.set_slc(wordd*FPDATA_WL,fpdata_word);
        }
        mem[i] = data_bv;
    }
}

void testbench::partition()
{
    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            master_array[phi_base_address + (i*matrix_dim + j)] = phi[i][j]; 

    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            master_array[Q_base_address + (i*matrix_dim + j)] = Q[i][j]; 

    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            master_array[H_base_address + (i*matrix_dim + j)] = H[i][j]; 

    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            master_array[R_base_address + (i*matrix_dim + j)] = R[i][j]; 

    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            master_array[Pp_base_address + (i*matrix_dim + j)] = Pp[i][j]; 

    for (int i = 0; i < num_iterations; i++)
    {
        master_array[measurement_vecs_base_address + matrix_dim*i + 0] = x_acc[i]; 
        master_array[measurement_vecs_base_address + matrix_dim*i + 1] = x_gps[i]; 
        master_array[measurement_vecs_base_address + matrix_dim*i + 2] = y_acc[i]; 
        master_array[measurement_vecs_base_address + matrix_dim*i + 3] = y_gps[i]; 

#if(matrix_dim == 6)
        master_array[measurement_vecs_base_address + matrix_dim*i + 4] = x_acc[i]; 
        master_array[measurement_vecs_base_address + matrix_dim*i + 5] = x_gps[i];         
#endif
    }
}

void testbench::print_variables()
{
        std::cout << "\nphi: [" << phi_base_address << "]\n";
    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            std::cout << master_array[phi_base_address + (i*matrix_dim + j)] << ((j == matrix_dim - 1) ? "\n": "\t");

    std::cout << "\nQ: [" << Q_base_address << "]\n";
    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            std::cout << master_array[Q_base_address + (i*matrix_dim + j)] << ((j == matrix_dim - 1) ? "\n": "\t");

    std::cout << "\nH: [" << H_base_address << "]\n";
    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            std::cout << master_array[H_base_address + (i*matrix_dim + j)] << ((j == matrix_dim - 1) ? "\n": "\t");

    std::cout << "\nR: [" << R_base_address << "]\n";
    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            std::cout << master_array[R_base_address + (i*matrix_dim + j)] << ((j == matrix_dim - 1) ? "\n": "\t");

    std::cout << "\nPp: [" << Pp_base_address << "]\n";
    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            std::cout << master_array[Pp_base_address + (i*matrix_dim + j)] << ((j == matrix_dim - 1) ? "\n": "\t");

    std::cout << "\nMeasurement vectors\n";
    for (uint32_t i= 0; i< num_iterations ; i++)
    {
        std::cout << "[" << measurement_vecs_base_address + matrix_dim*i + 0 << "]: " << master_array[measurement_vecs_base_address + matrix_dim*i + 0] << "\t";
        std::cout << "[" << measurement_vecs_base_address + matrix_dim*i + 1 << "]: " << master_array[measurement_vecs_base_address + matrix_dim*i + 1] << "\t";
        std::cout << "[" << measurement_vecs_base_address + matrix_dim*i + 2 << "]: " << master_array[measurement_vecs_base_address + matrix_dim*i + 2] << "\t";
        std::cout << "[" << measurement_vecs_base_address + matrix_dim*i + 3 << "]: " << master_array[measurement_vecs_base_address + matrix_dim*i + 3] << "\t";
    #if(matrix_dim == 6)
        std::cout << "[" << measurement_vecs_base_address + matrix_dim*i + 4 << "]: " << master_array[measurement_vecs_base_address + matrix_dim*i + 4] << "\t";
        std::cout << "[" << measurement_vecs_base_address + matrix_dim*i + 5 << "]: " << master_array[measurement_vecs_base_address + matrix_dim*i + 5] << "\n";
    #endif
    }
    std::cout << "\n";

    std::cout << "\nconstant_matrices_size: "<< constant_matrices_size << "\n" ;
    std::cout << "input_vecs_total_size: "<< input_vecs_total_size << "\n" ;
    std::cout << "output_total_size: "<< output_total_size << "\n";

    std::cout << "phi_base_address: "<< phi_base_address << "\n" ;
    std::cout << "Q_base_address: "<< Q_base_address << "\n" ;
    std::cout << "H_base_address: "<< H_base_address << "\n";
    std::cout << "R_base_address: "<< R_base_address << "\n" ;
    std::cout << "Pp_base_address: "<< Pp_base_address << "\n" ;
    std::cout << "constant_matrices_size: "<< constant_matrices_size << "\n";

    phi_base_address = 0;
    Q_base_address = phi_base_address + (matrix_dim*matrix_dim);
    H_base_address = Q_base_address + (matrix_dim*matrix_dim);
    R_base_address = H_base_address + (matrix_dim*matrix_dim);
    Pp_base_address = R_base_address + (matrix_dim*matrix_dim);
    constant_matrices_size = Pp_base_address + (matrix_dim*matrix_dim);

    measurement_vecs_base_address = constant_matrices_size;

    input_vecs_total_size = measurement_vecs_base_address + matrix_dim*num_iterations;
}


#ifdef GOLDEN_OP
void testbench::compute_golden()
{
    // //Compute golden output
    float  phi_cpp[const_mat_dim][const_mat_dim];
    float Q_cpp[const_mat_dim][const_mat_dim];
    float  H_cpp[const_mat_dim][const_mat_dim];
    float  R_cpp[const_mat_dim][const_mat_dim];
    float  Pp_cpp[const_mat_dim][const_mat_dim];
    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            phi_cpp[i][j] = phi[i][j]; 

    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            Q_cpp[i][j] = Q[i][j]; 

    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            H_cpp[i][j] = H[i][j]; 

    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            R_cpp[i][j] = R[i][j]; 

    for (int i = 0; i < matrix_dim; i++)
        for (int j = 0; j < matrix_dim; j++)
            Pp_cpp[i][j] = Pp[i][j]; 

  

    float  X[const_mat_dim];
    float  Xp[const_mat_dim];

    float  Zk[const_mat_dim];
    float K[const_mat_dim][const_mat_dim];
    float tmp_mat1[const_mat_dim][const_mat_dim];
    float tmp_mat2[const_mat_dim][const_mat_dim];
    float tmp_mat3[const_mat_dim][const_mat_dim];
    float tmp_trans[const_mat_dim][const_mat_dim];
    float tmp_vec[const_mat_dim];
    float tmp_vec2[const_mat_dim];
    float L[const_mat_dim][const_mat_dim]; // Lower triangular matrix

    float zk[const_mat_dim];


    for (uint16_t iter = 0; iter < kalman_iters; iter++)
    {
        if(iter == 0)
        {
            X[0] = x_acc[iter];
            X[1] = x_gps[iter];
            X[2] = y_acc[iter];
            X[3] = y_gps[iter];
#if(matrix_dim == 6)
            X[4] = x_acc[iter];
            X[5] = x_gps[iter];
#endif
        }

        multiplyMatrixVector_golden(phi_cpp, X, Xp, matrix_dim); // Xp = phi * X0;
        // #ifdef PRINT_STATEMENTS
        // std::cout << "PP TEST\n";
        // print_matrix_golden(Pp_cpp, matrix_dim);
        // #endif

        float phi_x_pp[const_mat_dim][const_mat_dim];
        float phi_trans[const_mat_dim][const_mat_dim];
        float temp_mat_x[const_mat_dim][const_mat_dim];

        // Pp = phi * Pp * phi' + Q
                    multiplyMatrices_golden(phi_cpp, Pp_cpp, phi_x_pp, matrix_dim); 
                    transposeMatrix_golden(phi_cpp, phi_trans, matrix_dim);
                    multiplyMatrices_golden(phi_x_pp, phi_trans, temp_mat_x, matrix_dim);
                    addMatrices_golden(temp_mat_x, Q_cpp, Pp_cpp, matrix_dim);
        // // End Pp = phi * Pp * phi' + Q
        // #ifdef PRINT_STATEMENTS
        // std::cout << "Pp Golden\n";
        // print_matrix_golden(Pp_cpp, matrix_dim);
        // #endif



        float H_x_pp[const_mat_dim][const_mat_dim];
        float H_trans[const_mat_dim][const_mat_dim];
        float H_Pp_H_trans[const_mat_dim][const_mat_dim];
        float before_invv[const_mat_dim][const_mat_dim];
        float after_invv[const_mat_dim][const_mat_dim];
        float Pp_x_H_trans[const_mat_dim][const_mat_dim];

    // // Compute Kalman Gain
    //     // K = Pp * H' * inv(H * Pp * H' + R);
            multiplyMatrices_golden(H_cpp, Pp_cpp, H_x_pp, matrix_dim);  // H * Pp
            transposeMatrix_golden(H_cpp, H_trans, matrix_dim);          // H'
            multiplyMatrices_golden(H_x_pp, H_trans, H_Pp_H_trans, matrix_dim); // (H * Pp) * H'
            addMatrices_golden(H_Pp_H_trans, R_cpp, before_invv, matrix_dim); // H * Pp * H' + R
            gauss_inverse_golden(before_invv, after_invv, matrix_dim); // inv(H * Pp * H' + R)                    

            multiplyMatrices_golden(Pp_cpp, H_trans, Pp_x_H_trans, matrix_dim); // Pp * H'
            multiplyMatrices_golden(Pp_x_H_trans, after_invv, K, matrix_dim); // Pp * H' * inv(H * Pp * H' + R)
    //     // END COMPUTE KALMAN GAIN
        // #ifdef PRINT_STATEMENTS
        // std::cout << "K Golden\n";
        // print_matrix_golden(K, matrix_dim);
        // #endif




    //     //Update
    //     // X(:,i+1) = Xp + K * (Zk - H * Xp);
        
            multiplyMatrixVector_golden(H_cpp, Xp, tmp_vec, matrix_dim); // H * Xp
            subtractVectors_golden(zk, tmp_vec, tmp_vec2, matrix_dim); // Zk - H * Xp

            multiplyMatrixVector_golden(K, tmp_vec2, tmp_vec, matrix_dim); // K * (Zk - H * Xp)
            addVectors_golden(Xp, tmp_vec, X, matrix_dim);
        // #ifdef PRINT_STATEMENTS
        // std::cout << "X Golden\n";
        // print_vector_golden(X, matrix_dim);
        // #endif



    //     //Update
    //     // Pp = Pp - K*H*Pp;
            multiplyMatrices_golden(K, H_cpp, tmp_mat1, matrix_dim); 
            multiplyMatrices_golden(tmp_mat1, Pp_cpp, tmp_mat2, matrix_dim);  // K*H*Pp
            subtractMatrices_golden(Pp_cpp, tmp_mat2, tmp_mat1, matrix_dim);
            copymat_golden(tmp_mat1, Pp_cpp, matrix_dim);
        // Update Pp done


    // ofs.close();

    }

        std::cout << "Pp Golden: \n";
        print_matrix_golden(Pp_cpp, matrix_dim);
        cout << "\n";


    float golden_output_array[out_size];
        for (uint32_t i = 0; i < matrix_dim; i++)
            for (uint32_t j = 0; j < matrix_dim; j++)
                golden_output_array[i*matrix_dim + j] = Pp_cpp[i][j]; 

    uint32_t output_elements = (matrix_dim * matrix_dim);
    ofs.open("golden_output.txt", std::ofstream::out);
    for (uint32_t i = 0; i < output_elements; i++)
    {
            ofs << golden_output_array[i] << std::endl;
    }
    ofs.close();

}
#endif


void testbench::do_config()
{
    {
        conf_info_t config;

        /* <<--params-->> */
        config.mac_n = mac_n;
        config.mac_vec = mac_vec;
        config.mac_len = mac_len;
        config.kalman_iters = kalman_iters;
        config.kalman_mat_rows = kalman_mat_rows;

        config.phi_base_address = phi_base_address;
        config.Q_base_address = Q_base_address;
        config.H_base_address = H_base_address;
        config.R_base_address = R_base_address;
        config.Pp_base_address = Pp_base_address;
        config.constant_matrices_size = constant_matrices_size;
        config.measurement_vecs_base_address = measurement_vecs_base_address;

        config.input_vecs_total_size = input_vecs_total_size;
        config.output_total_size = output_total_size;

        conf_info.Push(config);
    }
}

void testbench::dump_memory()
{
    std::cout << "Entered Dump Memory\n";
    do 
    {
        wait(); 
    } while (!acc_done.read());
    int offset=in_size;
    // std::cout << "Dump memory offset address: "<< offset << "\n";

    out= new ac_int<DATA_WIDTH,false>[out_size];
    out_float= new float[out_size];

    offset = offset / DMA_WORD_PER_BEAT;
    ofs.open("accelerator_output.txt", std::ofstream::out);
    for (uint32_t i = 0; i < out_size / DMA_WORD_PER_BEAT; i++)
    {
        for (uint32_t wordd = 0; wordd < DMA_WORD_PER_BEAT; wordd++)
        {
            out[i * DMA_WORD_PER_BEAT + wordd] = mem[offset + i].slc<DATA_WIDTH>(wordd*DATA_WIDTH);
            FPDATA out_fixed = 0;
            int2fx(out[i * DMA_WORD_PER_BEAT + wordd],out_fixed);
            if(i >= out_size - (matrix_dim*matrix_dim))
            {
                ofs << out_fixed << std::endl;
            }

            // ofs << i << ": " << out[i * DMA_WORD_PER_BEAT + wordd] << std::endl;
        }
    }
    ofs.close();
}


void testbench::validate()
{
    int tot_errors = 0;

    for (uint32_t i = 0; i < mac_n; i++)
        for (uint32_t j = 0; j < output_size_per_iter*num_iterations; j++)
        {

            FPDATA out_gold_fx = 0;
            int2fx(gold[i * out_words_adj + j],out_gold_fx);

            FPDATA out_res_fx = 0;
            int2fx(out[i * out_words_adj + j],out_res_fx);

// cout << "\nTESTTT\n";
// cout << output_size_per_iter*num_iterations << "\n";
// cout << output_size_per_iter*num_iterations - output_size_per_iter << "\n";
            if(j >= output_size_per_iter*num_iterations - output_size_per_iter)
            {
                    if(j%output_size_per_iter == 0)
                        std::cout << "\nXp_design[" << j/output_size_per_iter << "]:\t";
                    else if (j%output_size_per_iter == matrix_dim)
                    {
                        std::cout << "\nPp_design[" << j/output_size_per_iter << "]:\t";                        
                    }
                    if (j%matrix_dim == 0)
                    {
                        std::cout << "\n";                        
                    }
                std::cout << std::setw(20) << out_res_fx << "\t";
            }
        }
    float accelerator_validate_array[100];
    float golden_validate_array[100];

    FILE *file;
    char line[100]; // Adjust the size as per your needs
    // Open the file in read mode
    file = fopen("golden_output.txt", "r");
    if (file == NULL) {
        printf("Error opening the file.\n");
    }

    // Read each line from the file
    uint32_t ind = 0;
    while (fgets(line, sizeof(line), file)) {
        // Convert the line to a float
        float value = strtof(line, NULL);
        // Check if the conversion was successful
        if (value != 0.0f || (value == 0.0f && line[0] == '0')) {
            golden_validate_array[ind] = value;
            // Print the float value
            // printf("%f\n", value);
        } 
        ind++;
    }
    // Close the file
    fclose(file);


    // Open the file in read mode
    file = fopen("accelerator_output.txt", "r");
    if (file == NULL) {
        printf("Error opening the file.\n");
    }

    // Read each line from the file
    ind = 0;
    while (fgets(line, sizeof(line), file)) {
        // Convert the line to a float
        float value = strtof(line, NULL);
        // Check if the conversion was successful
        if (value != 0.0f || (value == 0.0f && line[0] == '0')) {
            accelerator_validate_array[ind] = value;
            // Print the float value
            // printf("%f\n", value);
        } 
        ind++;
    }
    // Close the file
    fclose(file);

    for(int i = 0; i < matrix_dim*matrix_dim; i++)
    {
        // cout << accelerator_validate_array[i] << "\t" << golden_validate_array[i] << "\n";
          if (accelerator_validate_array[i] != golden_validate_array[i])
        {
            float MSE = (accelerator_validate_array[i]-golden_validate_array[i])*(accelerator_validate_array[i]-golden_validate_array[i])
                / golden_validate_array[i];

            if (MSE > ERROR_THRESHOLD)
            {
                printf("L2: output[%d] = %f (expected: %f)",
                            i, accelerator_validate_array[i], golden_validate_array[i]);
                tot_errors += 1;
            }

       
        }
    }
    if (tot_errors == 0)
    {
        printf("------------------------------------\n");
        printf("  Validation succeeded!  \n");
        printf("------------------------------------\n");
    }
    else
    {
        printf("------------------------------------\n");
        printf("  Validation failed!  \n");
        printf("------------------------------------\n");
    }     
}

void testbench::single_input_array()
{
    for (uint32_t i= 0; i< mac_n ; i++)
    {
        for (uint32_t j=0; j< in_size ; j+=1)
        {
            float_data = master_array[j];
            FPDATA_WORD data_int32;
            data_int32.set_slc(0,data.slc<DATA_WIDTH>(0));
            in_float[i*in_words_adj+j]=float_data;
        }
    }
}
