//Copyright (c) 2011-2023 Columbia University, System Level Design Group
//SPDX-License-Identifier: Apache-2.0

#include "testbench.hpp"
#include "ac_math/ac_random.h"
#include <mc_connections.h>
#include <mc_scverify.h>

#include "data_init.h"

#define ERROR_THRESHOLD 0.01
int err=0;
FPDATA data;
float float_data;
FPDATA Pp_Final[matrix_dim][matrix_dim];
// uint32_t phi_base_address = 0;
// uint32_t Q_base_address = phi_base_address + (matrix_dim*matrix_dim);
// uint32_t H_base_address = Q_base_address + (matrix_dim*matrix_dim);
// uint32_t R_base_address = H_base_address + (matrix_dim*matrix_dim);
// uint32_t Pp_base_address = R_base_address + (matrix_dim*matrix_dim);
// uint32_t constant_matrices_size = Pp_base_address + (matrix_dim*matrix_dim);

// uint32_t measurement_vecs_base_address = constant_matrices_size;

// uint32_t input_vecs_total_size = measurement_vecs_base_address + 4*num_iterations;
// uint32_t output_total_size = 4*num_iterations;

void testbench::proc()
{
    conf_info.Reset();
    mac_n = 1;
    mac_vec = 100;
    mac_len = 64;
    kalman_iters = num_iterations; // Number of readings taken
    kalman_mat_dim = matrix_dim; // Number of num_iterations (matrix_dim: [X_GPS(i); X_pos(i); Y_GPS(i); Y_pos(i)])

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
    CCS_LOG("ESP - mac [Catapult HLS SystemC]");
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
    
    partition();
    std::cout << "Partition completed\n";

    print_variables();
    std::cout << "Print variables completed\n";

    single_input_array(); // Updates in_float by merging all the inputs
    std::cout << "Single input array completed\n";

    load_data(in_float, input_vecs_total_size); // Writes data in mem[i]
    std::cout << "Load datafloat done\n";

    // compute_golden();
    do_config();
    std::cout << "Do config done\n";

    dump_memory();
    std::cout << "Dump memory completed\n";
    int tot_errors = 0;

    validate();
    std::cout << "Validate completed\n";

    std::cout << FPDATA_WL;
    std::cout << "\t";
    std::cout << FPDATA_IL;
    std::cout << "\n";
    
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
        std::cout << "[" << measurement_vecs_base_address + matrix_dim*i + 3 << "]: " << master_array[measurement_vecs_base_address + matrix_dim*i + 3] << "\n";
    }

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


void testbench::compute_golden()
{
    //Compute golden output
}

   
void testbench::do_config()
{
    {
        conf_info_t config;

        /* <<--params-->> */
        config.mac_n = mac_n;
        config.mac_vec = mac_vec;
        config.mac_len = mac_len;
        config.kalman_iters = kalman_iters;
        config.kalman_mat_dim = kalman_mat_dim;

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
    std::cout << "Dump memory offset address: "<< offset << "\n";

    out= new ac_int<DATA_WIDTH,false>[out_size];
    out_float= new float[out_size];
    offset = offset / DMA_WORD_PER_BEAT;
    FPDATA out_res_fx = 0;
    for (uint32_t i = 0; i < out_size / DMA_WORD_PER_BEAT; i++)
    {
        for (uint32_t wordd = 0; wordd < DMA_WORD_PER_BEAT; wordd++)
        {
            out[i * DMA_WORD_PER_BEAT + wordd] = mem[offset + i].slc<DATA_WIDTH>(wordd*DATA_WIDTH);
        }
    }
}


void testbench::validate()
{
    for (uint32_t i = 0; i < mac_n; i++)
        for (uint32_t j = 0; j < output_size_per_iter*num_iterations; j++)
        {

            FPDATA out_gold_fx = 0;
            int2fx(gold[i * out_words_adj + j],out_gold_fx);
            FPDATA out_res_fx = 0;

            int2fx(out[i * out_words_adj + j],out_res_fx);

            if(j < output_size_per_iter*num_iterations)
            {
                    if(j%output_size_per_iter == 0)
                        std::cout << "\nXp[" << j/output_size_per_iter << "]:\t";
                    else if (j%output_size_per_iter == matrix_dim)
                    {
                        std::cout << "\nPp_Cpp[" << j/output_size_per_iter << "]:\t";                        
                    }
                    if (j%matrix_dim == 0)
                    {
                        std::cout << "\n";                        
                    }
                std::cout << std::setw(20) << out_res_fx << "\t";
            }
        }
        std::cout << "\n";
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