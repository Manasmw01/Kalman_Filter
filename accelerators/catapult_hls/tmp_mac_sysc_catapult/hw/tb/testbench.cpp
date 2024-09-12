//Copyright (c) 2011-2023 Columbia University, System Level Design Group
//SPDX-License-Identifier: Apache-2.0

#include "testbench.hpp"
#include "ac_math/ac_random.h"
#include <mc_connections.h>
#include <mc_scverify.h>
#define ERROR_THRESHOLD 0.01


void testbench::proc()
{
    conf_info.Reset();

    wait();

    CCS_LOG("--------------------------------");
    CCS_LOG("ESP - mac [Catapult HLS SystemC]");
    CCS_LOG("--------------------------------");

#if (DMA_WORD_PER_BEAT == 0)
    in_words_adj = mac_len*mac_vec;
    out_words_adj = mac_vec;
#else
    in_words_adj = round_up(mac_len*mac_vec, DMA_WORD_PER_BEAT);
    out_words_adj = round_up(mac_vec, DMA_WORD_PER_BEAT);
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
    FPDATA data;
    float float_data;

    //Initialize input
    // copy_array(data_tmpsamefloat, master_data, 0, 5);
    // copy_array(kalman_X_acc, master_data, 5, 5);
    for (uint32_t i= 0; i< mac_n ; i++)
    {
        for (uint32_t j=0; j< mac_len*mac_vec ; j+=1)
        {
            // data = data_tmpsame[j%20];
            float_data = data_tmpsamefloat[j%20];

            // float_data = master_data[j];
            cout << "Float data: " << float_data << "\n";
            // FPDATA_WORD data_int32;
            // data_int32.set_slc(0,data.slc<DATA_WIDTH>(0));
            // in[i*in_words_adj+j]=data_int32;
            in_float[i*in_words_adj+j]=float_data;
        }
    }

    //Compute golden output
    for (int i = 0; i < mac_n; i++)
            for (int j = 0; j < mac_vec; j++) {
            gold[i * out_words_adj + j] = 0;
            FPDATA acc=0;
            float acc_float = 0;
            for (int k = 0; k < mac_len; k += 2) {
                FPDATA data1;
                FPDATA data2;
                float data1_float = in_float[i * in_words_adj + j * mac_len + k];
                float data2_float = in_float[i * in_words_adj + j * mac_len + k + 1];
            // int2fx(in[i * in_words_adj + j * mac_len + k],data1);
            // int2fx(in[i * in_words_adj + j * mac_len + k + 1],data2);
            // acc+=data1*data2;
            acc_float+=data1_float*data2_float;
            if(i == 0 && (j == mac_vec - 1) && k == mac_len-2) 
            {
                std::cout << "Perfect Golden(" << j << "): " << data1_float <<" * "<< data2_float << " = "<< acc_float << "\n";
            }
        }
        FPDATA_WORD acc_int;
        fx2int(acc,acc_int);

            ac_ieee_float32 data = acc_float;
            FPDATA fpdata=data.convert_to_ac_fixed<FPDATA_WL,FPDATA_IL,true,AC_TRN, AC_WRAP>();

            FPDATA_WORD fpdata_word;

            fpdata_word.set_slc(0,fpdata.slc<FPDATA_WL>(0));
            // data_bv.set_slc(wordd*FPDATA_WL,fpdata_word);

            gold[i * out_words_adj + j] =fpdata_word;
     }
   
    // load_data(in, in_size);
    load_datafloat(in_float, in_size);


    CCS_LOG( "load memory completed");

    {
        conf_info_t config;

        /* <<--params-->> */
        config.mac_n = mac_n;
        config.mac_vec = mac_vec;
        config.mac_len = mac_len;
        config.kalman_n = kalman_n;
        config.kalman_mat_dim = kalman_mat_dim;

        conf_info.Push(config);
    }

    CCS_LOG( "config done");

    do { wait(); } while (!acc_done.read());

    int err=0;
    int offset=in_size;

    out= new ac_int<DATA_WIDTH,false>[out_size];
    out_float= new float[out_size];


    offset = offset / DMA_WORD_PER_BEAT;
    for (uint32_t i = 0; i < out_size / DMA_WORD_PER_BEAT; i++)
    {
        for (uint32_t wordd = 0; wordd < DMA_WORD_PER_BEAT; wordd++)
        {
            out[i * DMA_WORD_PER_BEAT + wordd] = mem[offset + i].slc<DATA_WIDTH>(wordd*DATA_WIDTH);
        }
    }



    CCS_LOG( "dump memory completed");
    int tot_errors = 0;

    for (uint32_t i = 0; i < mac_n; i++)
        for (uint32_t j = 0; j < mac_vec; j++)
        {
            FPDATA out_gold_fx = 0;
            int2fx(gold[i * out_words_adj + j],out_gold_fx);
            // if(i == 0 && (j >= (mac_vec - 1)*3/4)) 
            //     std::cout << "GOLD MEM(" << j << "): " << out_gold_fx << "\n";
            FPDATA out_res_fx = 0;

            int2fx(out[i * out_words_adj + j],out_res_fx);
                // if(i == 0 && (j == mac_vec - 1)) 
            // if(i == 0 && (j >= (mac_vec - 1)*3/4)) 
            //     std::cout << "OUT MEM(" << j << "): " << out_res_fx << "\n";

            if (out_gold_fx != out_res_fx)
            {
                FPDATA MSE = (out_res_fx-out_gold_fx)*(out_res_fx-out_gold_fx)
                    / out_gold_fx;

                if (MSE > ERROR_THRESHOLD)
                {
                    err++;
                }
            }
        }

    if (err > 0)
        CCS_LOG("SIMULATION FAILED ! \n - errors "<< err);
    else
        CCS_LOG("SIMULATION PASSED ! ");

    sc_stop();

    wait();
}

void testbench::load_datafloat(float *inn, uint32_t inn_size)
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

void testbench::copy_array(float* a, float* b, int offset_b, int n)
{
    for (int i = 0; i < n; i++) { 
        b[offset_b + i] = a[i]; 
    }     
}

