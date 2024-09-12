//Copyright (c) 2011-2023 Columbia University, System Level Design Group
//SPDX-License-Identifier: Apache-2.0

#include "mac_2.hpp"
#include <mc_scverify.h>

#include "mac_2_utils.hpp"
#include "mac_2_functions.hpp"

void mac_2_sysc_catapult:: config() {
    conf_info.Reset();
    conf1.ResetWrite();
    conf2.ResetWrite();
    conf2b.ResetWrite();
    conf3.ResetWrite();
    conf3b.ResetWrite();

    sync01.ResetWrite();
    sync02.ResetWrite();
    sync02b.ResetWrite();
    sync03.ResetWrite();
    sync03b.ResetWrite();

    wait();
    while(1) {
        conf_info_t conf_di = conf_info.Pop();

        conf1.Push(conf_di);
        conf2.Push(conf_di);
        conf2b.Push(conf_di);
        conf3.Push(conf_di);
        conf3b.Push(conf_di);

        sync01.Push(1);
        sync02.Push(1);
        sync02b.Push(1);
        sync03.Push(1);
        sync03b.Push(1);

    }
}



void mac_2_sysc_catapult:: load() {
    bool ping_pong = true;
    dma_read_chnl.Reset();
    dma_read_ctrl.Reset();

    sync01.ResetRead();
    sync12.reset_sync_out();
    sync12b.reset_sync_out();

    conf1.ResetRead();

    in_ping_w.ResetWrite();
    in_pong_w.ResetWrite();
    in_b_ping_w.ResetWrite();
    in_b_pong_w.ResetWrite();

    wait();

    while(1) {
        sync01.Pop();
        uint32_t offset = 0;

        conf_info_t conf=conf1.Pop();

        /* <<--local-params-->> */
        uint32_t mac_n = conf.mac_n;
        uint32_t mac_vec = conf.mac_vec;
        uint32_t mac_len = conf.mac_len;
        uint32_t kalman_iters = conf.kalman_iters;
        uint32_t kalman_mat_dim = conf.kalman_mat_dim;

        uint32_t inputs_base_address;
        uint32_t regs_base_address = 0;
        uint32_t regs_size = 5*kalman_mat_dim* kalman_mat_dim;

        load_b(ping_pong, regs_base_address, regs_size);
        for (uint16_t iter = 0; iter < kalman_iters; iter++)
        {
            // load_d(ping_pong, regs_base_address, regs_size);
            inputs_base_address = regs_size + (iter * kalman_mat_dim);

            #ifdef PRINT_STATEMENTS
            // cout << "Load_d: " << inputs_base_address << "\t" << kalman_mat_dim << "\t" << (inputs_base_address + kalman_mat_dim) << "\n";
            #endif
            load_d(ping_pong, inputs_base_address, kalman_mat_dim);
            sync12.sync_out();
            sync12b.sync_out();
            // ping_pong = !ping_pong;
        }
    }
}

void mac_2_sysc_catapult::compute_dataReq() {
    bool ping_pong = true;
    bool out_ping_pong = true;
    sync12.reset_sync_in();
    sync23.reset_sync_out();
    sync23b.reset_sync_out();

    sync02.ResetRead();
    conf2.ResetRead();

    sync_comp.reset_sync_out();

    in_ping_ra.ResetWrite();
    in_pong_ra.ResetWrite();
    in_b_ping_ra.ResetWrite();
    in_b_pong_ra.ResetWrite();

    xp_ping_ra.ResetWrite();
    xp_pong_ra.ResetWrite();


    wait();

    while(1) 
    {

        sync02.Pop();

        conf_info_t conf=conf2.Pop();

        /* <<--local-params-->> */
        uint32_t mac_n = conf.mac_n;
        uint32_t mac_vec = conf.mac_vec;
        uint32_t mac_len = conf.mac_len;

        uint32_t kalman_iters = conf.kalman_iters;
        uint32_t kalman_mat_dim = conf.kalman_mat_dim;
        uint32_t constant_matrices_size = conf.constant_matrices_size;

        for (uint16_t iter = 0; iter < kalman_iters; iter++)
        {
            sync12.sync_in();
            compute_req(iter, kalman_iters, kalman_mat_dim, constant_matrices_size, ping_pong, out_ping_pong);
            sync23.sync_out();
            sync23b.sync_out();
            // ping_pong = !ping_pong;
            // out_ping_pong = !out_ping_pong;
        }
    }
}

void mac_2_sysc_catapult:: compute() {
    bool ping_pong = true;
    bool out_ping_pong = true;
    sync12b.reset_sync_in();
    sync2b3.reset_sync_out();
    sync2b3b.reset_sync_out();

    sync02b.ResetRead();
    conf2b.ResetRead();

    sync_comp.reset_sync_in();

    xp_ping_w.ResetWrite();
    xp_pong_w.ResetWrite();
    out_ping_w.ResetWrite();
    out_pong_w.ResetWrite();
    in_ping_rd.ResetRead();
    in_pong_rd.ResetRead();
    in_b_ping_rd.ResetRead();
    in_b_pong_rd.ResetRead();

    xp_ping_rd.ResetRead();
    xp_pong_rd.ResetRead();


    wait();

    while(1) {

        sync02b.Pop();

        conf_info_t conf=conf2b.Pop();

        /* <<--local-params-->> */
        uint32_t mac_n = conf.mac_n;
        uint32_t mac_vec = conf.mac_vec;
        uint32_t mac_len = conf.mac_len;
        uint32_t kalman_iters = conf.kalman_iters;
        uint32_t kalman_mat_dim = conf.kalman_mat_dim;
        uint32_t constant_matrices_size = conf.constant_matrices_size;

        uint32_t phi_base_address = conf.phi_base_address;
        uint32_t Q_base_address = conf.Q_base_address;
        uint32_t H_base_address = conf.H_base_address;
        uint32_t R_base_address = conf.R_base_address;
        uint32_t Pp_base_address = conf.Pp_base_address;

        for (uint16_t iter = 0; iter < kalman_iters; iter++)
        {
            sync12b.sync_in();
            compute(iter, kalman_iters, kalman_mat_dim, 
                    phi_base_address, Q_base_address, H_base_address, R_base_address, Pp_base_address, 
                    constant_matrices_size, ping_pong, out_ping_pong);
            sync2b3.sync_out();
            sync2b3b.sync_out();
            // ping_pong = !ping_pong;
            // out_ping_pong = !out_ping_pong;
        }
    }
}

void mac_2_sysc_catapult:: store_dataReq() {
    bool ping_pong = true;
    sync23.reset_sync_in();
    sync2b3.reset_sync_in();
    sync03.ResetRead();
    conf3.ResetRead();
    out_pong_ra.ResetWrite();
    out_ping_ra.ResetWrite();
    wait();
    while(1) {

        sync03.Pop();
        conf_info_t conf=conf3.Pop();

        /* <<--local-params-->> */
        uint32_t mac_n = conf.mac_n;
        uint32_t mac_vec = conf.mac_vec;
        uint32_t mac_len = conf.mac_len;
        uint32_t kalman_iters = conf.kalman_iters;
        uint32_t kalman_mat_dim = conf.kalman_mat_dim;
        uint32_t input_vecs_total_size = conf.input_vecs_total_size;

        uint32_t out_index;

        const int out_len = kalman_mat_dim + kalman_mat_dim*kalman_mat_dim;
        for (uint32_t b = 0; b < kalman_iters; b++)
        {
            out_index = input_vecs_total_size + out_len*b;
            sync23.sync_in();
            sync2b3.sync_in();
            store_data_req(ping_pong, out_index, out_len);
        }
        // ping_pong = !ping_pong;
    }
}


void mac_2_sysc_catapult:: store() {
    bool ping_pong = true;
    dma_write_chnl.Reset();
    dma_write_ctrl.Reset();

    sync23b.reset_sync_in();
    sync2b3b.reset_sync_in();

    sync03b.ResetRead();
    conf3b.ResetRead();

    out_pong_rd.ResetRead();
    out_ping_rd.ResetRead();

    acc_done.write(false);

    wait();

    while(1) {

        sync03b.Pop();
        conf_info_t conf=conf3b.Pop();

        /* <<--local-params-->> */
        uint32_t mac_n = conf.mac_n;
        uint32_t mac_vec = conf.mac_vec;
        uint32_t mac_len = conf.mac_len;
        uint32_t kalman_iters = conf.kalman_iters;
        uint32_t kalman_mat_dim = conf.kalman_mat_dim;
        uint32_t input_vecs_total_size = conf.input_vecs_total_size;

        uint32_t out_index;
        const int out_len = kalman_mat_dim + kalman_mat_dim*kalman_mat_dim;
        for (uint32_t b = 0; b < kalman_iters; b++)
        {
            out_index = input_vecs_total_size + out_len*b;
            sync23b.sync_in();
            sync2b3b.sync_in();
            out_index = input_vecs_total_size + out_len*b;
            #ifdef PRINT_STATEMENTS
            cout << "store_data: (" << out_index << " " << out_len << ")\t" << (out_index + out_len) << "\n";
            #endif
            store_data(ping_pong, out_index, out_len);
        }
        // ping_pong = !ping_pong;
        acc_done.write(true); wait();
        acc_done.write(false);
    }
}




