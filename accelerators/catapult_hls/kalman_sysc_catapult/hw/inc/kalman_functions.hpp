// Copyright (c) 2011-2023 Columbia University, System Level Design Group
// SPDX-License-Identifier: Apache-2.0

#ifndef __FUNCTIONS_HPP__
#define __FUNCTIONS_HPP__

#include "kalman.hpp"
void kalman_sysc_catapult::compute_req(uint32_t iter, uint32_t kalman_iters, uint32_t kalman_mat_dim, uint32_t constant_matrices_size, bool pingpong, bool out_pingpong)
{
    
        for (uint32_t i = 0; i < kalman_mat_dim; i++)
        {
            plm_RRq<in_as,inrp> rreq;
            rreq.indx[0]=i;
            if(pingpong)            
                in_ping_ra.Push(rreq);
            else
                in_pong_ra.Push(rreq);
        }
        
        for (uint32_t i = 0; i < constant_matrices_size; i++)
        {
            plm_RRq<in_as,inrp> rreq;
            rreq.indx[0]=i;
            if(pingpong)            
                in_b_ping_ra.Push(rreq);
            else
                in_b_pong_ra.Push(rreq);
        }

        if(iter > 0)
        {
            uint32_t previous_state_elements = kalman_mat_dim + (kalman_mat_dim * kalman_mat_dim);    
            for (uint32_t k = 0; k < previous_state_elements; k++)
            {
                plm_RRq<out_as,outrp> rreq;
                rreq.indx[0]=k;
                if (pingpong)
                    xp_ping_ra.Push(rreq);
                else
                    xp_pong_ra.Push(rreq);
            }
        }
        wait();
        //keep
        sync_comp.sync_out();
}


void kalman_sysc_catapult::compute(uint32_t iter, uint32_t kalman_iters, uint32_t kalman_mat_dim, 
                                    uint32_t phi_base_address, uint32_t Q_base_address, uint32_t H_base_address, uint32_t R_base_address, 
                                    uint32_t Pp_base_address, uint32_t constant_matrices_size, bool pingpong, bool out_pingpong)
{


    FPDATA  phi_cpp[const_mat_dim][const_mat_dim];
    FPDATA Q_cpp[const_mat_dim][const_mat_dim];
    FPDATA  H_cpp[const_mat_dim][const_mat_dim];
    FPDATA  R_cpp[const_mat_dim][const_mat_dim];
    FPDATA  Pp_cpp[const_mat_dim][const_mat_dim];


    

    FPDATA  X_cpp[const_mat_dim];
    FPDATA  Xp[const_mat_dim];

    FPDATA  Zk[const_mat_dim];
    FPDATA K_cpp[const_mat_dim][const_mat_dim];
    FPDATA tmp_mat1[const_mat_dim][const_mat_dim];
    FPDATA tmp_mat2[const_mat_dim][const_mat_dim];
    FPDATA tmp_mat3[const_mat_dim][const_mat_dim];
    FPDATA tmp_trans[const_mat_dim][const_mat_dim];
    FPDATA tmp_vec[const_mat_dim];
    FPDATA tmp_vec2[const_mat_dim];
    FPDATA L[const_mat_dim][const_mat_dim]; // Lower triangular matrix

    FPDATA_WORD important_matrices_word;
    FPDATA input_regs_fx;

    // FPDATA_WORD input_measurements_word[200];
    // FPDATA input_measurements_fx[200];

    FPDATA_WORD input_measurements_word;
    FPDATA input_measurements_fx;

    FPDATA zk_plus_1[const_mat_dim];
    FPDATA zk[const_mat_dim];

    // uint32_t current_address = 0;


    for (uint32_t k = 0; k < kalman_mat_dim; k++)
    {
        if(pingpong)            
            input_measurements_word=in_ping_rd.Pop().data[0];
        else
            input_measurements_word=in_pong_rd.Pop().data[0];            
        int2fx(input_measurements_word,input_measurements_fx);
            zk[k] = input_measurements_fx;

        // #ifdef PRINT_STATEMENTS
        //     cout << "Zk: ";
        //     print_vector(zk, kalman_mat_dim);
        // #endif
    }

    uint32_t rows = 0;
    uint32_t cols = 0;

    for (uint32_t current_address = 0; current_address < constant_matrices_size; current_address++)
    {
        if(pingpong)            
            important_matrices_word=in_b_ping_rd.Pop().data[0];
        else
            important_matrices_word=in_b_pong_rd.Pop().data[0];

        int2fx(important_matrices_word,input_regs_fx);
        if(current_address >= phi_base_address && current_address < Q_base_address)
        {
            phi_cpp[rows][cols] = input_regs_fx;
            cols++;
            if(cols >= kalman_mat_dim)
            {
                rows++;
                cols = 0; 
            }
            if(rows >= kalman_mat_dim)
            {
                rows = 0;
                cols = 0; 
            }
        }
        if(current_address >= Q_base_address && current_address < H_base_address)
        {
            Q_cpp[rows][cols] = input_regs_fx;
            cols++;
            if(cols >= kalman_mat_dim)
            {
                rows++;
                cols = 0; 
            }
            if(rows >= kalman_mat_dim)
            {
                rows = 0;
                cols = 0; 
            }
        }
        if(current_address >= H_base_address && current_address < R_base_address)
        {
            H_cpp[rows][cols] = input_regs_fx;
            cols++;
            if(cols >= kalman_mat_dim)
            {
                rows++;
                cols = 0; 
            }
            if(rows >= kalman_mat_dim)
            {
                rows = 0;
                cols = 0; 
            }
        }
        if(current_address >= R_base_address && current_address < Pp_base_address)
        {
            R_cpp[rows][cols] = input_regs_fx;
            cols++;
            if(cols >= kalman_mat_dim)
            {
                rows++;
                cols = 0; 
            }
            if(rows >= kalman_mat_dim)
            {
                rows = 0;
                cols = 0; 
            }
        }
        if(current_address >= Pp_base_address && current_address < constant_matrices_size)
        {
            Pp_cpp[rows][cols] = input_regs_fx;
            // cout << "PP LOOP[" << rows << "]\t" << current_address << "\t" << Pp_cpp[rows][cols] << "\n"; 
            cols++;
            if(cols >= kalman_mat_dim)
            {
                rows++;
                cols = 0; 
            }
            if(rows >= kalman_mat_dim)
            {
                rows = 0;
                cols = 0; 
            }
        }        
    }

        // #ifdef PRINT_STATEMENTS
        // std::cout << "PHI: " << iter << "\n";
        // print_matrix(phi_cpp, kalman_mat_dim);
        // #endif

        // #ifdef PRINT_STATEMENTS
        // std::cout << "Pp_cpp: " << iter << "\n";
        // print_matrix(Pp_cpp, kalman_mat_dim);
        // #endif

    if(iter == 0)
    {
        X_cpp[0] = zk[0];
        X_cpp[1] = zk[1];
        X_cpp[2] = zk[2];
        X_cpp[3] = zk[3];
        X_cpp[4] = zk[4];
        X_cpp[5] = zk[5];
    }


    FPDATA_WORD op_word;
    FPDATA op_fx;
    
    if(iter > 0)
    {
        int rows = 0;
        int cols = 0;
        uint32_t previous_states_elements = kalman_mat_dim + (kalman_mat_dim * kalman_mat_dim);    
        // for (uint32_t k = 0; k < kalman_mat_dim; k++)
        for (uint32_t k = 0; k < previous_states_elements; k++)
        {
            if (pingpong)
                op_word=xp_ping_rd.Pop().data[0];
            else
                op_word=xp_pong_rd.Pop().data[0];
            int2fx(op_word,op_fx);

            if(k < kalman_mat_dim)
            {
                X_cpp[k] = op_fx; //REMOVE PLUS ONE
            }

            if(k >= kalman_mat_dim)
            {
                Pp_cpp[rows][cols] = op_fx;
                cols++;
                if(cols >= kalman_mat_dim)
                {
                    rows++;
                    cols = 0; 
                }
                if(rows >= kalman_mat_dim)
                {
                    rows = 0;
                    cols = 0; 
                }
            }
        }

        #ifdef PRINT_STATEMENTS
        // std::cout << "Reading Pp_cpp: "<< iter << "\n";
        // print_matrix(Pp_cpp, kalman_mat_dim);
        #endif
    }

        // #ifdef PRINT_STATEMENTS
        // std::cout << "PHI: " << iter << "\n";
        // print_matrix(phi_cpp, kalman_mat_dim);
        // #endif

        // #ifdef PRINT_STATEMENTS
        // std::cout << "X_Cpp: " << iter << "\n";
        // print_vector(X_cpp, kalman_mat_dim);
        // #endif

        multiplyMatrixVector(phi_cpp, X_cpp, Xp, kalman_mat_dim); // Xp = phi * X0;

        // #ifdef PRINT_STATEMENTS
        // std::cout << "Xp\n";
        // print_vector(Xp, kalman_mat_dim);
        // #endif

        FPDATA phi_x_pp[const_mat_dim][const_mat_dim];
        FPDATA phi_trans[const_mat_dim][const_mat_dim];
        FPDATA temp_mat_x[const_mat_dim][const_mat_dim];

        // Pp = phi * Pp * phi' + Q
                    multiplyMatrices(phi_cpp, Pp_cpp, phi_x_pp, kalman_mat_dim); 
        // #ifdef PRINT_STATEMENTS
        // std::cout << "phi_x_pp\n";
        // print_matrix(phi_x_pp, kalman_mat_dim);
        // #endif
                    transposeMatrix(phi_cpp, phi_trans, kalman_mat_dim);
                    multiplyMatrices(phi_x_pp, phi_trans, temp_mat_x, kalman_mat_dim);
        // #ifdef PRINT_STATEMENTS
        // std::cout << "temp_mat_x\n";
        // print_matrix(temp_mat_x, kalman_mat_dim);
        // #endif
                    addMatrices(temp_mat_x, Q_cpp, Pp_cpp, kalman_mat_dim);
        // // End Pp = phi * Pp * phi' + Q
        // #ifdef PRINT_STATEMENTS
        // std::cout << "Pp\n";
        // print_matrix(Pp_cpp, kalman_mat_dim);
        // #endif



        FPDATA H_x_pp[const_mat_dim][const_mat_dim];
        FPDATA H_trans[const_mat_dim][const_mat_dim];
        FPDATA H_Pp_H_trans[const_mat_dim][const_mat_dim];
        FPDATA before_invv[const_mat_dim][const_mat_dim];
        FPDATA after_invv[const_mat_dim][const_mat_dim];
        FPDATA Pp_x_H_trans[const_mat_dim][const_mat_dim];

    // // Compute Kalman Gain
    //     // K = Pp * H' * inv(H * Pp * H' + R);
            multiplyMatrices(H_cpp, Pp_cpp, H_x_pp, kalman_mat_dim);  // H * Pp
            transposeMatrix(H_cpp, H_trans, kalman_mat_dim);          // H'
            multiplyMatrices(H_x_pp, H_trans, H_Pp_H_trans, kalman_mat_dim); // (H * Pp) * H'
            addMatrices(H_Pp_H_trans, R_cpp, before_invv, kalman_mat_dim); // H * Pp * H' + R
            gauss_inverse(before_invv, after_invv, kalman_mat_dim); // inv(H * Pp * H' + R)                    

            multiplyMatrices(Pp_cpp, H_trans, Pp_x_H_trans, kalman_mat_dim); // Pp * H'
            multiplyMatrices(Pp_x_H_trans, after_invv, K_cpp, kalman_mat_dim); // Pp * H' * inv(H * Pp * H' + R)
    //     // END COMPUTE KALMAN GAIN

    //     //Update
    //     // X(:,i+1) = Xp + K * (Zk - H * Xp);
        
            multiplyMatrixVector(H_cpp, Xp, tmp_vec, kalman_mat_dim); // H * Xp
            subtractVectors(zk, tmp_vec, tmp_vec2, kalman_mat_dim); // Zk - H * Xp

            multiplyMatrixVector(K_cpp, tmp_vec2, tmp_vec, kalman_mat_dim); // K * (Zk - H * Xp)
            addVectors(Xp, tmp_vec, X_cpp, kalman_mat_dim);

    //     //Update
    //     // Pp = Pp - K*H*Pp;
            multiplyMatrices(K_cpp, H_cpp, tmp_mat1, kalman_mat_dim); 
            multiplyMatrices(tmp_mat1, Pp_cpp, tmp_mat2, kalman_mat_dim);  // K*H*Pp
            subtractMatrices(Pp_cpp, tmp_mat2, tmp_mat1, kalman_mat_dim);
            copymat(tmp_mat1, Pp_cpp, kalman_mat_dim);
        // Update Pp done

        FPDATA send_output_array[100];
        
        uint32_t input_ptr;
        input_ptr = 0;
        uint32_t output_elements = kalman_mat_dim + (kalman_mat_dim * kalman_mat_dim);

        for (uint32_t k = 0; k < kalman_mat_dim; k++)
            send_output_array[k] = Xp[k];

        for (uint32_t i = 0; i < kalman_mat_dim; i++)
            for (uint32_t j = 0; j < kalman_mat_dim; j++)
                send_output_array[kalman_mat_dim + (i*kalman_mat_dim + j)] = Pp_cpp[i][j]; 

        for (uint32_t k = 0; k < output_elements; k++)
        {
            FPDATA_WORD out_word;
            fx2int(send_output_array[k], out_word);

            plm_WR<out_as,outwp> wreq;
            wreq.data[0]=out_word;
            wreq.indx[0]=input_ptr;

            if (out_pingpong)
                out_ping_w.Push(wreq);
            else
                out_pong_w.Push(wreq);
            input_ptr++;
        }
        input_ptr = 0;
        FPDATA next_state_arr_fx[100];
        for (uint32_t k = 0; k < kalman_mat_dim; k++)
            next_state_arr_fx[k] = X_cpp[k];

        for (uint32_t i = 0; i < kalman_mat_dim; i++)
            for (uint32_t j = 0; j < kalman_mat_dim; j++)
                next_state_arr_fx[kalman_mat_dim + (i*kalman_mat_dim + j)] = Pp_cpp[i][j]; 

        uint32_t next_state_size = kalman_mat_dim + (kalman_mat_dim * kalman_mat_dim);
        for (uint32_t k = 0; k < next_state_size; k++)
        {
            FPDATA_WORD xp_word;
            fx2int(next_state_arr_fx[k], xp_word);

            plm_WR<out_as,outwp> wreq1;
            wreq1.data[0]=xp_word;
            wreq1.indx[0]=input_ptr;

            if (out_pingpong)
                xp_ping_w.Push(wreq1);
            else
                xp_pong_w.Push(wreq1);
            input_ptr++;
        }
    wait();
    // //keep
    sync_comp.sync_in();
}

void plusone(FPDATA& regg) 
{
    regg = regg + 1;
}
#endif // __FUNCTIONS_HPP__
