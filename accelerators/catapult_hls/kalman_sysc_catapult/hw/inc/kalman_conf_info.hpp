// Copyright (c) 2011-2023 Columbia University, System Level Design Group
// SPDX-License-Identifier: Apache-2.0

#ifndef __CONF_INFO_HPP__
#define __CONF_INFO_HPP__

#pragma once

#include <sstream>
#include <ac_int.h>
#include <ac_fixed.h>
#include "kalman_specs.hpp"

//
// Configuration parameters for the accelerator.
//

struct conf_info_t
{

    /* <<--params-->> */
        int32_t mac_n;
        int32_t mac_vec;
        int32_t mac_len;

        int32_t kalman_iters; // Number of readings taken
        int32_t kalman_mat_rows; // Number of measurements (4: [X_GPS(i); X_pos(i); Y_GPS(i); Y_pos(i)])
        int32_t kalman_mat_cols; // Number of measurements (4: [X_GPS(i); X_pos(i); Y_GPS(i); Y_pos(i)])


        int32_t phi_base_address;
        int32_t Q_base_address;
        int32_t H_base_address;
        int32_t R_base_address;
        int32_t Pp_base_address;
        int32_t constant_matrices_size;

        int32_t measurement_vecs_base_address;

        int32_t input_vecs_total_size;
        int32_t output_total_size;

    static const unsigned int width = 32*15;
    template <unsigned int Size> void Marshall(Marshaller <Size> &m) {
        /* <<--marsh-->> */
        m &mac_n;
        m &mac_vec;
        m &mac_len;
        m &kalman_iters;
        m &kalman_mat_rows;    
        m &kalman_mat_cols;    

        m &phi_base_address;    
        m &Q_base_address;    
        m &H_base_address;    
        m &R_base_address;    
        m &Pp_base_address;    
        m &constant_matrices_size;    
        m &measurement_vecs_base_address;    
        m &input_vecs_total_size;    
        m &output_total_size;    

        }

    //
    // constructors
    //
    conf_info_t()
    {
        /* <<--ctor-->> */
        this->mac_n = 1;
        this->mac_vec = 100;
        this->mac_len = 64;

        this->kalman_iters = 1;
        this->kalman_mat_rows = 1;
        this->kalman_mat_cols = 1;

        this->phi_base_address = 1;
        this->Q_base_address = 1;
        this->H_base_address = 1;
        this->R_base_address = 1;
        this->Pp_base_address = 1;
        this->constant_matrices_size = 1;

        this->measurement_vecs_base_address = 1;
        this->input_vecs_total_size = 1;
        this->output_total_size = 1;  
    }

    conf_info_t(
        /* <<--ctor-args-->> */
        int32_t mac_n, 
        int32_t mac_vec, 
        int32_t mac_len,

        int32_t kalman_iters,
        int32_t kalman_mat_rows,
        int32_t kalman_mat_cols,
            
        int32_t phi_base_address,
        int32_t Q_base_address,
        int32_t H_base_address,
        int32_t R_base_address,
        int32_t Pp_base_address,
        int32_t constant_matrices_size,
        int32_t measurement_vecs_base_address,
        int32_t input_vecs_total_size,
        int32_t output_total_size    
               )
    {
        /* <<--ctor-custom-->> */
        this->mac_n = mac_n;
        this->mac_vec = mac_vec;
        this->mac_len = mac_len;
        this->kalman_iters = kalman_iters;
        this->kalman_mat_rows = kalman_mat_rows;
        this->kalman_mat_cols = kalman_mat_cols;

        this->phi_base_address = phi_base_address;
        this->Q_base_address = Q_base_address;
        this->H_base_address = H_base_address;
        this->R_base_address = R_base_address;
        this->Pp_base_address = Pp_base_address;
        this->constant_matrices_size = constant_matrices_size;
        this->measurement_vecs_base_address = measurement_vecs_base_address;
        this->input_vecs_total_size = input_vecs_total_size;
        this->output_total_size = output_total_size;
    }

    // VCD dumping function
   inline friend void sc_trace(sc_trace_file *tf, const conf_info_t &v, const std::string &NAME)
    {
        /* <<--sctrc-->> */
        sc_trace(tf,v.mac_n, NAME + ".mac_n");
        sc_trace(tf,v.mac_vec, NAME + ".mac_vec");
        sc_trace(tf,v.mac_len, NAME + ".mac_len");

        sc_trace(tf,v.kalman_iters, NAME + ".kalman_iters");
        sc_trace(tf,v.kalman_mat_rows, NAME + ".kalman_mat_rows");
        sc_trace(tf,v.kalman_mat_cols, NAME + ".kalman_mat_cols");

        sc_trace(tf,v.phi_base_address, NAME + ".phi_base_address");
        sc_trace(tf,v.Q_base_address, NAME + ".Q_base_address");
        sc_trace(tf,v.H_base_address, NAME + ".H_base_address");
        sc_trace(tf,v.R_base_address, NAME + ".R_base_address");
        sc_trace(tf,v.Pp_base_address, NAME + ".Pp_base_address");
        sc_trace(tf,v.constant_matrices_size, NAME + ".constant_matrices_size");
        sc_trace(tf,v.measurement_vecs_base_address, NAME + ".measurement_vecs_base_address");
        sc_trace(tf,v.input_vecs_total_size, NAME + ".input_vecs_total_size");
        sc_trace(tf,v.output_total_size, NAME + ".output_total_size");

    }

    // redirection operator
    friend ostream& operator << (ostream& os, conf_info_t const &conf_info)
    {
        os << "{";
        /* <<--print-->> */
        os << "mac_n = " << conf_info.mac_n << ", ";
        os << "mac_vec = " << conf_info.mac_vec << ", ";
        os << "mac_len = " << conf_info.mac_len << "";

        os << "kalman_iters = " << conf_info.kalman_iters << "";
        os << "kalman_mat_rows = " << conf_info.kalman_mat_rows << "";
        os << "kalman_mat_cols = " << conf_info.kalman_mat_rows << "";

        os << "phi_base_address = " << conf_info.phi_base_address << "";
        os << "Q_base_address = " << conf_info.Q_base_address << "";
        os << "H_base_address = " << conf_info.H_base_address << "";
        os << "R_base_address = " << conf_info.R_base_address << "";
        os << "Pp_base_address = " << conf_info.Pp_base_address << "";
        os << "constant_matrices_size = " << conf_info.constant_matrices_size << "";
        os << "measurement_vecs_base_address = " << conf_info.measurement_vecs_base_address << "";
        os << "input_vecs_total_size = " << conf_info.input_vecs_total_size << "";
        os << "output_total_size = " << conf_info.output_total_size << "";

        os << "}";
        return os;
    }

};

#endif // __MAC_CONF_INFO_HPP__
