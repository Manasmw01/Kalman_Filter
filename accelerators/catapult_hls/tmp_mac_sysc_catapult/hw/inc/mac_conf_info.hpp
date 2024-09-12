// Copyright (c) 2011-2023 Columbia University, System Level Design Group
// SPDX-License-Identifier: Apache-2.0

#ifndef __CONF_INFO_HPP__
#define __CONF_INFO_HPP__

#pragma once

#include <sstream>
#include <ac_int.h>
#include <ac_fixed.h>
#include "mac_specs.hpp"

//
// Configuration parameters for the accelerator.
//

struct conf_info_t
{

    /* <<--params-->> */
        int32_t mac_n;
        int32_t mac_vec;
        int32_t mac_len;

        int32_t kalman_n; // Number of readings taken
        int32_t kalman_mat_dim; // Number of measurements (4: [X_GPS(i); X_pos(i); Y_GPS(i); Y_pos(i)])


    static const unsigned int width = 32*5;
    template <unsigned int Size> void Marshall(Marshaller <Size> &m) {
        /* <<--marsh-->> */
        m &mac_n;
        m &mac_vec;
        m &mac_len;

        m &kalman_n;
        m &kalman_mat_dim;
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

        this->kalman_n = 16;
        this->kalman_mat_dim = 4;
    }

    conf_info_t(
        /* <<--ctor-args-->> */
        int32_t mac_n, 
        int32_t mac_vec, 
        int32_t mac_len,

        int32_t kalman_n,
        int32_t kalman_mat_dim

        )
    {
        /* <<--ctor-custom-->> */
        this->mac_n = mac_n;
        this->mac_vec = mac_vec;
        this->mac_len = mac_len;

        this->kalman_n = kalman_n;
        this->kalman_mat_dim = kalman_mat_dim;
    }

    // VCD dumping function
   inline friend void sc_trace(sc_trace_file *tf, const conf_info_t &v, const std::string &NAME)
    {
        /* <<--sctrc-->> */
        sc_trace(tf,v.mac_n, NAME + ".mac_n");
        sc_trace(tf,v.mac_vec, NAME + ".mac_vec");
        sc_trace(tf,v.mac_len, NAME + ".mac_len");

        sc_trace(tf,v.kalman_n, NAME + ".kalman_n");
        sc_trace(tf,v.kalman_mat_dim, NAME + ".kalman_mat_dim");

        // sc_trace(tf,v.mac_n,  NAME + ".mac_n");
        // sc_trace(tf,v.mac_length,  NAME + ".mac_length");
        // sc_trace(tf,v.mac_vec,  NAME + ".mac_vec");
    }

    // redirection operator
    friend ostream& operator << (ostream& os, conf_info_t const &conf_info)
    {
        os << "{";
        /* <<--print-->> */
        os << "mac_n = " << conf_info.mac_n << ", ";
        os << "mac_vec = " << conf_info.mac_vec << ", ";
        os << "mac_len = " << conf_info.mac_len << "";

        os << "kalman_n = " << conf_info.kalman_n << "";
        os << "kalman_mat_dim = " << conf_info.kalman_mat_dim << "";

        // os << "mac_n = " << conf_info.mac_n << ", ";
        // os << "mac_length = " << conf_info.mac_length << ", ";
        // os << "mac_vec = " << conf_info.mac_vec << ", ";
        os << "}";
        return os;
    }

};

#endif // __MAC_CONF_INFO_HPP__
