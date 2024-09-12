// Copyright (c) 2011-2023 Columbia University, System Level Design Group
// SPDX-License-Identifier: Apache-2.0

#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include "mac_2.hpp"

void mac_2_sysc_catapult::load_d(bool ping, uint32_t base_addr, uint32_t size)
{
    uint32_t index = 0;
    uint32_t mem_index = 0;
    uint32_t mem_off = base_addr;

    for (index = size; index > 0; )
    {

        uint32_t j = 0;
        uint32_t beats = (index < 16) ? index : 16;
        uint32_t len=beats;

        dma_info_t dma_info(mem_off, len, DMA_SIZE);
        dma_read_ctrl.Push(dma_info);

        const uint32_t data_mask = (((uint32_t) 1) << FPDATA_WL) - 1;

        for (; j < beats; ++j)
        {

            DMA_WORD r=dma_read_chnl.Pop();

            plm_WR<in_as,inwp> wreq;
            wreq.indx[0]=mem_index++;
            wreq.data[0]=data_mask & r;

            if (ping)
                in_ping_w.Push(wreq);
            else
                in_pong_w.Push(wreq);

        }

        mem_off += beats;
        // mem_off += (beats << 2);
        index -= beats;

        wait();
    }
}

void mac_2_sysc_catapult::load_b(bool ping, uint32_t base_addr, uint32_t size)
{
    uint32_t index = 0;
    uint32_t mem_index = 0;
    uint32_t mem_off = base_addr;

    for (index = size; index > 0; )
    {

        uint32_t j = 0;
        uint32_t beats = (index < 16) ? index : 16;
        uint32_t len=beats;

        dma_info_t dma_info(mem_off, len, DMA_SIZE);
        dma_read_ctrl.Push(dma_info);

        const uint32_t data_mask = (((uint32_t) 1) << FPDATA_WL) - 1;

        for (; j < beats; ++j)
        {

            DMA_WORD r=dma_read_chnl.Pop();

            plm_WR<in_as,inwp> wreq;
            wreq.indx[0]=mem_index++;
            wreq.data[0]=data_mask & r;

            if (ping)
                in_b_ping_w.Push(wreq);
            else
                in_b_pong_w.Push(wreq);

        }

        mem_off += beats;
        // mem_off += (beats << 2);
        index -= beats;

        wait();
    }
}


void mac_2_sysc_catapult::store_data_req(bool ping, uint32_t base_addr, uint32_t size)
{
    uint32_t index = 0;
    uint32_t mem_index = 0;
    uint32_t mem_off = base_addr;
    for (index = size; index > 0; )
    {
        uint32_t j = 0;
        uint32_t beats = (index < 16) ? index : 16;

        for (; j < beats; ++j)
        {
            plm_RRq<out_as,outrp> rreq;
            rreq.indx[0]=mem_index;
            if (ping)
                out_ping_ra.Push(rreq);
            else
                out_pong_ra.Push(rreq);
            mem_index++;
        }
        mem_off += beats ;
        index -= beats;
        wait();
    }
}

void mac_2_sysc_catapult::store_data(bool ping, uint32_t base_addr, uint32_t size)
{
    uint32_t index = 0;
    uint32_t mem_off = base_addr;
    for (index = size; index > 0; )
    {

        uint32_t j = 0;
        uint32_t beats = (index < 16) ? index : 16;
        uint32_t len=beats;

        dma_info_t dma_info(mem_off, len, DMA_SIZE);
        dma_write_ctrl.Push(dma_info);

        #pragma hls_pipeline_init_interval 1
        #pragma pipeline_stallt_mode flush
        for (; j < beats; ++j)
        {
            FPDATA_WORD w;

            if (ping)
                w=out_ping_rd.Pop().data[0];
            else
                w=out_pong_rd.Pop().data[0];

            dma_write_chnl.Push(w);

            wait();
        }

        // mem_off += (beats << 2);
        mem_off += beats ;
        index -= beats;

        wait();
    }
}


#endif // __UTILS_HPP__
