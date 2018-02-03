/*
 * The Clear BSD License
 * Copyright (c) 2017 , NXP
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_port.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PORT_PINNUMS_EACHPORT (8U)   /* PORT pin numbers in each PTA/PTB etc. */
#define PORT_NUM_EACH_PULLUPREG (4U) /* The port numbers in each pull up register. */
#define FSL_PORT_FILTER_SELECT_BITMASK  (0x3U) /* The filter selection bit width. */
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void PORT_SetPinSelect(port_module_t module, port_pin_select_t pin)
{
    uint32_t pinSelReg;
    uint32_t bitwidth = ((uint32_t)module >> PORT_MODULEPS_BITWIDTH_OFFSET) & PORT_MODULEPS_BITWIDTH_MASK;
    uint32_t shift = (uint32_t)module & ~((uint32_t)PORT_PINSEL_REG_OFFSET | ((uint32_t)PORT_MODULEPS_BITWIDTH_MASK << PORT_MODULEPS_BITWIDTH_OFFSET));

    if (module > kPORT_SWDE)
    {
        if (!((uint32_t)module & PORT_PINSEL_REG_OFFSET))
        {
            pinSelReg = SIM->PINSEL0 & ~ (uint32_t)((uint32_t)((1 << bitwidth) - 1) << shift);
            SIM->PINSEL0 = pinSelReg | (pin << shift);
        }
        else
        {
            pinSelReg = SIM->PINSEL1 & ~ (uint32_t)((uint32_t)((1 << bitwidth) - 1) << shift);
            SIM->PINSEL1 = pinSelReg | (pin << shift);
        }
    }
    else
    {
        SIM->SOPT0 = (SIM->SOPT0 & ~(uint32_t)(1 << module)) | ((uint32_t)pin << module);
    }
}

void PORT_SetFilterSelect(PORT_Type *base, port_filter_pin_t port, port_filter_select_t filter)
{
    uint32_t fltReg;

    if (port < PORT_FILTER_REG_OFFSET)
    {
        /* Clear the filed to zero. */
        fltReg = base->IOFLT0 & ~(FSL_PORT_FILTER_SELECT_BITMASK << port);
        /* Set the filter selection. */
        base->IOFLT0 = (filter << port) | fltReg;        
    }
    else
    {
        port -= PORT_FILTER_REG_OFFSET;
        /* Clear the filed to zero. */
        fltReg = base->IOFLT1 & ~(FSL_PORT_FILTER_SELECT_BITMASK << port);
        /* Set the filter selection. */
        base->IOFLT1 = (filter << port) | fltReg;         
    }

}

void PORT_SetPinPullUpEnable(PORT_Type *base, port_type_t port, port_pin_index_t num, bool enable)
{
    if (enable)
    {
        /* Enable the pull up */
        if (port < kPORT_PTE)
        {
            base->PUE0 |= (1U << (PORT_PINNUMS_EACHPORT * port + num));
        }
        else if (port < kPORT_PTI)
        {
            base->PUE1 |= (1U << (PORT_PINNUMS_EACHPORT * (port - PORT_NUM_EACH_PULLUPREG) + num));
        }
        else
        {
            base->PUE2 |= (1U << (PORT_PINNUMS_EACHPORT * (port - 2 * PORT_NUM_EACH_PULLUPREG) + num));
        }
    }
    else
    {
        /* Disable the pull up */
        if (port < kPORT_PTE)
        {
            base->PUE0 &= ~(1U << (PORT_PINNUMS_EACHPORT * port + num));
        }
        else if (port < kPORT_PTI)
        {
            base->PUE1 &= ~(1U << (PORT_PINNUMS_EACHPORT * (port - PORT_NUM_EACH_PULLUPREG) + num));
        }
        else
        {
            base->PUE2 &= ~(1U << (PORT_PINNUMS_EACHPORT * (port - 2 * PORT_NUM_EACH_PULLUPREG) + num));            
        }
    }
}
