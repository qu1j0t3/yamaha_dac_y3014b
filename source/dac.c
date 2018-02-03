/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
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
 * o Neither the name of the copyright holder nor the names of its
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

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

#include "pin_mux.h"
#include "clock_config.h"

// Copyright (C) 2018 Toby Thain <toby@telegraphics.com.au>
// Code for Freedom KE06Z board (converted to 5V operation)

// For breadboard layout, see https://www.flickr.com/photos/qu1j0t3/25194819157/in/dateposted/

#define LOAD_MASK  (1u << 4)
#define CLOCK_MASK (1u << 5)
#define SD_MASK    (1u << 6)

#define N 7 // gives a ramp from 1.26 .. 3.70 V (2.44 p-p); buffering with LM2904 produces clipping ~ 3.62V, if powered at 5V
//#define N 6 // 1.78 .. 3.08V (1.3 p-p)
//#define N 5 // 2.1 .. 2.78V (.68 p-p)
// etc

int main(void)
{
    gpio_pin_config_t output = { kGPIO_DigitalOutput, 0 };

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    GPIO_PinInit(kGPIO_PORTE, 4, &output); // LOAD
    GPIO_PinInit(kGPIO_PORTE, 5, &output); // CLOCK
    GPIO_PinInit(kGPIO_PORTE, 6, &output); // SD

    GPIOB->PDOR = 0;
    for (uint32_t x = 0; ; ++x) {
    		// 16 bits are delivered serially to DAC
    		// 3 "invalid", 10 mantissa (D0 .. D9), 3 exponent (S0 .. S2)
    		// Vout = 1/2 Vdd + 1/4 Vdd (-1 + D9 + D8 2^-1 + ... D0 2^-9 + 2^-10) 2^(N-7)
    		// N = S2 2^2 + S1 2^1 + S0    where S2=S1=S0=0 is not allowed
    		// Note error in the datasheet relating to the exponent, the scale is 2^(N-7) not 2^-N

    		uint32_t v = (((N << 10) | (x & 0x3ff)) << 3) << 6; // shifted so LSB coincides with SD_MASK
    		for(uint32_t j = 0; j < 16; ++j, v >>= 1) {
    			// this loop cycles at about 119kHz in Release level,
    			// i.e. 16 bits are clocked out in about 8.42µs
    			GPIOB->PDOR = ((j & 8) << 1) | (v & SD_MASK);

    			// in Release level, there is about 200ns between the LOAD transition
    			// and the rising CLOCK pulse
			GPIOB->PSOR = CLOCK_MASK; // 200ns pulse in Release level
			GPIOB->PCOR = CLOCK_MASK;
    		}
    }
}


/* this loop manages ~243 kHz in Debug level, ~713 kHz in Release
while (1) {
    GPIO_PortToggle(kGPIO_PORTE, 1u << 5);
}*/
