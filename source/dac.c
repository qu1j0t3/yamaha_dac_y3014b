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

// Copyright (C) 2018-2022 Toby Thain <toby@telegraphics.com.au>
// Code for Freedom KE06Z board (converted to 5V operation)

#include <math.h>

#define M_PI        3.14159265358979323846264338327950288   /* pi             */
#define M_PI_2      1.57079632679489661923132169163975144   /* pi/2           */
#define M_PI_4      0.785398163397448309615660845819875721  /* pi/4           */

void reset_delay() {
    for (uint32_t i = 0; i < 100; ++i) { //  250 => ~81 µs ; 500 => ~163µs
        __asm("NOP"); /* delay */
    }
}

void delay(unsigned j) {
    for (uint32_t i = 0; i < j; ++i) { //  500 => ~163µs
        __asm("NOP"); /* delay */
    }
}

void nine_microsecond() { // Tuned for Release configuration only!
    for (uint32_t i = 0; i < 30; ++i) { //  500 => ~163µs
        __asm("NOP"); /* delay */
    }
}

#define _WR 0x8000 // pin F7 in GPIOB

#define CH_A 0
#define CH_B 1
#define CH_C 2
#define CH_D 3

#if 0
void setDAC(unsigned channel, uint8_t x) {
    unsigned portF = channel << 10;
    GPIOB->PDOR = portF | _WR | LOAD_MASK;  // set up channel address
    GPIO_PortSet(kGPIO_PORTF, _WR);       // latch address
    /*
     * All these pins are on header J2.
    GPIOA: | D7  6  5  4  3  2  1 D0 | C7  6  5  4  3  2  1 C0 | B7  6  5  4  3  2  1 B0 | A7  6  5  4  3  2  1 A0 |
    DAC:   | -- -- -- -- -- -- -- -- | -- -- -- -- -- -- D1 -- | -- -- D5 -- D3 D2 -- -- | D7 D6 -- -- -- -- -- D0 |
    GPIOB: | H7  6  5  4  3  2  1 H0 | G7  6  5  4  3  2  1 G0 | F7  6  5  4  3  2  1 F0 | E7  6  5  4  3  2  1 E0 |
    DAC:   | -- -- -- -- -- -- -- D4 | -- -- -- -- -- -- -- -- |_WR -- -- -- A1 A0 -- -- | -- -- -- -- -- -- -- -- |
    */
    GPIOA->PDOR = (x << 16) | (x << 8) | x; // ports C, B, A
    GPIOB->PDOR = (x << 20) | portF | LOAD_MASK;        // ports H, F -- hold LOAD high to avoid latching the Yamaha DACs
    // wait min 45ns  (1 clock is 20.83ns)
    __asm("NOP");__asm("NOP");__asm("NOP");
    GPIO_PortClear(kGPIO_PORTF, _WR); // latch data
    // wait min 10ns
    __asm("NOP");
}
#endif

#define DAC_HALF 0x200 // 2^9, half of full scale 2^10
#define DAC_ZERO DAC_HALF

// Format exponent and mantissa for DAC serial input
// e must be 1..7 (output scale *2^-7 .. *2^-1 because Exponent is negated in the DAC)
// d is the fractional part:
// Vout = 1/2 Vdd + 1/4 Vdd (-1 + d*2^-9 + 2^-10) 2^-N
#define DAC_WORD(e, d) (((e) << 10) | (d))

#define N_POINTS 20         // multiple of 4

void setCoefficients(uint32_t v1, uint32_t v2, unsigned open_reset) {
    for(uint32_t j = 0; j < 14; ++j, v1 >>= 1, v2 >>= 1) {
    	// j = 0..9  D0..D9   mantissa
    	// j = 10..12                S0..S2   exponent
    	// j = 13 "extra" bit not clearly discussed in the datasheet but seems to be required for LOAD timing

    	BOARD_INITPINS_DAC_CLOCK_FGPIO->PCOR = BOARD_INITPINS_DAC_CLOCK_GPIO_PIN_MASK;

    	if (v1 & 1) {
    		BOARD_INITPINS_DAC_SD_X_COEFF_FGPIO->PSOR = BOARD_INITPINS_DAC_SD_X_COEFF_GPIO_PIN_MASK;
    	} else {
    		BOARD_INITPINS_DAC_SD_X_COEFF_FGPIO->PCOR = BOARD_INITPINS_DAC_SD_X_COEFF_GPIO_PIN_MASK;
    	}

    	if (v2 & 1) {
    		BOARD_INITPINS_DAC_SD_Y_COEFF_FGPIO->PSOR = BOARD_INITPINS_DAC_SD_Y_COEFF_GPIO_PIN_MASK;
    	} else {
    		BOARD_INITPINS_DAC_SD_Y_COEFF_FGPIO->PCOR = BOARD_INITPINS_DAC_SD_Y_COEFF_GPIO_PIN_MASK;
    	}

    	if (j == 5) {
    		if (open_reset) {
				BOARD_INITPINS_X_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
				BOARD_INITPINS_Y_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
    		}
    		BOARD_INITPINS_DAC_LOAD_FGPIO->PSOR = BOARD_INITPINS_DAC_LOAD_GPIO_PIN_MASK;
    	} else if (j == 13){
    		BOARD_INITPINS_DAC_LOAD_FGPIO->PCOR = BOARD_INITPINS_DAC_LOAD_GPIO_PIN_MASK;
    	}

    	// This is the tDS Data Setup period -- min 100ns per datasheet. Actually looks approx 600ns in Debug build

    	BOARD_INITPINS_DAC_CLOCK_FGPIO->PSOR = BOARD_INITPINS_DAC_CLOCK_GPIO_PIN_MASK;
        // Min clock high time 100ns per datasheet. Actually Clock remains high for just over 300ns
    }
}

/* Input range -1..+1 */
uint32_t dac_encode(double coeff) {
    // Find exponent scale for coefficient fraction
	unsigned e;
    double max = (DAC_HALF-1)/2.0;

    coeff *= (DAC_HALF-1);

    for(e = 7; e > 1 && fabs(coeff) < max; --e) {
    	coeff *= 2;
    }

    return DAC_WORD(e, (unsigned)(coeff + DAC_ZERO));
}

int main(void) {
    gpio_pin_config_t output = { kGPIO_DigitalOutput, 0 };

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
/*
    // TI DAC

    GPIO_PinInit(kGPIO_PORTA, 0, &output); // D0
    GPIO_PinInit(kGPIO_PORTC, 1, &output); // D1
    GPIO_PinInit(kGPIO_PORTB, 2, &output); // D2
    GPIO_PinInit(kGPIO_PORTB, 3, &output); // D3
    GPIO_PinInit(kGPIO_PORTH, 0, &output); // D4
    GPIO_PinInit(kGPIO_PORTB, 5, &output); // D5
    GPIO_PinInit(kGPIO_PORTA, 6, &output); // D6
    GPIO_PinInit(kGPIO_PORTA, 7, &output); // D7
    GPIO_PinInit(kGPIO_PORTF, 2, &output); // A0
    GPIO_PinInit(kGPIO_PORTF, 3, &output); // A1
    GPIO_PinInit(kGPIO_PORTF, 7, &output); // _WR

    setDAC(CH_A, 0x20);
    setDAC(CH_B, 0x50);
    setDAC(CH_C, 0x80);
    setDAC(CH_D, 0xb0);
*/
    uint32_t sintab[N_POINTS], costab[N_POINTS];
    double k = 2*M_PI/N_POINTS;
    for(int i = 0; i < N_POINTS; ++i) {
        sintab[i] = dac_encode(sin(i*k));
        costab[i] = dac_encode(cos(i*k));
    }


	BOARD_INITPINS_X_INT_HOLD_GPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK;
	BOARD_INITPINS_X_INT_RESET_GPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK;

    #if 0 // Test sine wave and hold switch
    for(uint32_t cycle = 0; ; ++cycle) {

		BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger
		for (uint32_t i = 0; i < N_POINTS; ++i) {
			BOARD_INITPINS_TRIGGER_GPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger
            // Setting C2 and C3 will put the integrators into reset,
            // the capacitor will be discharged (according to the time constant).
            // Having D6 and D7 clear disconnects the DAC from the integrator input,
            // which makes the reset cleaner (closer to DAC zero).
            //GPIOA->PDOR = (!i << 16 /*C0 scope trigger*/) | (1 << 18 /*C2*/) | (1 << 19 /*C3*/); // reset

            //reset_delay();

            setCoefficients( sintab[i], sintab[(i + N_POINTS/4) % N_POINTS] );
            nine_microsecond(); nine_microsecond(); nine_microsecond();

            //GPIOA->PDOR = (1 << 30 /*D6*/) | (1 << 31 /*D7*/); // Release integrators from reset and hold

            //delay();
        }
    }
	#endif

    // Simple step test
    for(;0;) {
		BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger
		setCoefficients( DAC_WORD(7, 0), DAC_WORD(0, DAC_ZERO), 0 );
		BOARD_INITPINS_TRIGGER_GPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger
		nine_microsecond(); // settling time

		setCoefficients( DAC_WORD(7, 512), DAC_WORD(0, DAC_ZERO), 0 );
		nine_microsecond(); // settling time

	    // Set DAC to most negative
		setCoefficients( DAC_WORD(7, 1023), DAC_WORD(0, DAC_ZERO), 0 );
		nine_microsecond(); // settling time
    }


    // Ramp test integrator


	for(;0;) {
		setCoefficients( DAC_WORD(7, 1023), DAC_WORD(7, 0), 0 );


		BOARD_INITPINS_X_INT_RESET_GPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
		BOARD_INITPINS_Y_INT_RESET_GPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET

		BOARD_INITPINS_X_INT_HOLD_GPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch X
		BOARD_INITPINS_Y_INT_HOLD_GPIO->PSOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch Y


		// First ramp ---------------------------------------------


		BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger

		BOARD_INITPINS_Z_BLANK_GPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam ON

		nine_microsecond();
		nine_microsecond();

		BOARD_INITPINS_Z_BLANK_GPIO->PCOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam OFF

		BOARD_INITPINS_TRIGGER_GPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger

	    BOARD_INITPINS_X_INT_HOLD_GPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch X
	    BOARD_INITPINS_Y_INT_HOLD_GPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y

	    BOARD_INITPINS_X_INT_RESET_GPIO->PSOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
		BOARD_INITPINS_Y_INT_RESET_GPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch

	}

	// Starburst test

	for(;;) {
		for (uint32_t i = 0; i < N_POINTS; ++i) {
            setCoefficients( sintab[i], costab[i], 1 );
    		__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
    		__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
    		__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
    		__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
    		__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");

		    //delay(10); // Wait reset time

			BOARD_INITPINS_X_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
			BOARD_INITPINS_Y_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET

    		BOARD_INITPINS_X_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch X
    		BOARD_INITPINS_Y_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch Y

    		BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam ON
    		BOARD_INITPINS_TRIGGER_FGPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger

			// Wait integrating time
    		nine_microsecond();
    		nine_microsecond();

    		BOARD_INITPINS_Z_BLANK_FGPIO->PCOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam OFF
    		BOARD_INITPINS_TRIGGER_FGPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger


		    BOARD_INITPINS_X_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch X
		    BOARD_INITPINS_Y_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y

    		// As soon as beam is off, we can short the integrator
			BOARD_INITPINS_X_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
			BOARD_INITPINS_Y_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
        }
	}


}
