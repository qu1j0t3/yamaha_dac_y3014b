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

#define N 7 // gives a ramp from 1.26 .. 3.70 V (2.44 p-p); buffering with LM2904 produces clipping ~ 3.62V, if powered at 5V
//#define N 6 // 1.78 .. 3.08V (1.3 p-p)
//#define N 5 // 2.1 .. 2.78V (.68 p-p)
// etc

void reset_delay() {
    for (uint32_t i = 0; i < 20; ++i) { //  250 => ~81 µs ; 500 => ~163µs
        __asm("NOP"); /* delay */
    }
}

// this delay established by trial and error to avoid integrator clipping
void delay() {
    for (uint32_t i = 0; i < 6300; ++i) { //  500 => ~163µs
        __asm("NOP"); /* delay */
    }
}

void four_microsecond() { // Tuned for Release configuration only!
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
// e must be 1..7 (output scale *2^-7 .. *2^-1 because Exponent is negative in the DAC)
// d is the fractional part:
// Vout = 1/2 Vdd + 1/4 Vdd (-1 + d*2^-9 + 2^-10) 2^-N
// i.e. where d = 0    Vout = 1/2 Vdd - 1/4 Vdd * 2^-N = 3/8 Vdd where N = 1 (e=7)
#define DAC_WORD(e, d) (((e) << 10) | (d))

#define N_POINTS 64         // multiple of 4

void setCoefficients(uint32_t v1, uint32_t v2) {
	v1 <<= 2; v2 <<= 2;
    for(uint32_t j = 0; j < 16; ++j, v1 >>= 1, v2 >>= 1) {
    	// j = 0,1,2                   filler bits, ignored by DAC.
    	// j = 3,4,5,6,7,8,9,10,11,12  D0..D9   mantissa
    	// j = 13,14,15                S0..S2   exponent

    	BOARD_INITPINS_DAC_CLOCK_GPIO->PCOR = BOARD_INITPINS_DAC_CLOCK_GPIO_PIN_MASK;

    	if (v1 & 1) {
    		BOARD_INITPINS_DAC_SD_X_COEFF_GPIO->PSOR = BOARD_INITPINS_DAC_SD_X_COEFF_GPIO_PIN_MASK;
    	} else {
    		BOARD_INITPINS_DAC_SD_X_COEFF_GPIO->PCOR = BOARD_INITPINS_DAC_SD_X_COEFF_GPIO_PIN_MASK;
    	}

    	if (v2 & 1) {
    		BOARD_INITPINS_DAC_SD_Y_COEFF_GPIO->PSOR = BOARD_INITPINS_DAC_SD_Y_COEFF_GPIO_PIN_MASK;
    	} else {
    		BOARD_INITPINS_DAC_SD_Y_COEFF_GPIO->PCOR = BOARD_INITPINS_DAC_SD_Y_COEFF_GPIO_PIN_MASK;
    	}

    	if (j == 7) {
    		BOARD_INITPINS_DAC_LOAD_GPIO->PSOR = BOARD_INITPINS_DAC_LOAD_GPIO_PIN_MASK;
    	} else if (j == 15){
    		BOARD_INITPINS_DAC_LOAD_GPIO->PCOR = BOARD_INITPINS_DAC_LOAD_GPIO_PIN_MASK;
    	}

    	// This is the tDS Data Setup period -- min 100ns per datasheet. Actually looks approx 600ns in Debug build

    	BOARD_INITPINS_DAC_CLOCK_GPIO->PSOR = BOARD_INITPINS_DAC_CLOCK_GPIO_PIN_MASK;
        // Min clock high time 100ns per datasheet. Actually Clock remains high for just over 300ns
    }

    // Wait 4µs max settling time!
    // (this was tuned by bracketing setting/resetting pin C0 and measuring pulse on scope)
    //four_microsecond();
}

int main(void) {
    gpio_pin_config_t output = { kGPIO_DigitalOutput, 0 };

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
/*
    // GPIOA - D/C/B/A
    GPIO_PinInit(kGPIO_PORTC, 0, &output); // Utility/trigger pin
    GPIO_PinInit(kGPIO_PORTC, 2, &output); // INTEGRATOR 1 RESET
    GPIO_PinInit(kGPIO_PORTC, 3, &output); // INTEGRATOR 2 RESET
    GPIO_PinInit(kGPIO_PORTD, 6, &output); // INTEGRATOR 1 HOLD
    GPIO_PinInit(kGPIO_PORTD, 7, &output); // INTEGRATOR 2 HOLD

    // GPIOB - H/G/F/E
    GPIO_PinInit(kGPIO_PORTE, 4, &output); // LOAD
    GPIO_PinInit(kGPIO_PORTE, 5, &output); // CLOCK
    GPIO_PinInit(kGPIO_PORTE, 6, &output); // SD (DAC 1)
    GPIO_PinInit(kGPIO_PORTH, 6, &output); // SD (DAC 2)

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

    GPIOB->PDOR = LOAD_MASK;
    GPIOA->PDOR = 0;

    setDAC(CH_A, 0x20);
    setDAC(CH_B, 0x50);
    setDAC(CH_C, 0x80);
    setDAC(CH_D, 0xb0);
*/
    uint32_t sintab[N_POINTS];
    double k = 2*M_PI/N_POINTS;
    for(int i = 0; i < N_POINTS; ++i) {
        double x = (DAC_HALF-20)*sin(i*k);
        unsigned e = 7;
        double max = (DAC_HALF-1)/2.0;
        while(e > 1 && abs(x) < max) {
        	x *= 2;
        	--e;
        }

        sintab[i] = (uint32_t)DAC_WORD(e, (unsigned)(x + DAC_ZERO));
    }


	BOARD_INITPINS_X_INT_HOLD_GPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK;
	BOARD_INITPINS_X_INT_RESET_GPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK;
/*
	for(;;) {
		setCoefficients( DAC_EXPT(N) | (DAC_ZERO + (DAC_HALF-1)), DAC_EXPT(N) | (DAC_ZERO + (DAC_HALF-1)));
	    for (uint32_t i = 0; i < 4000000; ++i) {
	        __asm("NOP");
	    }
		setCoefficients( DAC_EXPT(N) | DAC_ZERO, DAC_EXPT(N) | DAC_ZERO);
	    for (uint32_t i = 0; i < 4000000; ++i) {
	        __asm("NOP");
	    }
		setCoefficients( DAC_EXPT(N) | (DAC_ZERO - (DAC_HALF-1)), DAC_EXPT(N) | (DAC_ZERO - (DAC_HALF-1)));
	    for (uint32_t i = 0; i < 4000000; ++i) {
	        __asm("NOP");
	    }
	}*/

    #if 0 // Test sine wave and hold switch
    for(uint32_t cycle = 0; ; ++cycle) {

		for (uint32_t i = 0; i < N_POINTS; ++i) {
            // Setting C2 and C3 will put the integrators into reset,
            // the capacitor will be discharged (according to the time constant).
            // Having D6 and D7 clear disconnects the DAC from the integrator input,
            // which makes the reset cleaner (closer to DAC zero).
            //GPIOA->PDOR = (!i << 16 /*C0 scope trigger*/) | (1 << 18 /*C2*/) | (1 << 19 /*C3*/); // reset

            //reset_delay();

            setCoefficients( sintab[i], sintab[(i + N_POINTS/4) % N_POINTS] );
            four_microsecond(); four_microsecond(); four_microsecond();

            //GPIOA->PDOR = (1 << 30 /*D6*/) | (1 << 31 /*D7*/); // Release integrators from reset and hold

            //delay();
        }
    }
	#endif

    // Test integrator

    // Notes: 10nF integrating capacitor, ≈125Ω switch R = tc 1.25µs ; 5 tc = 6.25µs to reset integrator

	for(;;) {
	    BOARD_INITPINS_X_INT_HOLD_GPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch



		BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger
		setCoefficients( DAC_WORD(7, 0), DAC_WORD(0, DAC_ZERO) );
		BOARD_INITPINS_TRIGGER_GPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger
		four_microsecond(); // settling time

		setCoefficients( DAC_WORD(7, 512), DAC_WORD(0, DAC_ZERO) );
		four_microsecond(); // settling time

	    // Set DAC to most negative
		setCoefficients( DAC_WORD(7, 1023), DAC_WORD(0, DAC_ZERO) );
		four_microsecond(); // settling time


/*
		BOARD_INITPINS_X_INT_HOLD_GPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch

		BOARD_INITPINS_X_INT_RESET_GPIO->PSOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
		reset_delay(); // Wait reset time
		reset_delay(); // Wait reset time
		BOARD_INITPINS_X_INT_RESET_GPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET


		// First ramp ---------------------------------------------

	    BOARD_INITPINS_X_INT_HOLD_GPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch

	    // Set DAC to most negative
		setCoefficients( DAC_EXPT(N) | (DAC_ZERO - (DAC_HALF-1)), DAC_EXPT(N) | DAC_ZERO);
		four_microsecond(); // settling time

		BOARD_INITPINS_X_INT_HOLD_GPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch

		// Wait integrating time
		// With 1.25 Vin (max DAC delta), C=10nF, R=1k, then 8µs integration time = 1V change
		four_microsecond();
		four_microsecond();


		// Second ramp ---------------------------------------------

		BOARD_INITPINS_X_INT_HOLD_GPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch

		// Set DAC to most positive
		setCoefficients( DAC_EXPT(N) | (DAC_ZERO + (DAC_HALF-1)), DAC_EXPT(N) | DAC_ZERO);
		four_microsecond(); // settling time

		BOARD_INITPINS_X_INT_HOLD_GPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch

		// Wait integrating time
		four_microsecond();
		four_microsecond();*/
	}


}
