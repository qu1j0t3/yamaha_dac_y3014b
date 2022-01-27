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

#define DEG2RAD(d) (2.0*M_PI*(d)/360.0)

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

void four_microseconds() { // Tuned for Release configuration only!
    for (uint32_t i = 0; i < 30; ++i) { //  500 => ~163µs
        __asm("NOP"); /* delay */
    }
}


#define DAC_HALF 0x200 // 2^9, half of full scale 2^10
#define DAC_ZERO DAC_HALF

// Format exponent and mantissa for DAC serial input
// e must be 1..7 (output scale *2^-7 .. *2^-1 because Exponent is negated in the DAC)
// d is the fractional part:
// Vout = 1/2 Vdd + 1/4 Vdd (-1 + d*2^-9 + 2^-10) 2^-N
#define DAC_WORD(e, d) (((e) << 10) | (d))



#define N_POINTS 15

double px[N_POINTS]={
  0,
  -2.558,
  -.953,
  -31.627,
  -28.557,
  -58.59,
  -50.975,
  -56.129,
  -37.608,
  -34.776,
  -19.754,
  -16.425,
  -22.965,
  -11.744,
  0
};

double py[N_POINTS]={
  0,
  -63.282,
  -30.632,
  -35.183,
  -25.058,
  -.409,
  3.192,
  23.773,
  20.088,
  29.424,
  12.848,
  14.423,
  47.308,
  41.31,
  63.282
};

uint32_t xcoeff[N_POINTS*2+2];
uint32_t ycoeff[N_POINTS*2+2];
uint32_t length[N_POINTS*2+2];

double wrapx(unsigned i) {
  return i < N_POINTS ? px[i] : -px[N_POINTS + N_POINTS - i - 2];
}
double wrapy(unsigned i) {
  return py[i < N_POINTS ? i : N_POINTS + N_POINTS - i - 2];
}



#define SINCOS_POINTS 16         // multiple of 4

void setCoefficients(uint32_t v1, uint32_t v2) {
    for(uint32_t j = 0; j < 14; ++j, v1 >>= 1, v2 >>= 1) {
    	// j = 0..9  D0..D9   mantissa
    	// j = 10..12                S0..S2   exponent
    	// j = 13 "extra" bit not clearly discussed in the datasheet but seems to be required for LOAD timing

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
    	if (j == 5) {
    		BOARD_INITPINS_DAC_LOAD_GPIO->PSOR = BOARD_INITPINS_DAC_LOAD_GPIO_PIN_MASK;
    	} else if (j == 13){
    		BOARD_INITPINS_DAC_LOAD_GPIO->PCOR = BOARD_INITPINS_DAC_LOAD_GPIO_PIN_MASK;
    	}

    	// This is the tDS Data Setup period -- min 100ns per datasheet. Actually looks approx 600ns in Debug build

    	BOARD_INITPINS_DAC_CLOCK_GPIO->PSOR = BOARD_INITPINS_DAC_CLOCK_GPIO_PIN_MASK;
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

// chip select
#define DAC_LIMIT  0
#define DAC_POS    1
// units
#define DAC_A      0
#define DAC_B      (1u << 15)
// control
#define DAC_BUFFERED (1u << 14)
#define DAC_GAINx2   0
#define DAC_GAINx1   (1u << 13)
#define DAC_ACTIVE   (1u << 12)

void spi(unsigned cs, unsigned unit, unsigned gain, uint16_t value) {
	// Write Command Register for MCP4922 (12-bit DAC)
	// 15   _A/B : 1 = DAC B,    0 = DAC A
	// 14   BUF  : 1 = Buffered, 0 = Unbuffered
	// 13   _GA  : 1 = 1x,       0 = 2x
	// 12  _SHDN : 1 = Active,   0 = Shutdown selected DAC channel
	// 11..0     : data

	// Select chip
	if (cs) {
		BOARD_INITPINS_NOTCS_DAC_1_GPIO->PCOR = BOARD_INITPINS_NOTCS_DAC_1_GPIO_PIN_MASK;
	} else {
		BOARD_INITPINS_NOTCS_DAC_0_GPIO->PCOR = BOARD_INITPINS_NOTCS_DAC_0_GPIO_PIN_MASK;
	}

	unsigned word = unit | DAC_BUFFERED | gain | DAC_ACTIVE | value;

	// Bit-bang SPI
	for (unsigned i = 16; i--;) {
		if ((word >> i) & 1) {
			BOARD_INITPINS_SPI_DATAOUT_GPIO->PSOR = BOARD_INITPINS_SPI_DATAOUT_GPIO_PIN_MASK;
		} else {
			BOARD_INITPINS_SPI_DATAOUT_GPIO->PCOR = BOARD_INITPINS_SPI_DATAOUT_GPIO_PIN_MASK;
		}
		BOARD_INITPINS_SPI_CLOCK_GPIO->PSOR = BOARD_INITPINS_SPI_CLOCK_GPIO_PIN_MASK;
		BOARD_INITPINS_SPI_CLOCK_GPIO->PCOR = BOARD_INITPINS_SPI_CLOCK_GPIO_PIN_MASK;
	}

	// Deselect chip (and also latch DAC data when NOT_LDAC is tied low)
	if (cs) {
		BOARD_INITPINS_NOTCS_DAC_1_GPIO->PSOR = BOARD_INITPINS_NOTCS_DAC_1_GPIO_PIN_MASK;
	} else {
		BOARD_INITPINS_NOTCS_DAC_0_GPIO->PSOR = BOARD_INITPINS_NOTCS_DAC_0_GPIO_PIN_MASK;
	}

	// Without this delay, making an immediate next call to set unit B will fail (DAC won't latch)
	__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");

	// Note that DAC takes approx 4.5µs to slew 2.5V
}

int main(void) {
    /* Init the boards */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    uint32_t sintab[SINCOS_POINTS], costab[SINCOS_POINTS];
    double k = 2*M_PI/SINCOS_POINTS;
    for(int i = 0; i < SINCOS_POINTS; ++i) {
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
            four_microseconds(); four_microseconds(); four_microseconds();

            //GPIOA->PDOR = (1 << 30 /*D6*/) | (1 << 31 /*D7*/); // Release integrators from reset and hold

            //delay();
        }
    }
	#endif

    // Test SPI DAC MCP4922

    for(;0;) {
		BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger

		spi(0, DAC_A, DAC_GAINx1, 0xfffu);
		spi(0, DAC_B, DAC_GAINx1, 0xfffu);
		spi(1, DAC_A, DAC_GAINx1, 0xfffu);
		spi(1, DAC_B, DAC_GAINx1, 0xfffu);

		BOARD_INITPINS_TRIGGER_GPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger

		four_microseconds();

		spi(0, DAC_A, DAC_GAINx1, 0x800u);
		spi(0, DAC_B, DAC_GAINx1, 0x800u);
		spi(1, DAC_A, DAC_GAINx1, 0x800u);
		spi(1, DAC_B, DAC_GAINx1, 0x800u);

		four_microseconds();

		spi(0, DAC_A, DAC_GAINx1, 0);
		spi(0, DAC_B, DAC_GAINx1, 0);
		spi(1, DAC_A, DAC_GAINx1, 0);
		spi(1, DAC_B, DAC_GAINx1, 0);

		four_microseconds();
    }

    // Simple step test
    for(;0;) {
		BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger
		setCoefficients( DAC_WORD(7, 0), DAC_WORD(7, 0) );
		BOARD_INITPINS_TRIGGER_GPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger
		four_microseconds(); // settling time

		setCoefficients( DAC_WORD(7, 512), DAC_WORD(7, 512) );
		four_microseconds(); // settling time

	    // Set DAC to most negative
		setCoefficients( DAC_WORD(7, 1023), DAC_WORD(7, 1023) );
		four_microseconds(); // settling time
    }


    // Ramp test integrator

	for(;0;) {
		setCoefficients( DAC_WORD(7, 1023), DAC_WORD(7, 0) );


		BOARD_INITPINS_X_INT_RESET_GPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
		BOARD_INITPINS_Y_INT_RESET_GPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET

		BOARD_INITPINS_X_INT_HOLD_GPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch X
		BOARD_INITPINS_Y_INT_HOLD_GPIO->PSOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch Y


		// First ramp ---------------------------------------------


		BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger

		BOARD_INITPINS_Z_BLANK_GPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam ON

		four_microseconds();
		four_microseconds();

		BOARD_INITPINS_Z_BLANK_GPIO->PCOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam OFF

		BOARD_INITPINS_TRIGGER_GPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger

	    BOARD_INITPINS_X_INT_HOLD_GPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch X
	    BOARD_INITPINS_Y_INT_HOLD_GPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y

	    BOARD_INITPINS_X_INT_RESET_GPIO->PSOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
		BOARD_INITPINS_Y_INT_RESET_GPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
	}


	// Positioning box test

	if(0) {
		BOARD_INITPINS_X_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
		BOARD_INITPINS_Y_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch

		BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam ON
		for(unsigned i = 0; 1; ++i) {

			if((i & 3) == 0) BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger
			spi(DAC_POS, DAC_A, DAC_GAINx1, i & 2 ? 0xfff : 0);
			spi(DAC_POS, DAC_B, DAC_GAINx1, (i+1) & 2 ? 0xfff : 0);

			BOARD_INITPINS_TRIGGER_FGPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger
			four_microseconds();
		}
	}

	// Comparator test

	// Set threshold voltage (in actual use, only one threshold needs to be set - for the fastest changing axis)
	spi(DAC_LIMIT, DAC_A, DAC_GAINx2, (uint16_t)((3.0 / 5.0) * 0xfff));
	spi(DAC_LIMIT, DAC_B, DAC_GAINx2, (uint16_t)((3.0 / 5.0) * 0xfff));

	// Arm X comparator
	BOARD_INITPINS_X_COMP_SEL_GPIO->PSOR = BOARD_INITPINS_X_COMP_SEL_GPIO_PIN_MASK;
	BOARD_INITPINS_Y_COMP_SEL_GPIO->PCOR = BOARD_INITPINS_Y_COMP_SEL_GPIO_PIN_MASK;

	// A realistic pair of coefficients. X will change faster than Y (angle < 45°)
	uint32_t xc = dac_encode(-cos(DEG2RAD(20.0)));
	uint32_t yc = dac_encode(-sin(DEG2RAD(20.0)));

    setCoefficients( xc, yc );

    for(;1;) {
		BOARD_INITPINS_TRIGGER_FGPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK;

	    BOARD_INITPINS_X_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch X
	    BOARD_INITPINS_Y_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y

		BOARD_INITPINS_X_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
		BOARD_INITPINS_Y_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch

		delay(120); // Wait reset time

		BOARD_INITPINS_TRIGGER_FGPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger

		BOARD_INITPINS_X_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
		BOARD_INITPINS_Y_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET

		BOARD_INITPINS_X_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch X
		BOARD_INITPINS_Y_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch Y


		// Wait integrating time

		while(! (BOARD_INITPINS_STOP_GPIO->PDIR & BOARD_INITPINS_STOP_GPIO_PIN_MASK)) ;
	}


	// Starburst test

	for(;1;) {
		for (unsigned i = 0; i < SINCOS_POINTS; ++i) {
            setCoefficients( sintab[i], costab[i] );

            // Position DAC has 2.5V range
    		spi(DAC_POS, DAC_A, DAC_GAINx1, i & 1 ? 0x600 : 0); // 0.9375V
    		spi(DAC_POS, DAC_B, DAC_GAINx1, i & 2 ? 0x400 : 0); // 0.625V

		    delay(120); // Wait reset time
    		BOARD_INITPINS_TRIGGER_FGPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger

			BOARD_INITPINS_X_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
			BOARD_INITPINS_Y_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET

    		BOARD_INITPINS_X_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch X
    		BOARD_INITPINS_Y_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch Y

    		BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam ON

			// Wait integrating time
    		four_microseconds();
    		four_microseconds();
    		four_microseconds();
    		four_microseconds();
    		four_microseconds();
    		four_microseconds();

    		BOARD_INITPINS_Z_BLANK_FGPIO->PCOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam OFF

		    BOARD_INITPINS_X_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch X
		    BOARD_INITPINS_Y_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y

		    if(i == SINCOS_POINTS/4) BOARD_INITPINS_TRIGGER_FGPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger

    		// As soon as beam is off, we can short the integrator
		    // Based on measurements, reset takes about 14µs
			BOARD_INITPINS_X_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
			BOARD_INITPINS_Y_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
        }
	}

	// FLAG test

	uint32_t t = 0;
	  for(unsigned i = 0; i < 2*(N_POINTS-1); ++i) {
	    double dx = wrapx(i+1) - wrapx(i);
	    double dy = wrapy(i+1) - wrapy(i);
	    double d = sqrt(dx*dx + dy*dy);
	    length[i] = (uint32_t)( d*.7 );
	    t += length[i];
	    xcoeff[i] = dac_encode(dx/d);//0x80 + 126*dx/d;
	    ycoeff[i] = dac_encode(dy/d);//0x80 - 126*dy/d;
	  }

	  for(unsigned i = 0;;) {
		  if(i==0){
			BOARD_INITPINS_TRIGGER_FGPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger
		  }
	    setCoefficients( xcoeff[i], ycoeff[i] );
  		BOARD_INITPINS_TRIGGER_FGPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger
	    delay(10);

			BOARD_INITPINS_X_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
			BOARD_INITPINS_Y_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET

		BOARD_INITPINS_X_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch X
		BOARD_INITPINS_Y_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch Y

			BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam ON

			// Wait integrating time
		delay(length[i]);

      		BOARD_INITPINS_Z_BLANK_FGPIO->PCOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam OFF

		BOARD_INITPINS_X_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch X
		BOARD_INITPINS_Y_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y


          ++i;
          if(i == 2*(N_POINTS-1)) {

  			BOARD_INITPINS_X_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
  			BOARD_INITPINS_Y_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch

  			delay(40);

  			i = 0;
          }
	  }


}
