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

void spi(unsigned cs, uint16_t word) {
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
	BOARD_INITPINS_NOTCS_DAC_1_GPIO->PSOR = BOARD_INITPINS_NOTCS_DAC_1_GPIO_PIN_MASK;
	BOARD_INITPINS_NOTCS_DAC_0_GPIO->PSOR = BOARD_INITPINS_NOTCS_DAC_0_GPIO_PIN_MASK;

	// Without this delay, making an immediate next call to set unit B will fail (DAC won't latch)
	__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");

	// Note that DAC takes approx 4.5µs to slew 2.5V
}

#define SINCOS_POINTS 8         // multiple of 4


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


double wrapx(unsigned i) {
  return i < N_POINTS ? px[i] : -px[N_POINTS + N_POINTS - i - 2];
}
double wrapy(unsigned i) {
  return py[i < N_POINTS ? i : N_POINTS + N_POINTS - i - 2];
}

uint32_t xcoeff[N_POINTS*2+2], ycoeff[N_POINTS*2+2];
uint16_t pos_dac_x[N_POINTS*2+2], pos_dac_y[N_POINTS*2+2], limit_dac[N_POINTS*2+2];
uint32_t line_limit_x[N_POINTS*2+2],
		 line_limit_low[N_POINTS*2+2],
		 line_active[N_POINTS*2+2];

unsigned setup_line(unsigned i, double k, double x0, double y0, double x1, double y1) {
	double origin_x = 0, origin_y = 0; // shift position by this amount in DAC units (4095 is full scale = 2.5V)

	origin_x = 2048; // data is centred around origin
	origin_y = 2048;

	double dx = x1-x0, dy = y1-y0,
		   len = sqrt(dx*dx + dy*dy),
		   c = dx/len, s = dy/len;

	xcoeff[i] = dac_encode(c);
	ycoeff[i] = dac_encode(s);

	int32_t posx = (int32_t)( k*x0*0xfffu + origin_x ),
			posy = (int32_t)( k*y0*0xfffu + origin_y );

	pos_dac_x[i] = DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | (uint16_t)posx;
	pos_dac_y[i] = DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | (uint16_t)posy;

	line_limit_x[i] = fabs(dx) > fabs(dy); // set if X is faster changing integrator

	double larger_delta = line_limit_x[i] ? dx : dy;

	line_limit_low[i] = larger_delta > 0; // set if the integrator is decreasing (coefficient positive)

	int32_t limit = (int32_t)( (0.5 - k*larger_delta/2.0) * 0xfffu );

	// While the limit DAC can use almost the whole range between 0 and 5V (with integrator "zero" at 2.5V),
	// the integrators themselves cannot reach these limits. We therefore need to clamp the limit DAC
	// to a reduced, practical range, or the system will stop, waiting forever for a threshold that can't be reached.
	// In tests with LF412CP op amp, the integrators can rise to about 4V (+1.5V), and drop to about 1.3V (-1.2V).
	// With a different rail to rail amp, these limits can be increased.
	// FIXME: This range may not be enough! Because the scope is typically calibrated to 1V full deflection.
	// (As a failsafe, we might need a timeout as well.)
	// TODO: This can be self calibrating
	int32_t limit_max = (int32_t)( (3.9/5.0)*0xfffu );
	int32_t limit_min = (int32_t)( (1.4/5.0)*0xfffu );
	uint16_t clamped = (uint16_t)( limit < limit_min ? limit_min : (limit > limit_max ? limit_max : limit) );

	limit_dac[i] = (uint16_t)( (line_limit_x[i] ? DAC_A : DAC_B) | DAC_BUFFERED | DAC_GAINx2 | DAC_ACTIVE | clamped );

	// suppress lines that push DACs out of bounds
	// TODO: proper clipping
	return posx >= 0 && posx < 0x1000 && posy >= 0 && posy < 0x1000;
}

double starburst_costab[N_POINTS], starburst_sintab[N_POINTS];

void update_display_list(double k) {
	for(unsigned i = 0; i < (2*N_POINTS-2); ++i) {
		double x0, y0, x1, y1, kk = k;

		// x0 = wrapx(i);   y0 = wrapy(i);
		// x1 = wrapx(i+1); y1 = wrapy(i+1);



		/* starburst*/
		if (i < N_POINTS-1) {
			x0 = y0 = 0;
			x1 = starburst_costab[i]; y1 = starburst_sintab[i];
			kk = 0.2;
		} else {
			x0 = 30*starburst_costab[i - N_POINTS+1];
			y0 = 30*starburst_sintab[i - N_POINTS+1];
			x1 = 30*starburst_costab[i - N_POINTS+2];
			y1 = 30*starburst_sintab[i - N_POINTS+2];
		}


		// Show line if either endpoint is within valid position range
		// This is simpler than full clipping
		if (setup_line(i, kk, x0, y0, x1, y1)) {
			line_active[i] = 1;
		} else if (setup_line(i, kk, x1, y1, x0, y0)) {
			line_active[i] = 1;
		} else  {
			line_active[i] = 0;
		}
	}
}

void execute_line(unsigned i) {
	if(!line_active[i]) return;

    setCoefficients( xcoeff[i], ycoeff[i] );

	// Set either a high-crossing or low-crossing threshold at the limit DAC.
	// If limitlow[i] is set, it means we must invert the comparator output

	spi(DAC_LIMIT, limit_dac[i]);

	// Position DAC has 2.5V range
	spi(DAC_POS, pos_dac_x[i]); // TODO: These can be optimised to skip
	spi(DAC_POS, pos_dac_y[i]); //       if the value does not change

	if(line_limit_low[i]) {
		BOARD_INITPINS_LIMIT_LOW_FGPIO->PSOR = BOARD_INITPINS_LIMIT_LOW_GPIO_PIN_MASK;
	} else {
		BOARD_INITPINS_LIMIT_LOW_FGPIO->PCOR = BOARD_INITPINS_LIMIT_LOW_GPIO_PIN_MASK;
	}

	four_microseconds(); // just one of these calls isn't quite enough
	four_microseconds();

	// Arm comparator on the fastest-changing integrator
	// (greater magnitude coefficient of X and Y)
	if(line_limit_x[i]) {
		BOARD_INITPINS_X_COMP_SEL_FGPIO->PSOR = BOARD_INITPINS_X_COMP_SEL_GPIO_PIN_MASK;
		BOARD_INITPINS_Y_COMP_SEL_FGPIO->PCOR = BOARD_INITPINS_Y_COMP_SEL_GPIO_PIN_MASK;
	} else {
		BOARD_INITPINS_X_COMP_SEL_FGPIO->PCOR = BOARD_INITPINS_X_COMP_SEL_GPIO_PIN_MASK;
		BOARD_INITPINS_Y_COMP_SEL_FGPIO->PSOR = BOARD_INITPINS_Y_COMP_SEL_GPIO_PIN_MASK;
	}

    uint32_t limit_mask = line_limit_low[i] ? 0 : BOARD_INITPINS_STOP_GPIO_PIN_MASK;

    if(i == 0) BOARD_INITPINS_TRIGGER_FGPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger

    BOARD_INITPINS_X_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
	BOARD_INITPINS_Y_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET

	BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam ON

	BOARD_INITPINS_X_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch X
	BOARD_INITPINS_Y_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch Y


	// All the above takes about 30-32 µs

	// Wait integrating time

    while( limit_mask ^ (BOARD_INITPINS_STOP_FGPIO->PDIR & BOARD_INITPINS_STOP_GPIO_PIN_MASK) )
    	;

	BOARD_INITPINS_Z_BLANK_FGPIO->PCOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam OFF

	BOARD_INITPINS_TRIGGER_FGPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger

    BOARD_INITPINS_X_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch X
    BOARD_INITPINS_Y_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y

	// As soon as beam is off, we can short the integrator
    // Based on measurements, reset takes about 14µs
	BOARD_INITPINS_X_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
	BOARD_INITPINS_Y_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
}


int main(void) {
    /* Init the boards */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitLEDsPins();


	double a = 2*M_PI/(N_POINTS-1);
	for(int i = 0; i < N_POINTS; ++i) {
		starburst_costab[i] = cos(a*i);
		starburst_sintab[i] = sin(a*i);
	}

    uint32_t sintab[SINCOS_POINTS], costab[SINCOS_POINTS], limitx[SINCOS_POINTS], limitlow[SINCOS_POINTS];
    double coeffmag[SINCOS_POINTS];

    double k = 2*M_PI/SINCOS_POINTS;
    for(int i = 0; i < SINCOS_POINTS; ++i) {
    	double c = cos(i*k), s = sin(i*k);
        costab[i] = dac_encode(c);
        sintab[i] = dac_encode(s);
        limitx[i] = fabs(c) > fabs(s); // set if X is faster changing integrator
        limitlow[i] = limitx[i] ? c > 0 : s > 0; // set if the integrator is decreasing (coefficient positive)
        coeffmag[i] = limitx[i] ? fabs(c) : fabs(s); // test purposes, we will use this to set a testing "distance" for limit DAC
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

    if(0) {
    	/* Test levels

    	// ;  Vref 2.499
		spi(DAC_POS, DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0x800u); // should be ~ 1.25V  // 1.250
		spi(DAC_POS, DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0xfffu); // should be ~ 2.5V   // 2.499
		spi(DAC_LIMIT, DAC_A | DAC_GAINx2 | DAC_BUFFERED | DAC_ACTIVE | 0x800u); // should be ~ 2.5V  // 1.253 WHY????????
		//spi(DAC_LIMIT, DAC_A | DAC_GAINx2 | DAC_BUFFERED | DAC_ACTIVE | 0xc00u); // should be ~ 3.75V  // 3.751
		//spi(DAC_LIMIT, DAC_A | DAC_GAINx2 | DAC_BUFFERED | DAC_ACTIVE | 0xfffu); // should be ~ 5V  // 4.789
		spi(DAC_LIMIT, DAC_B | DAC_GAINx2 | DAC_BUFFERED | DAC_ACTIVE | 0x800u); // should be ~ 2.5V  // 2.512
		//spi(DAC_LIMIT, DAC_B | DAC_GAINx2 | DAC_BUFFERED | DAC_ACTIVE | 0xc00u); // should be ~ 3.75V  // 3.766
		//spi(DAC_LIMIT, DAC_B | DAC_GAINx2 | DAC_BUFFERED | DAC_ACTIVE | 0xfffu); // should be ~ 5V    // 4.78
		for(;;) ;*/
    }

    if (0) {
		for(unsigned i = 0; ; ++i) {
			// Test DAC ramp

			if((i & 0xfff) == 0) BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK;
			spi(DAC_LIMIT, DAC_A | DAC_GAINx2 | DAC_BUFFERED | DAC_ACTIVE | (i & 0xfff));
			spi(DAC_LIMIT, DAC_B | DAC_GAINx2 | DAC_BUFFERED | DAC_ACTIVE | (i & 0xfff)); // should be ~ 2.5V  // 1.253 WHY????????

			four_microseconds();
			BOARD_INITPINS_TRIGGER_GPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK;
		}
    }

    for(;0;) {
		BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger

		spi(0, DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0xfffu);
		spi(0, DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0xfffu);
		spi(1, DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0xfffu);
		spi(1, DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0xfffu);

		BOARD_INITPINS_TRIGGER_GPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger

		four_microseconds();

		spi(0, DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0x800u);
		spi(0, DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0x800u);
		spi(1, DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0x800u);
		spi(1, DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0x800u);

		four_microseconds();

		spi(0, DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0);
		spi(0, DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0);
		spi(1, DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0);
		spi(1, DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | 0);

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

		four_microseconds();
		four_microseconds();
		four_microseconds();

		BOARD_INITPINS_X_INT_RESET_GPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
		BOARD_INITPINS_Y_INT_RESET_GPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET

		BOARD_INITPINS_X_INT_HOLD_GPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch X
		BOARD_INITPINS_Y_INT_HOLD_GPIO->PSOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch Y


		// First ramp ---------------------------------------------


		BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger

		BOARD_INITPINS_Z_BLANK_GPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam ON

		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
		four_microseconds();
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
			spi(DAC_POS, DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | (i     & 2 ? 0xfff : 0));
			spi(DAC_POS, DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | ((i+1) & 2 ? 0xfff : 0));

			BOARD_INITPINS_TRIGGER_FGPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger
			four_microseconds();
		}
	}

	// Comparator test

	if(0) {
		// Set threshold voltage (in actual use, only one threshold needs to be set - for the fastest changing axis)
		uint16_t v = (uint16_t) ((3.0 / 5.0) * 0xfff);
		spi(DAC_LIMIT, DAC_A | DAC_GAINx2 | DAC_BUFFERED | DAC_ACTIVE | v);
		spi(DAC_LIMIT, DAC_B | DAC_GAINx2 | DAC_BUFFERED | DAC_ACTIVE | v);

		// Arm X comparator
		BOARD_INITPINS_X_COMP_SEL_GPIO->PSOR = BOARD_INITPINS_X_COMP_SEL_GPIO_PIN_MASK;
		BOARD_INITPINS_Y_COMP_SEL_GPIO->PCOR = BOARD_INITPINS_Y_COMP_SEL_GPIO_PIN_MASK;

		// A realistic pair of coefficients. X will change faster than Y (angle < 45°)
		setCoefficients( dac_encode(-cos(DEG2RAD(20.0))), dac_encode(-sin(DEG2RAD(20.0))) );

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
	}


	// Starburst test


	// Precompute control words for limit DAC
	// Slow because of floating point

	static uint16_t limit_dac_value[SINCOS_POINTS];

	for (unsigned i = 0; i < SINCOS_POINTS; ++i) {
		unsigned value = (unsigned)( ((2.5 + 0.25*coeffmag[i] * (limitlow[i] ? -1 : 1)) / 5.0) * 0xfff );
		limit_dac_value[i] = (uint16_t)( (limitx[i] ? DAC_A : DAC_B) | DAC_BUFFERED | DAC_GAINx2 | DAC_ACTIVE | value );
	}

	for(;0;) {

		for (unsigned i = 0; i < SINCOS_POINTS; ++i) {

            setCoefficients( costab[i], sintab[i] );

    		// Set either a high-crossing or low-crossing threshold at the limit DAC.
    		// If limitlow[i] is set, it means we must invert the comparator output

			spi(DAC_LIMIT, limit_dac_value[i]);

			// Position DAC has 2.5V range
    		spi(DAC_POS, DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | (i & 1 ? 0x600 : 0)); // 0.9375V
    		spi(DAC_POS, DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | (i & 2 ? 0x600 : 0)); // 0.625V

    		// Arm comparator on the fastest-changing integrator
			// (greater magnitude coeffient of X and Y)
			if(limitx[i]) {
				BOARD_INITPINS_X_COMP_SEL_FGPIO->PSOR = BOARD_INITPINS_X_COMP_SEL_GPIO_PIN_MASK;
				BOARD_INITPINS_Y_COMP_SEL_FGPIO->PCOR = BOARD_INITPINS_Y_COMP_SEL_GPIO_PIN_MASK;
			} else {
				BOARD_INITPINS_X_COMP_SEL_FGPIO->PCOR = BOARD_INITPINS_X_COMP_SEL_GPIO_PIN_MASK;
				BOARD_INITPINS_Y_COMP_SEL_FGPIO->PSOR = BOARD_INITPINS_Y_COMP_SEL_GPIO_PIN_MASK;
			}

			if(limitlow[i]) {
				BOARD_INITPINS_LIMIT_LOW_FGPIO->PSOR = BOARD_INITPINS_LIMIT_LOW_GPIO_PIN_MASK;
			} else {
				BOARD_INITPINS_LIMIT_LOW_FGPIO->PCOR = BOARD_INITPINS_LIMIT_LOW_GPIO_PIN_MASK;
			}

			four_microseconds();

			BOARD_INITPINS_X_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Open INT RESET
			BOARD_INITPINS_Y_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET

    		BOARD_INITPINS_X_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch X
    		BOARD_INITPINS_Y_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch Y

    		BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam ON

    		// All the above takes about 30-32 µs


			// Wait integrating time


		    if(i == 0) BOARD_INITPINS_TRIGGER_FGPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger

			for(unsigned j = 0; j < 100000 && ! (limitlow[i] ^ ((BOARD_INITPINS_STOP_FGPIO->PDIR & BOARD_INITPINS_STOP_GPIO_PIN_MASK) != 0)); ++j )
				;

			BOARD_INITPINS_Z_BLANK_FGPIO->PCOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK; // Turn beam OFF

    		BOARD_INITPINS_TRIGGER_FGPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger

		    BOARD_INITPINS_X_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch X
		    BOARD_INITPINS_Y_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y

    		// As soon as beam is off, we can short the integrator
		    // Based on measurements, reset takes about 14µs
			BOARD_INITPINS_X_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
			BOARD_INITPINS_Y_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
        }
	}


	//  square test pattern

	if(1) {
		// Note that X = 0 and Y = 0 correspond to Position DAC in mid-range, i.e. 1.25V
		// DAC value = k*x0*0xfffu + 2048
		// The addressable range of Position DAC is therefore
		// -2048/(4095*k) .. (4095-2048)/(4095*k) ... if k = 1,   -0.5 .. +0.5
		// The limit DAC (line endpoint, integrator stop) is intended to be in the same units

		double k = 0.5; // X ordinate from -0.5..+0.5
		line_active[0] = setup_line(0, k, -.5, -.5, +.5, -.5);
		line_active[1] = setup_line(1, k, +.5, -.5, +.5, +.5);
		line_active[2] = setup_line(2, k, +.5, +.5, -.5, +.5);
		line_active[3] = setup_line(3, k, -.5, +.5, -.5, -.5);
		line_active[4] = setup_line(4, k, -.5, -.4, +.5, -.4);
		line_active[5] = setup_line(5, k, -.5, +.4, +.5, +.4);
		line_active[6] = setup_line(6, k, -.5, 0, +.5, 0);
		line_active[7] = setup_line(7, k, 0, -.5, 0, +.5);
		line_active[8] = setup_line(8, k, -.4, -.5, -.4, +.5);
		line_active[9] = setup_line(9, k, +.4, -.5, +.4, +.5);

		for(;;) {
			execute_line(0);
			execute_line(1);
			execute_line(2);
			execute_line(3);
			execute_line(4);
			execute_line(5);
			execute_line(6);
			execute_line(7);
			execute_line(8);
			execute_line(9);
		}
	}

	// diagonal_test

	if(0) {
		// Note that X = 0 and Y = 0 correspond to Position DAC in mid-range, i.e. 1.25V
		// DAC value = k*x0*0xfffu + 2048
		// The addressable range of Position DAC is therefore
		// -2048/(4095*k) .. (4095-2048)/(4095*k) ... if k = 1,   -0.5 .. +0.5
		// The limit DAC (line endpoint, integrator stop) is intended to be in the same units

		double k = 0.25; // X ordinate from -2 .. 2

		// Pick angles so that X will be larger coefficient
		// first line dx=1, dy=1, this is 1023 position DAC units or 0.625V.
		// measured limit dac @ 1.878 X    2.200 Y   position DAC  2.200  2.200
		// limit should be target delta voltage (x1 or y1), biased by 2.5v which is the integrator "zero"
		//   int32_t limit = (int32_t)( (0.5 - k*larger_delta/2.0) * 0xfffu );
		// = (0.5 - 0.125) * 5.0
		line_active[0] = setup_line(0, k, 0, 0.95, 1, 0);
		line_active[1] = setup_line(1, k, 0, 0, .5, .475);
		line_active[2] = setup_line(2, k, 1, 0, 1+0.05*(0-1), 0+0.05*(0.95-0));
		line_active[3] = setup_line(3, k, .5, .475, .5+0.05*(0-.5), .475+0.05*(0-.475));

		for(;;) {
			execute_line(0);
			execute_line(1);
			execute_line(2);
			execute_line(3);
		}
	}

	// FLAG test

	int gg = 0;
	for(unsigned f = 0; ; ++f) {
		int g = (f >> 8) & 63;
		if (g != gg) {
			gg = g;
			k = g ? k*1.08 : 0.0002;
			update_display_list(k);
		}
		for(unsigned i = 0; i < (2*N_POINTS-2); ++i) {
			execute_line(i);
		}
	}


}
