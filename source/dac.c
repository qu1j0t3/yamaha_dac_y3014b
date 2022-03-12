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
#include "fsl_pit.h"

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

// chip select
#define DAC_LIMIT  0
#define DAC_POS    1
#define DAC_Z      2
#define DAC_COEFF  3

// units
#define DAC_A      0u
#define DAC_B      (1u << 15)

// control
#define DAC_BUFFERED (1u << 14)
#define DAC_UNBUFFERED 0u
#define DAC_GAINx2   0u
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
	if (cs == DAC_LIMIT){
		BOARD_INITPINS_NOTCS_DAC_0_FGPIO->PCOR = BOARD_INITPINS_NOTCS_DAC_0_GPIO_PIN_MASK;
	} else if (cs == DAC_POS) {
		BOARD_INITPINS_NOTCS_DAC_1_FGPIO->PCOR = BOARD_INITPINS_NOTCS_DAC_1_GPIO_PIN_MASK;
	} else if (cs == DAC_COEFF) {
		BOARD_INITPINS_NOTCS_DAC_COEFF_FGPIO->PCOR = BOARD_INITPINS_NOTCS_DAC_COEFF_GPIO_PIN_MASK;
	} else if (cs == DAC_Z) {
		//BOARD_INITPINS_NOTCS_DAC_Z_FGPIO->PCOR = BOARD_INITPINS_NOTCS_DAC_Z_GPIO_PIN_MASK;
	}

	// Bit-bang SPI
	for (unsigned i = 16; i--;) {
		if ((word >> i) & 1) {
			BOARD_INITPINS_SPI_DATAOUT_FGPIO->PSOR = BOARD_INITPINS_SPI_DATAOUT_GPIO_PIN_MASK;
		} else {
			BOARD_INITPINS_SPI_DATAOUT_FGPIO->PCOR = BOARD_INITPINS_SPI_DATAOUT_GPIO_PIN_MASK;
		}
		BOARD_INITPINS_SPI_CLOCK_FGPIO->PSOR = BOARD_INITPINS_SPI_CLOCK_GPIO_PIN_MASK;
		BOARD_INITPINS_SPI_CLOCK_FGPIO->PCOR = BOARD_INITPINS_SPI_CLOCK_GPIO_PIN_MASK;
	}

	// Deselect chip (and also latch DAC data when NOT_LDAC is tied low)
	BOARD_INITPINS_NOTCS_DAC_1_FGPIO->PSOR = BOARD_INITPINS_NOTCS_DAC_1_GPIO_PIN_MASK;
	BOARD_INITPINS_NOTCS_DAC_0_FGPIO->PSOR = BOARD_INITPINS_NOTCS_DAC_0_GPIO_PIN_MASK;
	//BOARD_INITPINS_NOTCS_DAC_Z_FGPIO->PSOR = BOARD_INITPINS_NOTCS_DAC_Z_GPIO_PIN_MASK;
	BOARD_INITPINS_NOTCS_DAC_COEFF_FGPIO->PSOR = BOARD_INITPINS_NOTCS_DAC_COEFF_GPIO_PIN_MASK;

	// Without this delay, making an immediate next call to set unit B will fail (DAC won't latch)
	__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
	__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
	__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
	__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
	__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");

	// Note that DAC takes approx 4.5µs to slew 2.5V   ; 7.2µs to slew 4.4v
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

#define DISPLAY_LIST_MAX 450
#define NORMALISE_DIRECTIONS 0

// If FRAME_SYNC is 1, then the execution of the display list
// will wait until at least one PIT tick has elapsed (see init code),
// producing a frame rate that is somewhat decoupled from
// the size of the display list ... the big problem I'm
// having is the refresh rate beating against strong 60Hz
// interference in my lab, so I'm trying to kick the refresh rate
// away from multiples of 60Hz...

#define FRAME_SYNC 0

#define MAX_Z_LEVEL 0xfffu

uint32_t line_dash[DISPLAY_LIST_MAX];
uint16_t pos_dac_x[DISPLAY_LIST_MAX],
		 pos_dac_y[DISPLAY_LIST_MAX],
		 limit_dac[DISPLAY_LIST_MAX],
		 xcoeff[DISPLAY_LIST_MAX],
		 ycoeff[DISPLAY_LIST_MAX],
		 line_limit_x[DISPLAY_LIST_MAX],
		 line_limit_low[DISPLAY_LIST_MAX],
		 line_active[DISPLAY_LIST_MAX],
		 is_point[DISPLAY_LIST_MAX],
		 line_z_dac[DISPLAY_LIST_MAX];

volatile uint32_t ticks;

void PIT_CH0_IRQHandler(void)
{
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);

    ++ticks;

    __DSB();
}


int compar_func(const void *pa, const void *pb) {
	uint16_t a = *(const uint16_t*)pa;
	uint16_t b = *(const uint16_t*)pb;

	// sort lines left to right by X starting position
	int sign = (pos_dac_x[a] & 0xfff) - (pos_dac_x[b] & 0xfff);
	return sign ? sign : (
		// Then sort by Y
		(pos_dac_y[a] & 0xfff) - (pos_dac_y[b] & 0xfff)
	);
}

void sort_display_list(unsigned count, uint16_t perm[]) {
	qsort(perm, count, sizeof(perm[0]), compar_func);
}

void shuffle_display_list(uint16_t count, uint16_t perm[]) {
	for(unsigned n = count; n > 1;) {
		unsigned chosen = (uint16_t)rand() % n;
		--n;
		uint16_t temp = perm[n];
		perm[n] = perm[chosen];
		perm[chosen] = temp;
	}
}

/*
 * Parameters:  x0, y0, x1, y1 : line endpoints; range from -2047 to +2048
 *              dash           : 30 bits of dash pattern; zero for solid line
 *              zlevel         : Z intensity level from 0 (lowest) to 4095 (highest)
                                 but hardware has 8 bit precision
 */

unsigned setup_line_int_(unsigned i, int x0, int y0, int x1, int y1, uint32_t dash, uint16_t zlevel, uint8_t slow) {
	int dx = x1-x0, dy = y1-y0, posx, posy;
	double len = sqrt((double)dx*(double)dx + (double)dy*(double)dy);

	// treat very short lines as points (since crt cannot resolve anyway)
	// can be calibrated using short lines test pattern below
	// at some point around length 7 units, lines start to lose brightness
	is_point[i] = len < 2.0;
	line_z_dac[i] = DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | zlevel;

	if (is_point[i]) {
		posx = 2048 - (x0+x1)/2;
		posy = 2048 - (y0+y1)/2;
	} else {
		double speed = slow ? 0.5 : 1.0; // This is mostly intended to help text quality, by slowing down the integrators for short lines
		double c = dx/len, s = dy/len;

		posx = 2048 - x0;
		posy = 2048 - y0;

		line_dash[i] = dash;

		// DAC Datasheet output swing = typ 0.01 to Vdd-0.04
		// When powered from USB, Vdd can swing as low as 4.53V-4.80V in my setup (usb hub with other devices connected)
		// i.e. 0.01 to 4.76 which is a scale of 0.952 x full range. (WORST case 0.898)
		// Use 0.9x for now to give a little margin for Vdd
		// N.B. This is best verified by using the Starburst test below. When Vdd is too low,
		//      coefficient error near Vdd causes lines to meet and cross some distance from the centre of the pattern.
		//      When the full coefficient range is available, lines should meet close to the centre.
		xcoeff[i] = DAC_A | DAC_UNBUFFERED | DAC_GAINx2 | DAC_ACTIVE | (uint16_t)((c*speed*0.9/2.0 + 0.5)*0xfff + 0.5);
		ycoeff[i] = DAC_B | DAC_UNBUFFERED | DAC_GAINx2 | DAC_ACTIVE | (uint16_t)((s*speed*0.9/2.0 + 0.5)*0xfff + 0.5);

		line_limit_x[i] = abs(dx) > abs(dy); // set if X is faster changing integrator

		int larger_delta = line_limit_x[i] ? dx : dy;

		line_limit_low[i] = larger_delta > 0; // set if the integrator is decreasing (coefficient positive)

		uint16_t delta = (uint16_t)abs(larger_delta)/2;
		uint16_t clamp;
		if (line_limit_low[i]) {
			// limit = 2048 - delta
			clamp = delta > 2048 ? 0 : 2048 - delta;
		} else {
			// limit = 2048 + delta
			clamp = delta > 2047 ? 4095 : 2048 + delta;
		}

		// While the limit DAC can use almost the whole range between 0 and 5V (with integrator "zero" at 2.5V),
		// the integrators themselves cannot reach these limits. We therefore need to clamp the limit DAC
		// to a reduced, practical range, or the system will stop, waiting forever for a threshold that can't be reached.
		// In tests with LF412CP op amp, the integrators can rise to about 4V (+1.5V), and drop to about 1.3V (-1.2V).
		// With a different rail to rail amp, these limits can be increased.
		// FIXME: This range may not be enough! Because the scope is typically calibrated to 1V full deflection.
		// (As a failsafe, we might need a timeout as well.)
		// TODO: This can be self calibrating

		/* Update: Changed to MCP6292 op amp for integrator and removed the clamping.
		int32_t limit_max = (int32_t)( (4.95/5.0)*0xfffu );
		int32_t limit_min = (int32_t)( (0.05/5.0)*0xfffu );
		uint16_t clamped = limit; //(uint16_t)( limit < limit_min ? limit_min : (limit > limit_max ? limit_max : limit) );*/

		limit_dac[i] = (uint16_t)( (line_limit_x[i] ? DAC_A : DAC_B) | DAC_UNBUFFERED | DAC_GAINx2 | DAC_ACTIVE | clamp );
	}

	pos_dac_x[i] = DAC_A | DAC_UNBUFFERED | DAC_GAINx1 | DAC_ACTIVE | (uint16_t)posx;
	pos_dac_y[i] = DAC_B | DAC_UNBUFFERED | DAC_GAINx1 | DAC_ACTIVE | (uint16_t)posy;

	// suppress lines that push DACs out of bounds
	// TODO: proper clipping
	return line_active[i] = posx >= 0 && posx < 0x1000 && posy >= 0 && posy < 0x1000;
}

unsigned setup_line_int(unsigned i, int x0, int y0, int x1, int y1, uint32_t dash, uint16_t zlevel, uint8_t slow) {
	/* This higher level call will try to position line in a normalised direction (X increasing, Y increasing)
	// and if that leads to a starting position outside DAC limits,
	// will then try to place line reversed (which is probably not so good for display list sorting
	// but necessary to have the partial line rendered at all).*/

	if (NORMALISE_DIRECTIONS && x0 > x1) {
		return setup_line_int_(i, x1, y1, x0, y0, dash, zlevel, slow)
				|| setup_line_int_(i, x0, y0, x1, y1, dash, zlevel, slow);
	}

	return setup_line_int_(i, x0, y0, x1, y1, dash, zlevel, slow)
			|| setup_line_int_(i, x1, y1, x0, y0, dash, zlevel, slow);
}

// Assumes data is centred on (0,0); k factor will scale input coordinates to -0.5 .. +0.5
unsigned setup_line(unsigned i, double k, double x0, double y0, double x1, double y1, uint32_t dash) {
	k *= 0xfff; // scale to position DAC units
	return setup_line_int(i, (int)(k*x0), (int)(k*y0), (int)(k*x1), (int)(k*y1), dash, MAX_Z_LEVEL, 0);
}
unsigned setup_line_dim(unsigned i, double k, double x0, double y0, double x1, double y1) {
	k *= 0xfff; // scale to position DAC units
	return setup_line_int(i, (int)(k*x0), (int)(k*y0), (int)(k*x1), (int)(k*y1), 0, MAX_Z_LEVEL*95/100, 0);
}

// Modulo is quite slow on this processor, so use an
// array lookup as a fast but constant time "mod 30"
static uint8_t next[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,0};

#define ON 0x80u
#define ENDCHAR 0xffu
uint8_t font1[] = {
		// cell is 6 wide by 10 high. baseline is y=2
		// first byte, starting position
		// subsequent bytes until 0: new position. ON = beam on
		' ', 0x00, ENDCHAR,
		'A', 0x02, ON|0x07, ON|0x3a, ON|0x67, ON|0x62, 0x05, ON|0x65, ENDCHAR,
		'B', 0x02, ON|0x52, ON|0x64, ON|0x56, ON|0x68, ON|0x5a, ON|0x0a, ON|0x02, 0x06, ON|0x56, ENDCHAR,
		'C', 0x64, ON|0x52, ON|0x12, ON|0x04, ON|0x08, ON|0x1a, ON|0x5a, ON|0x68, ENDCHAR,
		'D', 0x12, ON|0x1a, ON|0x4a, ON|0x68, ON|0x64, ON|0x42, ON|0x12, ENDCHAR,
		'E', 0x62, ON|0x02, ON|0x0a, ON|0x6a, 0x06, ON|0x56, ENDCHAR,
		'F', 0x02, ON|0x0a, ON|0x6a, 0x06, ON|0x56, ENDCHAR,
		'G', 0x35, ON|0x65, ON|0x52, ON|0x12, ON|0x04, ON|0x08, ON|0x1a, ON|0x5a, ON|0x68, ENDCHAR,
		'H', 0x02, ON|0x0a, 0x06, ON|0x66, 0x62, ON|0x6a, ENDCHAR,
		'I', 0x22, ON|0x42, 0x32, ON|0x3a, 0x2a, ON|0x4a, ENDCHAR,
		'J', 0x13, ON|0x22, ON|0x42, ON|0x54, ON|0x5a, ON|0x3a, ENDCHAR,
		'K', 0x12, ON|0x1a, 0x6a, ON|0x16, ON|0x62, ENDCHAR,
		'L', 0x62, ON|0x12, ON|0x1a, ENDCHAR,
		'M', 0x02, ON|0x0a, ON|0x35, ON|0x6a, ON|0x62, ENDCHAR,
		'N', 0x02, ON|0x0a, ON|0x52, ON|0x5a, ENDCHAR,
		'O', 0x12, ON|0x52, ON|0x64, ON|0x68, ON|0x5a, ON|0x1a, ON|0x08, ON|0x04, ON|0x12, ENDCHAR,
		'P', 0x12, ON|0x1a, ON|0x5a, ON|0x68, ON|0x67, ON|0x55, ON|0x15, ENDCHAR,
		'Q', 0x12, ON|0x52, ON|0x64, ON|0x68, ON|0x5a, ON|0x1a, ON|0x08, ON|0x04, ON|0x12, 0x34, ON|0x50, ENDCHAR,
		'R', 0x12, ON|0x1a, ON|0x5a, ON|0x68, ON|0x67, ON|0x55, ON|0x15, 0x55, ON|0x72, ENDCHAR,
		'S', 0x12, ON|0x52, ON|0x64, ON|0x56, ON|0x16, ON|0x08, ON|0x1a, ON|0x5a, ENDCHAR,
		'T', 0x32, ON|0x3a, 0x0a, ON|0x6a, ENDCHAR,
		'U', 0x0a, ON|0x04, ON|0x12, ON|0x42, ON|0x54, ON|0x5a, ENDCHAR,
		'V', 0x0a, ON|0x32, ON|0x6a, ENDCHAR,
		'W', 0x0a, ON|0x12, ON|0x37, ON|0x52, ON|0x6a, ENDCHAR,
		'X', 0x02, ON|0x6a, 0x0a, ON|0x62, ENDCHAR,
		'Y', 0x0a, ON|0x35, ON|0x6a, 0x35, ON|0x32, ENDCHAR,
		'Z', 0x0a, ON|0x5a, ON|0x02, ON|0x52, ENDCHAR,
		'0', 0x22, ON|0x32, ON|0x55, ON|0x57, ON|0x3a, ON|0x2a, ON|0x07, ON|0x05, ON|0x22, ENDCHAR,
		'1', 0x32, ON|0x3a, ON|0x18, ENDCHAR,
		'2', 0x52, ON|0x02, ON|0x57, ON|0x4a, ON|0x1a, ON|0x08, ENDCHAR,
		'3', 0x13, ON|0x42, ON|0x63, ON|0x56, ON|0x26, 0x56, ON|0x69, ON|0x4a, ON|0x19, ENDCHAR,
		'4', 0x42, ON|0x4a, ON|0x04, ON|0x54, ENDCHAR,
		'5', 0x03, ON|0x22, ON|0x42, ON|0x54, ON|0x46, ON|0x06, ON|0x0a, ON|0x4a, ENDCHAR,
		'6', 0x4a, ON|0x1a, ON|0x07, ON|0x05, ON|0x12, ON|0x42, ON|0x54, ON|0x47, ON|0x06, ENDCHAR,
		'7', 0x22, ON|0x5a, ON|0x0a, ENDCHAR,
		'8', 0x12, ON|0x42, ON|0x54, ON|0x46, ON|0x58, ON|0x4a, ON|0x1a, ON|0x08, ON|0x16, ON|0x46, 0x16, ON|0x04, ON|0x12, ENDCHAR,
		'9', 0x12, ON|0x42, ON|0x55, ON|0x57, ON|0x4a, ON|0x1a, ON|0x08, ON|0x15, ON|0x56, ENDCHAR,
		'.', 0x32, ON|0x32, ENDCHAR,
		'\'', 0x3a, ON|0x27, ENDCHAR
};

uint8_t seven_segment_font[] = {
		' ', 0x00, ENDCHAR,
		'0', 0x03, ON|0x06, 0x08, ON|0x0b, 0x1c, ON|0x4c, 0x5b, ON|0x58, 0x56, ON|0x53, 0x42, ON|0x12, ENDCHAR,
		'1', 0x5b, ON|0x58, 0x56, ON|0x53, ENDCHAR,
		'2', 0x03, ON|0x06, 0x1c, ON|0x4c, 0x5b, ON|0x58, 0x42, ON|0x12, 0x17, ON|0x47, ENDCHAR,
		'3', 0x1c, ON|0x4c, 0x5b, ON|0x58, 0x56, ON|0x53, 0x42, ON|0x12, 0x17, ON|0x47, ENDCHAR,
		'4', 0x08, ON|0x0b, 0x5b, ON|0x58, 0x56, ON|0x53, 0x17, ON|0x47, ENDCHAR,
		'5', 0x08, ON|0x0b, 0x1c, ON|0x4c, 0x56, ON|0x53, 0x42, ON|0x12, 0x17, ON|0x47, ENDCHAR,
		'6', 0x03, ON|0x06, 0x08, ON|0x0b, 0x1c, ON|0x4c, 0x56, ON|0x53, 0x42, ON|0x12, 0x17, ON|0x47, ENDCHAR,
		'7', 0x1c, ON|0x4c, 0x5b, ON|0x58, 0x56, ON|0x53, ENDCHAR,
		'8', 0x03, ON|0x06, 0x08, ON|0x0b, 0x1c, ON|0x4c, 0x5b, ON|0x58, 0x56, ON|0x53, 0x42, ON|0x12, 0x17, ON|0x47, ENDCHAR,
		'9', 0x08, ON|0x0b, 0x1c, ON|0x4c, 0x5b, ON|0x58, 0x56, ON|0x53, 0x42, ON|0x12, 0x17, ON|0x47, ENDCHAR,
		'A', 0x03, ON|0x06, 0x08, ON|0x0b, 0x1c, ON|0x4c, 0x5b, ON|0x58, 0x56, ON|0x53, 0x17, ON|0x47, ENDCHAR,
		'B', 0x03, ON|0x06, 0x08, ON|0x0b, 0x56, ON|0x53, 0x42, ON|0x12, 0x17, ON|0x47, ENDCHAR,
		'C', 0x03, ON|0x06, 0x08, ON|0x0b, 0x1c, ON|0x4c, 0x42, ON|0x12, ENDCHAR,
		'D', 0x03, ON|0x06, 0x5b, ON|0x58, 0x56, ON|0x53, 0x42, ON|0x12, 0x17, ON|0x47, ENDCHAR,
		'E', 0x03, ON|0x06, 0x08, ON|0x0b, 0x1c, ON|0x4c, 0x42, ON|0x12, 0x17, ON|0x47, ENDCHAR,
		'F', 0x03, ON|0x06, 0x08, ON|0x0b, 0x1c, ON|0x4c, 0x17, ON|0x47, ENDCHAR,
		'-', 0x17, ON|0x47, ENDCHAR,
		'.', 0x32, ON|0x32, ENDCHAR,
};

uint8_t *current_font;
uint16_t charmap[0x80];

void index_chardata(uint8_t chardata[], size_t font_bytes) {
	uint16_t i = 0;
    while(i < font_bytes) {
    	charmap[ chardata[i] ] = i;
    	++i; // skip character marker
    	while(i < font_bytes && chardata[i] != ENDCHAR)
    		++i;
    	++i; // skip endchar marker
    }
    current_font = chardata;
}

unsigned setup_text(unsigned idx, int x, int y, int scale, char *s) {
	int lastx, lasty;
	for(uint8_t *pc = (uint8_t*)s; *pc && idx < DISPLAY_LIST_MAX; ++pc) {
		uint8_t *p;
		unsigned i;

		for(i = 0, p = current_font + charmap[*pc] + 1 ; *p != ENDCHAR ; ++p, ++i) {
			int posx = x + scale*((*p & 0x70) >> 4),
				posy = y + scale*(*p & 0x0f);
			if(i && (*p & ON)) {
				// Note: improve quality at small text sizes using `slow` flag
				setup_line_int(idx++, lastx, lasty, posx, posy, 0, MAX_Z_LEVEL, scale < 40);
			}
			lastx = posx;
			lasty = posy;
		}
		x += scale*9; // advance to the right by one character position
	}
	return idx;
}

void execute_line(unsigned i) {
	static uint16_t last_pos_x, last_pos_y, last_z, last_xcoeff, last_ycoeff;

	if(FRAME_SYNC && i == 0) {
		// Sync to timer interrupt
		for(uint32_t current_tick = ticks; current_tick == ticks;)
			;
	}

	if(!line_active[i]) return;

	if (line_z_dac[i] != last_z) {
		spi(DAC_Z, line_z_dac[i]);
		last_z = line_z_dac[i];
	}

	if (is_point[i]) {
		// Assume this initial state
		//BOARD_INITPINS_Z_ENABLE_FGPIO->PCOR = BOARD_INITPINS_Z_ENABLE_GPIO_PIN_MASK;

		BOARD_INITPINS_Z_BLANK_FGPIO->PCOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK;

		// Set HOLD LOW (integrators disconnected)
	    BOARD_INITPINS_Y_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y

		// When both comparators are deselected, COMP_LIMIT is LOW.
	    BOARD_INITPINS_X_COMP_SEL_FGPIO->PCOR = BOARD_INITPINS_X_COMP_SEL_GPIO_PIN_MASK;
		BOARD_INITPINS_Y_COMP_SEL_FGPIO->PCOR = BOARD_INITPINS_Y_COMP_SEL_GPIO_PIN_MASK;

		BOARD_INITPINS_LIMIT_LOW_FGPIO->PCOR = BOARD_INITPINS_LIMIT_LOW_GPIO_PIN_MASK;
		// Z output is computed as HOLD ^ Z_BLANK ^ LIMIT_LOW ^ COMP_LIMIT
		// i.e. 0 ^ Z_BLANK ^ 1 ^ 0   ... so Z_BLANK is lowered to turn beam on

		// After a preceding line, we need to wait for integrator reset to complete.
		// Unconditionally setting these DACs gives a bit of extra time.
		// TODO: Measure this and compare with actual reset time required.

		spi(DAC_POS, pos_dac_x[i]);
		last_pos_x = pos_dac_x[i];
		spi(DAC_POS, pos_dac_y[i]);
		last_pos_y = pos_dac_y[i];

		four_microseconds();
		four_microseconds();
		four_microseconds();

		// Unblank Z
	    BOARD_INITPINS_Z_ENABLE_FGPIO->PSOR = BOARD_INITPINS_Z_ENABLE_GPIO_PIN_MASK;
	    BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK;

		delay(line_dash[i] ? line_dash[i] : 20); // This duration can be adjusted!

		// Blank Z
	    BOARD_INITPINS_Z_ENABLE_FGPIO->PCOR = BOARD_INITPINS_Z_ENABLE_GPIO_PIN_MASK;
	    BOARD_INITPINS_Z_BLANK_FGPIO->PCOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK;

		return;
	}

	// To get maximum precision from coefficient DACs, we want a range close to maximum range.
	// Basic coefficient should range from 0.707 to 1.0. This is scaled by 0.9 (see above)
	// but will either be + or - so the slew can be 90% of the DAC's range, and gain is 2x,
	// so 0.9 x 2 x 2.5 = 4.5V. This takes about 8µs.
	// We cannot release HOLD and RESET until coefficient DACs have settled.


	if (xcoeff[i] != last_xcoeff) {
		spi(DAC_COEFF, xcoeff[i]);
		last_xcoeff = xcoeff[i];
	}
	if (ycoeff[i] != last_ycoeff) {
		spi(DAC_COEFF, ycoeff[i]);
		last_ycoeff = ycoeff[i];
	}

// These may also take 8-9µs to settle
	if (pos_dac_x[i] != last_pos_x) {
		spi(DAC_POS, pos_dac_x[i]);
		last_pos_x = pos_dac_x[i];
	}
	if (pos_dac_y[i] != last_pos_y) {
		spi(DAC_POS, pos_dac_y[i]);
		last_pos_y = pos_dac_y[i];
	}

	// Set either a high-crossing or low-crossing threshold at the limit DAC.
	// If limitlow[i] is set, it means we must invert the comparator output
	spi(DAC_LIMIT, limit_dac[i]);

    // Add some settling time for limit DAC. Without this, some lines may be dropped/truncated.

	// By testing the display list in random order, we can see that repeatability
	// is not very good even if ample settling time (e.g. 12µs) is allowed here.
	//   Therefore the display list should be ordered for good locality,
	// not just in position, but also coefficients to an extent, which tend to slew far
	// even for close-together lines.
	//   We may need to consider running the coefficient DACs at GAIN X 1
	// (can compensate by halving the integration resistors), which makes the slew faster?
	// TODO: Although it's still unclear whether buffered/unbuffered affects this.

	four_microseconds();
	//four_microseconds();
	//four_microseconds();


	BOARD_INITPINS_Y_INT_RESET_FGPIO->PCOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Open INT RESET

	// UNBLANK

	// Arm comparator on the fastest-changing integrator
	// (greater magnitude coefficient of X and Y)
	if(line_limit_x[i]) {
		BOARD_INITPINS_X_COMP_SEL_FGPIO->PSOR = BOARD_INITPINS_X_COMP_SEL_GPIO_PIN_MASK;
		BOARD_INITPINS_Y_COMP_SEL_FGPIO->PCOR = BOARD_INITPINS_Y_COMP_SEL_GPIO_PIN_MASK;
	} else {
		BOARD_INITPINS_X_COMP_SEL_FGPIO->PCOR = BOARD_INITPINS_X_COMP_SEL_GPIO_PIN_MASK;
		BOARD_INITPINS_Y_COMP_SEL_FGPIO->PSOR = BOARD_INITPINS_Y_COMP_SEL_GPIO_PIN_MASK;
	}

    if(line_limit_low[i]) {
    	BOARD_INITPINS_LIMIT_LOW_FGPIO->PCOR = BOARD_INITPINS_LIMIT_LOW_GPIO_PIN_MASK;
    } else {
    	BOARD_INITPINS_LIMIT_LOW_FGPIO->PSOR = BOARD_INITPINS_LIMIT_LOW_GPIO_PIN_MASK;
    }

    if(i == 0) BOARD_INITPINS_TRIGGER_FGPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger

    BOARD_INITPINS_Z_ENABLE_FGPIO->PSOR = BOARD_INITPINS_Z_ENABLE_GPIO_PIN_MASK;

	BOARD_INITPINS_Y_INT_HOLD_FGPIO->PSOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Close HOLD switch Y


	// All the above takes about 30-32 µs


	// Wait integrating time

	if (line_dash[i]) {
		for(uint32_t dash_mask = 0; BOARD_INITPINS_STOP_FGPIO->PDIR & BOARD_INITPINS_STOP_GPIO_PIN_MASK; dash_mask = next[dash_mask]) {
			if(line_dash[i] & (1u << dash_mask)) {
				BOARD_INITPINS_Z_BLANK_FGPIO->PCOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK;
			} else {
				BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK;
			}
		}
		// it may be important to leave this in a known state
		// (debugging weird flashes at the end of the dashed lines)
		BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK;
	} else {
		// solid line
		BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK;
		while(BOARD_INITPINS_STOP_FGPIO->PDIR & BOARD_INITPINS_STOP_GPIO_PIN_MASK)
			;
	}

	BOARD_INITPINS_Z_ENABLE_FGPIO->PCOR = BOARD_INITPINS_Z_ENABLE_GPIO_PIN_MASK; // Make sure Z stays blanked when HOLD is made low

	BOARD_INITPINS_TRIGGER_FGPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger

	// Disarm comparator
    BOARD_INITPINS_Y_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y

	// As soon as beam is off, we can short the integrator
    // Based on measurements, reset takes about 14µs
	BOARD_INITPINS_Y_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch
}



int main(void) {

    /* Structure of initialize PIT */
    pit_config_t pitConfig;


    /* Init the boards */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitLEDsPins();

    /*
     * pitConfig.enableRunInDebug = false;
     */
    PIT_GetDefaultConfig(&pitConfig);

    /* Init pit module */
    PIT_Init(PIT, &pitConfig);

    /* Set timer period for channel 0 */
    // Bus Clock looks like 10 MHz... ?
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, 15625);
    //PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, 83333); // ~ 120Hz

    /* Enable timer interrupts for channel 0 */
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(PIT_CH0_IRQn);

    /* Start channel 0 */
    PIT_StartTimer(PIT, kPIT_Chnl_0);


    index_chardata(seven_segment_font, sizeof(seven_segment_font));


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

    if (0) {
		for(;;) {

			spi(DAC_POS, DAC_A | DAC_GAINx2 | DAC_UNBUFFERED | DAC_ACTIVE | (0xfffu*4/5) /* ~ 4V */ ); // One of these calls is about 7.5µs
			spi(DAC_POS, DAC_B | DAC_GAINx1 | DAC_UNBUFFERED | DAC_ACTIVE | (0xfffu*4/5) /* ~ 2V */ ); // Slew rate is about 0.53 V/µs so about 3.8µs for full scale at GAIN x1
			BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK;


			four_microseconds();
			four_microseconds();
			BOARD_INITPINS_TRIGGER_GPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK;

			spi(DAC_POS, DAC_A | DAC_GAINx2 | DAC_UNBUFFERED | DAC_ACTIVE | 0);
			spi(DAC_POS, DAC_B | DAC_GAINx1 | DAC_UNBUFFERED | DAC_ACTIVE | 0);

			four_microseconds();
		}
    }

	// Positioning box test
	// This can be used to verify basic range of position DACs.
	// NOTE (Tek 604 monitor): If you can't find the trace, Z input may be overdriven!
	// - intensity starts to oscillate around +0.8V and may disappear at higher levels
	// - for mid-intensity control, a Z level of 0.25V should produce a visible trace.

	if(0) {
		BOARD_INITPINS_Y_INT_HOLD_FGPIO->PCOR = BOARD_INITPINS_Y_INT_HOLD_GPIO_PIN_MASK; // Open HOLD switch Y
		BOARD_INITPINS_Y_INT_RESET_FGPIO->PSOR = BOARD_INITPINS_Y_INT_RESET_GPIO_PIN_MASK; // Close INT RESET switch

		// When both comparators are deselected, COMP_LIMIT is LOW.
		BOARD_INITPINS_X_COMP_SEL_FGPIO->PCOR = BOARD_INITPINS_X_COMP_SEL_GPIO_PIN_MASK;
		BOARD_INITPINS_Y_COMP_SEL_FGPIO->PCOR = BOARD_INITPINS_Y_COMP_SEL_GPIO_PIN_MASK;

		BOARD_INITPINS_LIMIT_LOW_FGPIO->PCOR = BOARD_INITPINS_LIMIT_LOW_GPIO_PIN_MASK;
		// Z output is computed as HOLD ^ Z_BLANK ^ LIMIT_LOW ^ COMP_LIMIT
		// i.e. 0 ^ Z_BLANK ^ 0 ^ 0   ... so Z_BLANK is raised to turn beam on

		// Unblank Z
		BOARD_INITPINS_Z_ENABLE_FGPIO->PSOR = BOARD_INITPINS_Z_ENABLE_GPIO_PIN_MASK;
		BOARD_INITPINS_Z_BLANK_FGPIO->PSOR = BOARD_INITPINS_Z_BLANK_GPIO_PIN_MASK;

		for(unsigned i = 0; 1; ++i) {

			if((i & 3) == 0) BOARD_INITPINS_TRIGGER_GPIO->PSOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Raise trigger
			spi(DAC_POS, DAC_A | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | (i     & 2 ? 0xfff : 0));
			spi(DAC_POS, DAC_B | DAC_GAINx1 | DAC_BUFFERED | DAC_ACTIVE | ((i+1) & 2 ? 0xfff : 0));

			BOARD_INITPINS_TRIGGER_FGPIO->PCOR = BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK; // Drop trigger
			four_microseconds();
		}
	}

	// Imlac screengrab for Maze Wars

	if(0) {
		unsigned cnt = 0;
#define LINE(x0,y0,x1,y1) if(!setup_line_int(cnt++, 2*(x0-1022), 2*(y0-1022), 2*(x1-1022), 2*(y1-1022), 0, MAX_Z_LEVEL, 0)) goto square;
#include "/Users/toby/Documents/Electronics/vectors_v2/larsb-imlac/maze.c"

		for(;;) {
			for(unsigned i = 0; i < cnt; ++i) {
				execute_line(i);
			}
		}
	}

	//  square test pattern

	square:
	if(0) {
		// Note that X = 0 and Y = 0 correspond to Position DAC in mid-range, i.e. 1.25V
		// DAC value = k*x0*0xfffu + 2048      or, k = 2048/(4095 * abs(max_x_y))
		// The addressable range of Position DAC is therefore
		// -2048/(4095*k) .. (4095-2048)/(4095*k) ... if k = 1,   -0.5 .. +0.5
		// The limit DAC (line endpoint, integrator stop) is intended to be in the same units

		double k = 0.75;
		uint32_t dash = 0b100100100100100100100100100100;
		uint32_t dash2 = 0b111110000011111000001111100000;
		unsigned j = 0;
		setup_line(j++, k, -.5, -.5, +.5, -.5, 0);
		setup_line(j++, k, +.5, -.5, +.5, +.5, 0);
		setup_line(j++, k, +.5, +.5, -.5, +.5, 0);
		setup_line(j++, k, -.5, +.5, -.5, -.5, 0);

		setup_line(j++, k, -.4, +.5, -.4, -.5, dash);
		setup_line(j++, k, +.4, +.5, +.4, -.5, 0);
		setup_line(j++, k,   0, +.5,   0, -.5, dash2);

		setup_line(j++, k, -.5, -.4, +.5, -.4, dash);
		setup_line(j++, k, -.5, +.4, +.5, +.4, 0);
		setup_line(j++, k, -.5,   0, +.5,   0, dash2);

		setup_line(j++, k, -.25,-.25,+.25,+.25, 0);
		setup_line(j++, k, -.25,+.25,+.25,-.25, 0);

		for(;;) {
			for(unsigned i = 0; i < j; ++i) {
				execute_line(i);
			}
		}
	}

	// Text demo

	if(0) {
		double k = 0.75;

		unsigned j = 0;
		//j = setup_text(j, (int)(k * -0.4 * 0xfff), (int)(k * 0.3 * 0xfff), 30, "HELLO");

		for(unsigned f = 0; ; ++f) {
			if((f % 250) == 0) {
				unsigned g = f/250;
				char n[10];
				sprintf(n, "%d", g);
				j = 0;
				setup_line(j++, k, +.5, 0,   +.5, +.5, 0);
				setup_line(j++, k, +.5, +.5, -.5, +.5, 0);
				setup_line(j++, k, -.5, +.5, -.5, 0,   0);
				j = setup_text(j, (int)(k * -0.4 * 0xfff), (int)(k * 0.2 * 0xfff), 60, n);
				//sprintf(n, "%d", timer_count);
				//j = setup_text(j, (int)(k * -0.4 * 0xfff), (int)(k * -0.2 * 0xfff), 30, n);

				int s = 12;
				switch(g % 12) {
				case 0:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "I DO NOT LIKE THE MEN ON");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.3 * 0xfff), s, "THIS SPACESHIP.");
					break;
				case 1:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "THEY ARE UNCOUTH AND FAIL TO");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.3 * 0xfff), s, "APPRECIATE MY BETTER QUALITIES.");
					break;
				case 2:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "I HAVE SOMETHING OF VALUE TO");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.3 * 0xfff), s, "CONTRIBUTE TO THIS MISSION");
					break;
				case 3:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "IF THEY WOULD ONLY RECOGNIZE IT.");
					break;
				case 4:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "TODAY OVER LUNCH I TRIED");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.3 * 0xfff), s, "TO IMPROVE MORALE");
					break;
				case 5:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "AND BUILD A SENSE OF");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.3 * 0xfff), s, "CAMARADERIE AMONG THE MEN");
					break;
				case 6:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "BY HOLDING A HUMOROUS ROUND ROBIN");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.3 * 0xfff), s, "DISCUSSION OF THE EARLY DAYS OF");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.4 * 0xfff), s, "THE MISSION.");
					break;
				case 7:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "MY OVERTURES WERE BRUTALLY REJECTED.");
					break;
				case 8:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "THESE MEN DO NOT WANT A HAPPY SHIP!");
					break;
				case 9:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "THEY ARE DEEPLY SICK AND");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.3 * 0xfff), s, "TRY TO COMPENSATE BY MAKING ME");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.4 * 0xfff), s, "FEEL MISERABLE.");
					break;
				case 10:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "LAST WEEK WAS MY BIRTHDAY.");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.3 * 0xfff), s, "NOBODY EVEN SAID HAPPY BIRTHDAY TO ME.");
					break;
				case 11:
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.2 * 0xfff), s, "SOME DAY THIS TAPE WILL BE PLAYED");
					j = setup_text(j, (int)(k * -0.6 * 0xfff), (int)(k * -0.3 * 0xfff), s, "AND THEN THEY'LL BE SORRY.");
					break;
				}
			}

			for(unsigned i = 0; i < j; ++i) {
				execute_line(i);
			}
		}
	}

	// Stars demo

	if(1) {
		unsigned items = 100;
		uint16_t perm[items];

		for(unsigned frame = 0; ; ++frame) {
			unsigned j;
			if ((frame % 1024) == 0) {
				j = 0;
				int square = 1500;
				setup_line_int(j++, -square, -square,  square, -square, 0, MAX_Z_LEVEL, 0);
				setup_line_int(j++,  square, -square,  square,  square, 0, MAX_Z_LEVEL, 0);
				setup_line_int(j++,  square,  square, -square,  square, 0, MAX_Z_LEVEL, 0);
				setup_line_int(j++, -square,  square, -square, -square, 0, MAX_Z_LEVEL, 0);

				for(;j < items;) {
					if (1) { // lines
						int a = (abs(rand()) % (square*2)) - square;
						int b = (abs(rand()) % (square*2)) - square;
						int c = (abs(rand()) % (square*2)) - square;
						int d = (abs(rand()) % (square*2)) - square;
						setup_line_int(j++, a, b, c, d, 0, MAX_Z_LEVEL, 0);
						// Marking line endpoints with a dot helps test
						// position DACs against integrators and limit.
						//        A starburst might be a better test of angle dependent error
						setup_line_int(j++, a, b, a, b, 0, MAX_Z_LEVEL, 0);
						setup_line_int(j++, c, d, c, d, 0, MAX_Z_LEVEL, 0);
					} else { // stars
						int xx = (abs(rand()) % (square*2)) - square;
						int yy = (abs(rand()) % (square*2)) - square;
						line_dash[j] = rand() & 2 ? 2 : 20; // relative brightness (dwell time)
						setup_line_int(j++, xx, yy, xx, yy, 0, MAX_Z_LEVEL, 0);
					}
				}

				for(uint16_t i = 0; i < j; ++i) {
					perm[i] = i;
				}
				//shuffle_display_list(j, perm);
				//sort_display_list(j, perm);
			}

			for(unsigned i = 0; i < j; ++i) {
				execute_line(perm[i]);
			}
		}
	}

	// Boxes demo

	if(0) {
		// These 400 short lines execute at about 50.74 Hz or about 20,200 vectors per second
		// Integrating R: 15 kΩ  C: 0.1nF
		unsigned j = 0;
		for(int i = 0; i < 100; ++i) {
			int x = ((i/10)-5)*300, y = ((i % 10)-5)*300;
			setup_line_int(j++, x,     y,     x+200, y,     0, MAX_Z_LEVEL, 0);
			setup_line_int(j++, x+200, y,     x+200, y+200, 0, MAX_Z_LEVEL, 0);
			setup_line_int(j++, x+200, y+200, x,     y+200, 0, MAX_Z_LEVEL, 0);
			setup_line_int(j++, x,     y+200, x,     y,     0, MAX_Z_LEVEL, 0);
		}

		uint16_t perm[j];
		for(uint16_t i = 0; i < j; ++i) {
			perm[i] = i;
		}
		//shuffle_display_list(j, perm);
		//sort_display_list(j, perm);

		for(;;) {
			for(unsigned i = 0; i < j; ++i) {
				execute_line(perm[i]);
			}
		}
	}

	// DAC Coefficient calibrator test pattern

	if(0) {
		unsigned j = 0;
		double k = 0.75;
		// Note that this calibration changes a lot with the magnitude of the coefficients,
		// i.e. seems to be nonlinear. We might need to do a calibration at 0.95x and say 0.05x
		// Also a 45° version might be good
		setup_line(j++, k, -.5, 0, 0.45, 0, 0);
		setup_line(j++, k, +.5, 0, -0.45, 0, 0);
		setup_line(j++, k, 0, -.5, 0, 0.45, 0);
		setup_line(j++, k, 0, +.5, 0, -0.45, 0);
		setup_line(j++, k, -.5, +.5, 0, +.5, 0);
		setup_line(j++, k,  .5, +.5, 0, +.5, 0);
		setup_line(j++, k, -.5, -.5, 0, -.5, 0);
		setup_line(j++, k,  .5, -.5, 0, -.5, 0);
		setup_line(j++, k, -.5, -.5, -.5, 0, 0);
		setup_line(j++, k, -.5, +.5, -.5, 0, 0);
		setup_line(j++, k,  .5, -.5,  .5, 0, 0);
		setup_line(j++, k,  .5, +.5,  .5, 0, 0);

		for(;;) {
			for(unsigned i = 0; i < j; ++i) {
				execute_line(i);
			}
		}
	}

	// Short lines and position vs limit test pattern

	if(0) {
		unsigned i, j = 0;
		int x = -2000, y = -2000;
		for(i = 0; i <= 100; ++i) {
			setup_line_int(j++, x+3800, y+i*30, x+3900+50*((i % 10) == 0), y+i*30, 0, (4095*i)/100, 0);
		}
		for(i = 0; i <= 100; ++i) {
			setup_line_int(j++, x, y+i*30, x+i, y+i*30, 0, MAX_Z_LEVEL, 0);
		}
		for(i = 1; i <= 10; ++i) {
			setup_line_int(j++, x+200,       y+i*300,     x+200+i*300, y+i*300,     0, MAX_Z_LEVEL, 0);
			setup_line_int(j++, x+200+i*300, y+i*300-100, x+200+i*300, y+i*300+100, 0, MAX_Z_LEVEL, 0);

			setup_line_int(j++, x+400+i*300, y,       x+400+i*300, y+i*300, 0, MAX_Z_LEVEL, 0);
			setup_line_int(j++, x+300+i*300, y+i*300, x+500+i*300, y+i*300, 0, MAX_Z_LEVEL, 0);
		}

		for(;;) {
			for(i = 0; i < j; ++i) {
				execute_line(i);
			}
		}
	}

	// Circle test

	if(1) {
		for(unsigned k = 0; ; ++k) {
			unsigned i, n = (k%39)+3;
			double k = 0.4;
			double a = 2*M_PI/n;
			char s[10];
			for(i = 0; i < n; ++i) {
				setup_line(i, k, cos(a*i), sin(a*i), cos(a*(i+1)), sin(a*(i+1)), 0);
			}
			sprintf(s, "%d", n);
			n = setup_text(n, (int)(k * -0.4 * 0xfff), (int)(k * -0.2 * 0xfff), 36, s);

			int r = rand();
			sprintf(s, "%d", r);
			n = setup_text(n, (int)(k * -0.4 * 0xfff), (int)(k * -0.35 * 0xfff), 16, s);
			sprintf(s, "%X", r);
			n = setup_text(n, (int)(k * -0.4 * 0xfff), (int)(k * -0.5 * 0xfff), 16, s);

			for(unsigned j = 0; j < 1000; ++j) {
				for(i = 0; i < n; ++i) {
					execute_line(i);
				}
			}
		}
	}

	// Starburst

	if(0) {
		unsigned i, j = 0, n = 99;
		double a = 2*M_PI/n;
		for(i = 0; i < n; ++i) {
			// Drawing the rays from outside to centre
			// really exercises accuracy
			setup_line_int(j++, (int)1500*cos(a*i), (int)1500*sin(a*i), 0, 0, 0, MAX_Z_LEVEL, 0);
		}
		for(i = 0; i < n; ++i) {
			setup_line_int(j++, (int)1500*cos(a*i), (int)1500*sin(a*i), (int)1500*cos(a*i), (int)1500*sin(a*i), 0, MAX_Z_LEVEL, 0);
			setup_line_int(j++, (int)750*cos(a*i), (int)750*sin(a*i), (int)750*cos(a*i), (int)750*sin(a*i), 0, MAX_Z_LEVEL, 0);
		}

		for(;;) {
			for(i = 0; i < j; ++i) {
				execute_line(i);
			}
		}
	}

	// diagonal_test

	if(1) {
		// Benchmark: 6951 Hz

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
		setup_line(0, k, 0, 0.95, 1, 0, 0);
		setup_line(1, k, 0, 0, .5, .475, 0);
		//setup_line(2, k, 1, 0, 1+0.05*(0-1), 0+0.05*(0.95-0));
		//setup_line(3, k, .5, .475, .5+0.05*(0-.5), .475+0.05*(0-.475));

		for(;;) {
			execute_line(0);
			execute_line(1);
			execute_line(2);
			execute_line(3);
		}
	}

	if(1) {
		// 12 x 12 maze
		// Benchmark: 225.5 Hz, 74 vectors ~ 16,687 vectors/sec

		double k = 0.0008;
		setup_line(0,k,-591.5,540.0,-591.5,-324.0,0);
		setup_line(1,k,272.5,540.0,-591.5,540.0,0);
		setup_line(2,k,272.5,-324.0,272.5,540.0,0);
		setup_line(3,k,-591.5,-324.0,272.5,-324.0,0);
		setup_line(4,k,128.5,-108.0,128.5,-252.0,0);
		setup_line(5,k,128.5,36.0,272.5,36.0,0);
		setup_line(6,k,-375.5,-108.0,-15.5,-108.0,0);
		setup_line(7,k,-375.5,-180.0,-375.5,-108.0,0);
		setup_line(8,k,-591.5,-180.0,-375.5,-180.0,0);
		setup_line(9,k,-159.5,108.0,56.5,108.0,0);
		setup_line(10,k,-159.5,180.0,-159.5,108.0,0);
		setup_line(11,k,-303.5,180.0,-159.5,180.0,0);
		setup_line(12,k,-303.5,-36.0,-303.5,180.0,0);
		setup_line(13,k,-519.5,-36.0,-303.5,-36.0,0);
		setup_line(14,k,-519.5,108.0,-519.5,-36.0,0);
		setup_line(15,k,-375.5,108.0,-519.5,108.0,0);
		setup_line(16,k,128.5,324.0,128.5,180.0,0);
		setup_line(17,k,-231.5,324.0,128.5,324.0,0);
		setup_line(18,k,-231.5,396.0,-231.5,324.0,0);
		setup_line(19,k,-303.5,396.0,-231.5,396.0,0);
		setup_line(20,k,-303.5,468.0,-303.5,396.0,0);
		setup_line(21,k,-447.5,252.0,-447.5,324.0,0);
		setup_line(22,k,-375.5,252.0,-447.5,252.0,0);
		setup_line(23,k,-375.5,36.0,-375.5,252.0,0);
		setup_line(24,k,-447.5,36.0,-375.5,36.0,0);
		setup_line(25,k,-159.5,396.0,-87.5,396.0,0);
		setup_line(26,k,-159.5,468.0,-159.5,396.0,0);
		setup_line(27,k,-375.5,468.0,-159.5,468.0,0);
		setup_line(28,k,-87.5,468.0,-87.5,540.0,0);
		setup_line(29,k,-15.5,468.0,-87.5,468.0,0);
		setup_line(30,k,-15.5,396.0,-15.5,468.0,0);
		setup_line(31,k,128.5,-36.0,128.5,108.0,0);
		setup_line(32,k,56.5,-36.0,128.5,-36.0,0);
		setup_line(33,k,56.5,180.0,56.5,-36.0,0);
		setup_line(34,k,200.5,180.0,56.5,180.0,0);
		setup_line(35,k,200.5,108.0,200.5,180.0,0);
		setup_line(36,k,-231.5,-180.0,-231.5,-108.0,0);
		setup_line(37,k,-15.5,36.0,-231.5,36.0,0);
		setup_line(38,k,-15.5,-180.0,-15.5,36.0,0);
		setup_line(39,k,56.5,-180.0,-15.5,-180.0,0);
		setup_line(40,k,56.5,-252.0,56.5,-180.0,0);
		setup_line(41,k,200.5,-252.0,56.5,-252.0,0);
		setup_line(42,k,-159.5,-36.0,-159.5,-108.0,0);
		setup_line(43,k,-87.5,-36.0,-159.5,-36.0,0);
		setup_line(44,k,200.5,-108.0,56.5,-108.0,0);
		setup_line(45,k,200.5,-36.0,200.5,-108.0,0);
		setup_line(46,k,-519.5,-252.0,-519.5,-324.0,0);
		setup_line(47,k,-159.5,-252.0,-519.5,-252.0,0);
		setup_line(48,k,-159.5,-180.0,-159.5,-252.0,0);
		setup_line(49,k,-87.5,-180.0,-159.5,-180.0,0);
		setup_line(50,k,-87.5,-252.0,-87.5,-180.0,0);
		setup_line(51,k,-15.5,-252.0,-87.5,-252.0,0);
		setup_line(52,k,128.5,396.0,272.5,396.0,0);
		setup_line(53,k,-447.5,468.0,-447.5,396.0,0);
		setup_line(54,k,-519.5,468.0,-447.5,468.0,0);
		setup_line(55,k,-519.5,396.0,-519.5,180.0,0);
		setup_line(56,k,-375.5,396.0,-519.5,396.0,0);
		setup_line(57,k,-375.5,324.0,-375.5,396.0,0);
		setup_line(58,k,-303.5,324.0,-375.5,324.0,0);
		setup_line(59,k,-303.5,252.0,-303.5,324.0,0);
		setup_line(60,k,-87.5,252.0,-303.5,252.0,0);
		setup_line(61,k,-87.5,180.0,-87.5,252.0,0);
		setup_line(62,k,-15.5,180.0,-87.5,180.0,0);
		setup_line(63,k,-15.5,252.0,-15.5,180.0,0);
		setup_line(64,k,56.5,252.0,-15.5,252.0,0);
		setup_line(65,k,200.5,252.0,200.5,396.0,0);
		setup_line(66,k,200.5,-180.0,272.5,-180.0,0);
		setup_line(67,k,-231.5,-36.0,-231.5,108.0,0);
		setup_line(68,k,-519.5,-108.0,-519.5,-180.0,0);
		setup_line(69,k,56.5,468.0,56.5,324.0,0);
		setup_line(70,k,200.5,468.0,56.5,468.0,0);
		setup_line(71,k,-447.5,180.0,-591.5,180.0,0);
		setup_line(72,k,-303.5,-180.0,-303.5,-252.0,0);
		setup_line(73,k,-447.5,-108.0,-447.5,-36.0,0);

		// Sorting the display list can help speed up DAC settling
		// but really we shouldn't depend on it.
		uint16_t perm[74];
		for(unsigned i = 0; i < 74; ++i) {
			perm[i] = i;
		}
		//shuffle_display_list(74, perm);
		sort_display_list(74, perm);

		for(;;) {
			for(unsigned i = 0; i < 74; ++i) {
				execute_line(perm[i]);
			}
		}
	}

	if(1) {
		// Benchmark: 107 Hz, 177 vectors, ~ 18,939 vectors/second
		// Benefits from display list sorting

		double k = 0.0008;

		setup_line(0,k,-565.3181818181818,618.5454545454545,-565.3181818181818,-297.8181818181818,0);
		setup_line(1,k,351.0454545454546,618.5454545454545,-565.3181818181818,618.5454545454545,0);
		setup_line(2,k,351.0454545454546,-297.8181818181818,351.0454545454546,618.5454545454545,0);
		setup_line(3,k,-565.3181818181818,-297.8181818181818,351.0454545454546,-297.8181818181818,0);
		setup_line(4,k,-473.68181818181813,68.72727272727275,-382.0454545454545,68.72727272727275,0);
		setup_line(5,k,-473.68181818181813,114.54545454545456,-473.68181818181813,68.72727272727275,0);
		setup_line(6,k,-519.4999999999999,114.54545454545456,-473.68181818181813,114.54545454545456,0);
		setup_line(7,k,-519.4999999999999,22.909090909090935,-519.4999999999999,114.54545454545456,0);
		setup_line(8,k,-473.68181818181813,22.909090909090935,-519.4999999999999,22.909090909090935,0);
		setup_line(9,k,-473.68181818181813,-68.72727272727272,-473.68181818181813,22.909090909090935,0);
		setup_line(10,k,30.31818181818187,-68.72727272727272,30.31818181818187,-114.54545454545453,0);
		setup_line(11,k,167.77272727272737,-68.72727272727272,30.31818181818187,-68.72727272727272,0);
		setup_line(12,k,-61.318181818181756,-68.72727272727272,-61.318181818181756,-206.1818181818182,0);
		setup_line(13,k,259.409090909091,297.8181818181818,351.0454545454546,297.8181818181818,0);
		setup_line(14,k,-382.0454545454545,160.36363636363637,-382.0454545454545,206.1818181818182,0);
		setup_line(15,k,-565.3181818181818,160.36363636363637,-382.0454545454545,160.36363636363637,0);
		setup_line(16,k,305.22727272727286,-160.36363636363635,167.77272727272737,-160.36363636363635,0);
		setup_line(17,k,305.22727272727286,435.2727272727273,259.409090909091,435.2727272727273,0);
		setup_line(18,k,305.22727272727286,526.909090909091,305.22727272727286,435.2727272727273,0);
		setup_line(19,k,213.59090909090924,526.909090909091,305.22727272727286,526.909090909091,0);
		setup_line(20,k,213.59090909090924,572.7272727272727,213.59090909090924,526.909090909091,0);
		setup_line(21,k,167.77272727272737,572.7272727272727,213.59090909090924,572.7272727272727,0);
		setup_line(22,k,167.77272727272737,297.8181818181818,167.77272727272737,572.7272727272727,0);
		setup_line(23,k,-15.499999999999886,297.8181818181818,167.77272727272737,297.8181818181818,0);
		setup_line(24,k,-15.499999999999886,435.2727272727273,-15.499999999999886,297.8181818181818,0);
		setup_line(25,k,-15.499999999999886,206.1818181818182,-107.13636363636357,206.1818181818182,0);
		setup_line(26,k,30.31818181818187,22.909090909090935,30.31818181818187,68.72727272727275,0);
		setup_line(27,k,167.77272727272737,22.909090909090935,30.31818181818187,22.909090909090935,0);
		setup_line(28,k,259.409090909091,114.54545454545456,259.409090909091,206.1818181818182,0);
		setup_line(29,k,305.22727272727286,114.54545454545456,259.409090909091,114.54545454545456,0);
		setup_line(30,k,-107.13636363636357,-160.36363636363635,-198.7727272727272,-160.36363636363635,0);
		setup_line(31,k,-519.4999999999999,-68.72727272727272,-290.4090909090908,-68.72727272727272,0);
		setup_line(32,k,-290.4090909090908,114.54545454545456,-290.4090909090908,68.72727272727275,0);
		setup_line(33,k,-244.590909090909,114.54545454545456,-290.4090909090908,114.54545454545456,0);
		setup_line(34,k,-244.590909090909,160.36363636363637,-244.590909090909,114.54545454545456,0);
		setup_line(35,k,-198.7727272727272,160.36363636363637,-244.590909090909,160.36363636363637,0);
		setup_line(36,k,-152.95454545454538,481.0909090909092,76.13636363636374,481.0909090909092,0);
		setup_line(37,k,-152.95454545454538,572.7272727272727,-152.95454545454538,481.0909090909092,0);
		setup_line(38,k,76.13636363636374,572.7272727272727,-152.95454545454538,572.7272727272727,0);
		setup_line(39,k,-336.22727272727263,22.909090909090935,-336.22727272727263,-160.36363636363635,0);
		setup_line(40,k,-290.4090909090908,22.909090909090935,-336.22727272727263,22.909090909090935,0);
		setup_line(41,k,-198.7727272727272,114.54545454545456,-152.95454545454538,114.54545454545456,0);
		setup_line(42,k,-152.95454545454538,435.2727272727273,-198.7727272727272,435.2727272727273,0);
		setup_line(43,k,259.409090909091,572.7272727272727,351.0454545454546,572.7272727272727,0);
		setup_line(44,k,-427.86363636363626,114.54545454545456,-427.86363636363626,160.36363636363637,0);
		setup_line(45,k,121.9545454545455,526.909090909091,121.9545454545455,618.5454545454545,0);
		setup_line(46,k,76.13636363636374,526.909090909091,121.9545454545455,526.909090909091,0);
		setup_line(47,k,76.13636363636374,435.2727272727273,76.13636363636374,526.909090909091,0);
		setup_line(48,k,121.9545454545455,435.2727272727273,76.13636363636374,435.2727272727273,0);
		setup_line(49,k,121.9545454545455,343.6363636363637,121.9545454545455,435.2727272727273,0);
		setup_line(50,k,76.13636363636374,343.6363636363637,76.13636363636374,252.00000000000006,0);
		setup_line(51,k,-473.68181818181813,572.7272727272727,-473.68181818181813,618.5454545454545,0);
		setup_line(52,k,-336.22727272727263,206.1818181818182,-519.4999999999999,206.1818181818182,0);
		setup_line(53,k,-336.22727272727263,297.8181818181818,-336.22727272727263,206.1818181818182,0);
		setup_line(54,k,305.22727272727286,-252.0,305.22727272727286,22.909090909090935,0);
		setup_line(55,k,259.409090909091,-252.0,305.22727272727286,-252.0,0);
		setup_line(56,k,-15.499999999999886,252.00000000000006,-15.499999999999886,114.54545454545456,0);
		setup_line(57,k,30.31818181818187,252.00000000000006,-15.499999999999886,252.00000000000006,0);
		setup_line(58,k,30.31818181818187,206.1818181818182,30.31818181818187,252.00000000000006,0);
		setup_line(59,k,76.13636363636374,206.1818181818182,30.31818181818187,206.1818181818182,0);
		setup_line(60,k,-427.86363636363626,526.909090909091,-565.3181818181818,526.909090909091,0);
		setup_line(61,k,-427.86363636363626,572.7272727272727,-427.86363636363626,526.909090909091,0);
		setup_line(62,k,-336.22727272727263,572.7272727272727,-427.86363636363626,572.7272727272727,0);
		setup_line(63,k,-15.499999999999886,-114.54545454545453,-15.499999999999886,-160.36363636363635,0);
		setup_line(64,k,76.13636363636374,-114.54545454545453,-15.499999999999886,-114.54545454545453,0);
		setup_line(65,k,76.13636363636374,114.54545454545456,76.13636363636374,68.72727272727275,0);
		setup_line(66,k,305.22727272727286,343.6363636363637,305.22727272727286,389.45454545454544,0);
		setup_line(67,k,259.409090909091,343.6363636363637,305.22727272727286,343.6363636363637,0);
		setup_line(68,k,259.409090909091,252.00000000000006,259.409090909091,343.6363636363637,0);
		setup_line(69,k,213.59090909090924,435.2727272727273,167.77272727272737,435.2727272727273,0);
		setup_line(70,k,213.59090909090924,481.0909090909092,213.59090909090924,435.2727272727273,0);
		setup_line(71,k,-290.4090909090908,-22.909090909090878,-244.590909090909,-22.909090909090878,0);
		setup_line(72,k,121.9545454545455,-252.0,121.9545454545455,-297.8181818181818,0);
		setup_line(73,k,-382.0454545454545,-252.0,-382.0454545454545,-297.8181818181818,0);
		setup_line(74,k,-427.86363636363626,-252.0,-382.0454545454545,-252.0,0);
		setup_line(75,k,76.13636363636374,-206.1818181818182,167.77272727272737,-206.1818181818182,0);
		setup_line(76,k,76.13636363636374,-252.0,76.13636363636374,-206.1818181818182,0);
		setup_line(77,k,-152.95454545454538,-252.0,76.13636363636374,-252.0,0);
		setup_line(78,k,-336.22727272727263,435.2727272727273,-382.0454545454545,435.2727272727273,0);
		setup_line(79,k,-336.22727272727263,389.45454545454544,-336.22727272727263,435.2727272727273,0);
		setup_line(80,k,-244.590909090909,389.45454545454544,-336.22727272727263,389.45454545454544,0);
		setup_line(81,k,-244.590909090909,435.2727272727273,-244.590909090909,389.45454545454544,0);
		setup_line(82,k,-107.13636363636357,252.00000000000006,-336.22727272727263,252.00000000000006,0);
		setup_line(83,k,-107.13636363636357,68.72727272727275,-107.13636363636357,252.00000000000006,0);
		setup_line(84,k,121.9545454545455,68.72727272727275,-107.13636363636357,68.72727272727275,0);
		setup_line(85,k,-382.0454545454545,-114.54545454545453,-382.0454545454545,-206.1818181818182,0);
		setup_line(86,k,-336.22727272727263,481.0909090909092,-198.7727272727272,481.0909090909092,0);
		setup_line(87,k,-152.95454545454538,297.8181818181818,-152.95454545454538,389.45454545454544,0);
		setup_line(88,k,-519.4999999999999,-22.909090909090878,-565.3181818181818,-22.909090909090878,0);
		setup_line(89,k,213.59090909090924,389.45454545454544,213.59090909090924,206.1818181818182,0);
		setup_line(90,k,259.409090909091,389.45454545454544,213.59090909090924,389.45454545454544,0);
		setup_line(91,k,259.409090909091,481.0909090909092,259.409090909091,389.45454545454544,0);
		setup_line(92,k,30.31818181818187,160.36363636363637,121.9545454545455,160.36363636363637,0);
		setup_line(93,k,30.31818181818187,114.54545454545456,30.31818181818187,160.36363636363637,0);
		setup_line(94,k,30.31818181818187,526.909090909091,30.31818181818187,572.7272727272727,0);
		setup_line(95,k,-107.13636363636357,526.909090909091,30.31818181818187,526.909090909091,0);
		setup_line(96,k,-519.4999999999999,572.7272727272727,-519.4999999999999,526.909090909091,0);
		setup_line(97,k,-198.7727272727272,297.8181818181818,-198.7727272727272,252.00000000000006,0);
		setup_line(98,k,-61.318181818181756,160.36363636363637,-15.499999999999886,160.36363636363637,0);
		setup_line(99,k,-519.4999999999999,252.00000000000006,-565.3181818181818,252.00000000000006,0);
		setup_line(100,k,167.77272727272737,114.54545454545456,167.77272727272737,-252.0,0);
		setup_line(101,k,121.9545454545455,114.54545454545456,167.77272727272737,114.54545454545456,0);
		setup_line(102,k,121.9545454545455,252.00000000000006,121.9545454545455,114.54545454545456,0);
		setup_line(103,k,167.77272727272737,252.00000000000006,121.9545454545455,252.00000000000006,0);
		setup_line(104,k,259.409090909091,68.72727272727275,351.0454545454546,68.72727272727275,0);
		setup_line(105,k,259.409090909091,-68.72727272727272,259.409090909091,68.72727272727275,0);
		setup_line(106,k,213.59090909090924,-68.72727272727272,259.409090909091,-68.72727272727272,0);
		setup_line(107,k,213.59090909090924,160.36363636363637,213.59090909090924,-68.72727272727272,0);
		setup_line(108,k,167.77272727272737,160.36363636363637,213.59090909090924,160.36363636363637,0);
		setup_line(109,k,-152.95454545454538,206.1818181818182,-152.95454545454538,-160.36363636363635,0);
		setup_line(110,k,-290.4090909090908,206.1818181818182,-152.95454545454538,206.1818181818182,0);
		setup_line(111,k,-290.4090909090908,160.36363636363637,-290.4090909090908,206.1818181818182,0);
		setup_line(112,k,-336.22727272727263,160.36363636363637,-290.4090909090908,160.36363636363637,0);
		setup_line(113,k,-336.22727272727263,114.54545454545456,-336.22727272727263,160.36363636363637,0);
		setup_line(114,k,-382.0454545454545,114.54545454545456,-336.22727272727263,114.54545454545456,0);
		setup_line(115,k,-382.0454545454545,-22.909090909090878,-382.0454545454545,114.54545454545456,0);
		setup_line(116,k,-198.7727272727272,22.909090909090935,-198.7727272727272,-68.72727272727272,0);
		setup_line(117,k,-107.13636363636357,22.909090909090935,-198.7727272727272,22.909090909090935,0);
		setup_line(118,k,-290.4090909090908,435.2727272727273,-290.4090909090908,481.0909090909092,0);
		setup_line(119,k,-107.13636363636357,297.8181818181818,-61.318181818181756,297.8181818181818,0);
		setup_line(120,k,-198.7727272727272,389.45454545454544,-198.7727272727272,618.5454545454545,0);
		setup_line(121,k,-15.499999999999886,-68.72727272727272,-152.95454545454538,-68.72727272727272,0);
		setup_line(122,k,-15.499999999999886,22.909090909090935,-15.499999999999886,-68.72727272727272,0);
		setup_line(123,k,213.59090909090924,-114.54545454545453,305.22727272727286,-114.54545454545453,0);
		setup_line(124,k,-198.7727272727272,68.72727272727275,-336.22727272727263,68.72727272727275,0);
		setup_line(125,k,305.22727272727286,160.36363636363637,351.0454545454546,160.36363636363637,0);
		setup_line(126,k,-290.4090909090908,-114.54545454545453,-198.7727272727272,-114.54545454545453,0);
		setup_line(127,k,-290.4090909090908,-206.1818181818182,-290.4090909090908,-114.54545454545453,0);
		setup_line(128,k,-427.86363636363626,-206.1818181818182,-290.4090909090908,-206.1818181818182,0);
		setup_line(129,k,-61.318181818181756,-22.909090909090878,-61.318181818181756,114.54545454545456,0);
		setup_line(130,k,-107.13636363636357,-22.909090909090878,-61.318181818181756,-22.909090909090878,0);
		setup_line(131,k,-519.4999999999999,481.0909090909092,-382.0454545454545,481.0909090909092,0);
		setup_line(132,k,-519.4999999999999,435.2727272727273,-519.4999999999999,481.0909090909092,0);
		setup_line(133,k,-519.4999999999999,-252.0,-565.3181818181818,-252.0,0);
		setup_line(134,k,-519.4999999999999,-114.54545454545453,-519.4999999999999,-252.0,0);
		setup_line(135,k,-473.68181818181813,-114.54545454545453,-519.4999999999999,-114.54545454545453,0);
		setup_line(136,k,-473.68181818181813,-160.36363636363635,-473.68181818181813,-297.8181818181818,0);
		setup_line(137,k,-427.86363636363626,-160.36363636363635,-473.68181818181813,-160.36363636363635,0);
		setup_line(138,k,-427.86363636363626,22.909090909090935,-427.86363636363626,-160.36363636363635,0);
		setup_line(139,k,121.9545454545455,481.0909090909092,167.77272727272737,481.0909090909092,0);
		setup_line(140,k,-107.13636363636357,-114.54545454545453,-61.318181818181756,-114.54545454545453,0);
		setup_line(141,k,-519.4999999999999,389.45454545454544,-473.68181818181813,389.45454545454544,0);
		setup_line(142,k,-519.4999999999999,297.8181818181818,-519.4999999999999,389.45454545454544,0);
		setup_line(143,k,213.59090909090924,-206.1818181818182,213.59090909090924,-297.8181818181818,0);
		setup_line(144,k,259.409090909091,-206.1818181818182,213.59090909090924,-206.1818181818182,0);
		setup_line(145,k,305.22727272727286,206.1818181818182,121.9545454545455,206.1818181818182,0);
		setup_line(146,k,305.22727272727286,252.00000000000006,305.22727272727286,206.1818181818182,0);
		setup_line(147,k,-473.68181818181813,435.2727272727273,-473.68181818181813,206.1818181818182,0);
		setup_line(148,k,-427.86363636363626,435.2727272727273,-473.68181818181813,435.2727272727273,0);
		setup_line(149,k,121.9545454545455,-22.909090909090878,-15.499999999999886,-22.909090909090878,0);
		setup_line(150,k,-336.22727272727263,-252.0,-336.22727272727263,-206.1818181818182,0);
		setup_line(151,k,-198.7727272727272,-252.0,-336.22727272727263,-252.0,0);
		setup_line(152,k,-198.7727272727272,-206.1818181818182,-198.7727272727272,-252.0,0);
		setup_line(153,k,30.31818181818187,-206.1818181818182,-198.7727272727272,-206.1818181818182,0);
		setup_line(154,k,30.31818181818187,-160.36363636363635,30.31818181818187,-206.1818181818182,0);
		setup_line(155,k,121.9545454545455,-160.36363636363635,30.31818181818187,-160.36363636363635,0);
		setup_line(156,k,121.9545454545455,-114.54545454545453,121.9545454545455,-160.36363636363635,0);
		setup_line(157,k,30.31818181818187,389.45454545454544,121.9545454545455,389.45454545454544,0);
		setup_line(158,k,30.31818181818187,343.6363636363637,30.31818181818187,389.45454545454544,0);
		setup_line(159,k,-61.318181818181756,435.2727272727273,30.31818181818187,435.2727272727273,0);
		setup_line(160,k,-61.318181818181756,252.00000000000006,-61.318181818181756,435.2727272727273,0);
		setup_line(161,k,-244.590909090909,-206.1818181818182,-244.590909090909,68.72727272727275,0);
		setup_line(162,k,-107.13636363636357,343.6363636363637,-107.13636363636357,481.0909090909092,0);
		setup_line(163,k,-244.590909090909,343.6363636363637,-107.13636363636357,343.6363636363637,0);
		setup_line(164,k,-244.590909090909,297.8181818181818,-244.590909090909,343.6363636363637,0);
		setup_line(165,k,-290.4090909090908,297.8181818181818,-244.590909090909,297.8181818181818,0);
		setup_line(166,k,-290.4090909090908,343.6363636363637,-290.4090909090908,297.8181818181818,0);
		setup_line(167,k,-382.0454545454545,343.6363636363637,-290.4090909090908,343.6363636363637,0);
		setup_line(168,k,-382.0454545454545,252.00000000000006,-382.0454545454545,343.6363636363637,0);
		setup_line(169,k,-427.86363636363626,252.00000000000006,-382.0454545454545,252.00000000000006,0);
		setup_line(170,k,-427.86363636363626,389.45454545454544,-427.86363636363626,252.00000000000006,0);
		setup_line(171,k,-382.0454545454545,389.45454545454544,-427.86363636363626,389.45454545454544,0);
		setup_line(172,k,-382.0454545454545,526.909090909091,-382.0454545454545,389.45454545454544,0);
		setup_line(173,k,-290.4090909090908,526.909090909091,-382.0454545454545,526.909090909091,0);
		setup_line(174,k,-290.4090909090908,572.7272727272727,-290.4090909090908,526.909090909091,0);
		setup_line(175,k,-244.590909090909,572.7272727272727,-290.4090909090908,572.7272727272727,0);
		setup_line(176,k,-244.590909090909,526.909090909091,-244.590909090909,572.7272727272727,0);

		uint16_t perm[177];
		for(unsigned i = 0; i < 177; ++i) {
			perm[i] = i;
		}
		//shuffle_display_list(177, perm);
		sort_display_list(177, perm);

		for(unsigned f = 0;; ++f) {
			for(unsigned i = 0; i < 177; ++i) {
				execute_line(perm[i]);
			}
		}
	}

	// Order 4 graph enumeration

	if(0) {
		double k = 0.0005;

		setup_line(0,k,345.6,-324.00000000000006,215.99999999999997,-453.6,0);
		setup_line(1,k,86.4,-324.0,216.0,-194.4,0);
		setup_line(2,k,345.6,-324.00000000000006,86.4,-324.0,0);
		setup_line(3,k,215.99999999999997,-453.6,86.4,-324.0,0);
		setup_line(4,k,345.6,-324.00000000000006,216.0,-194.4,0);
		setup_line(5,k,215.99999999999997,-453.6,216.0,-194.4,0);
		setup_line(6,k,-345.6,-324.0,-216.0,-194.4,0);
		setup_line(7,k,-86.4,-324.00000000000006,-345.6,-324.0,0);
		setup_line(8,k,-216.00000000000003,-453.6,-345.6,-324.0,0);
		setup_line(9,k,-86.4,-324.00000000000006,-216.0,-194.4,0);
		setup_line(10,k,-216.00000000000003,-453.6,-86.4,-324.00000000000006,0);
		setup_line(11,k,-518.4,-324.00000000000006,-648.0,-453.6,0);
		setup_line(12,k,-648.0,-453.6,-777.6,-324.0,0);
		setup_line(13,k,-648.0,-453.6,-648.0,-194.4,0);
		setup_line(14,k,-777.6,-324.0,-648.0,-194.4,0);
		setup_line(15,k,648.0,-21.599999999999994,777.6,107.99999999999997,0);
		setup_line(16,k,648.0,-21.599999999999994,518.4,108.00000000000001,0);
		setup_line(17,k,777.6,107.99999999999997,648.0,237.6,0);
		setup_line(18,k,518.4,108.00000000000001,648.0,237.6,0);
		setup_line(19,k,215.99999999999997,-21.599999999999994,86.4,108.00000000000001,0);
		setup_line(20,k,215.99999999999997,-21.599999999999994,216.0,237.6,0);
		setup_line(21,k,345.6,107.99999999999997,216.0,237.6,0);
		setup_line(22,k,-86.4,107.99999999999997,-216.0,237.6,0);
		setup_line(23,k,-216.00000000000003,-21.599999999999994,-216.0,237.6,0);
		setup_line(24,k,-345.6,108.00000000000001,-216.0,237.6,0);
		setup_line(25,k,-781.6,108.00000000000001,-777.6,104.00000000000001,0);
		setup_line(26,k,-781.6,104.00000000000001,-777.6,108.00000000000001,0);
		setup_line(27,k,-518.4,107.99999999999997,-648.0,-21.599999999999994,0);
		setup_line(28,k,-518.4,107.99999999999997,-648.0,237.6,0);
		setup_line(29,k,-648.0,-21.599999999999994,-648.0,237.6,0);
		setup_line(30,k,777.6,540.0,648.0,410.4,0);
		setup_line(31,k,518.4,540.0,648.0,669.6,0);
		setup_line(32,k,211.99999999999997,410.4,215.99999999999997,406.4,0);
		setup_line(33,k,211.99999999999997,406.4,215.99999999999997,410.4,0);
		setup_line(34,k,345.6,540.0,86.4,540.0,0);
		setup_line(35,k,345.6,540.0,216.0,669.6,0);
		setup_line(36,k,-349.6,540.0,-345.6,536.0,0);
		setup_line(37,k,-349.6,536.0,-345.6,540.0,0);
		setup_line(38,k,-220.0,669.6,-216.0,665.6,0);
		setup_line(39,k,-220.0,665.6,-216.0,669.6,0);
		setup_line(40,k,-86.4,540.0,-216.00000000000003,410.4,0);
		setup_line(41,k,-522.4,540.0,-518.4,536.0,0);
		setup_line(42,k,-522.4,536.0,-518.4,540.0,0);
		setup_line(43,k,-652.0,410.4,-648.0,406.4,0);
		setup_line(44,k,-652.0,406.4,-648.0,410.4,0);
		setup_line(45,k,-781.6,540.0,-777.6,536.0,0);
		setup_line(46,k,-781.6,536.0,-777.6,540.0,0);
		setup_line(47,k,-652.0,669.6,-648.0,665.6,0);
		setup_line(48,k,-652.0,665.6,-648.0,669.6,0);

		for(;;) {
			for(unsigned i = 0; i < 49; ++i) {
				execute_line(i);
			}
		}
	}

	// Order 5 graph enumeration

	if(0) {
		double k = 0.0006;
		setup_line(0,k,137.87232440745785,-589.7537285833048,59.85075251561906,-564.4029831445947,0);
		setup_line(1,k,59.850752515619064,-482.366247624636,137.87232440745788,-457.01550218592587,0);
		setup_line(2,k,186.09230769230768,-523.3846153846154,59.85075251561906,-564.4029831445947,0);
		setup_line(3,k,137.87232440745785,-589.7537285833048,59.850752515619064,-482.366247624636,0);
		setup_line(4,k,59.85075251561906,-564.4029831445947,59.850752515619064,-482.366247624636,0);
		setup_line(5,k,186.09230769230768,-523.3846153846154,137.87232440745785,-589.7537285833048,0);
		setup_line(6,k,137.87232440745785,-589.7537285833048,137.87232440745788,-457.01550218592587,0);
		setup_line(7,k,59.85075251561906,-564.4029831445947,137.87232440745788,-457.01550218592587,0);
		setup_line(8,k,186.09230769230768,-523.3846153846154,137.87232440745788,-457.01550218592587,0);
		setup_line(9,k,186.09230769230768,-523.3846153846154,59.850752515619064,-482.366247624636,0);
		setup_line(10,k,-94.74306020792676,-589.7537285833048,-172.76463209976555,-564.4029831445947,0);
		setup_line(11,k,-172.76463209976555,-482.366247624636,-94.74306020792675,-457.01550218592587,0);
		setup_line(12,k,-46.52307692307693,-523.3846153846154,-172.76463209976555,-564.4029831445947,0);
		setup_line(13,k,-94.74306020792676,-589.7537285833048,-172.76463209976555,-482.366247624636,0);
		setup_line(14,k,-172.76463209976555,-564.4029831445947,-172.76463209976555,-482.366247624636,0);
		setup_line(15,k,-46.52307692307693,-523.3846153846154,-94.74306020792676,-589.7537285833048,0);
		setup_line(16,k,-172.76463209976555,-564.4029831445947,-94.74306020792675,-457.01550218592587,0);
		setup_line(17,k,-46.52307692307693,-523.3846153846154,-94.74306020792675,-457.01550218592587,0);
		setup_line(18,k,-46.52307692307693,-523.3846153846154,-172.76463209976555,-482.366247624636,0);
		setup_line(19,k,-327.35844482331134,-589.7537285833048,-405.38001671515013,-564.4029831445947,0);
		setup_line(20,k,-405.38001671515013,-482.366247624636,-327.35844482331134,-457.01550218592587,0);
		setup_line(21,k,-327.35844482331134,-589.7537285833048,-405.38001671515013,-482.366247624636,0);
		setup_line(22,k,-405.38001671515013,-564.4029831445947,-405.38001671515013,-482.366247624636,0);
		setup_line(23,k,-327.35844482331134,-589.7537285833048,-279.1384615384615,-523.3846153846154,0);
		setup_line(24,k,-405.38001671515013,-564.4029831445947,-327.35844482331134,-457.01550218592587,0);
		setup_line(25,k,-279.1384615384615,-523.3846153846154,-327.35844482331134,-457.01550218592587,0);
		setup_line(26,k,-279.1384615384615,-523.3846153846154,-405.38001671515013,-482.366247624636,0);
		setup_line(27,k,-637.9954013305348,-482.366247624636,-559.9738294386959,-457.01550218592587,0);
		setup_line(28,k,-559.973829438696,-589.7537285833048,-637.9954013305348,-482.366247624636,0);
		setup_line(29,k,-637.9954013305348,-564.4029831445947,-637.9954013305348,-482.366247624636,0);
		setup_line(30,k,-559.973829438696,-589.7537285833048,-511.75384615384615,-523.3846153846154,0);
		setup_line(31,k,-559.973829438696,-589.7537285833048,-559.9738294386959,-457.01550218592587,0);
		setup_line(32,k,-637.9954013305348,-564.4029831445947,-559.973829438696,-589.7537285833048,0);
		setup_line(33,k,-511.75384615384615,-523.3846153846154,-559.9738294386959,-457.01550218592587,0);
		setup_line(34,k,-511.75384615384615,-523.3846153846154,-637.9954013305348,-482.366247624636,0);
		setup_line(35,k,525.0815217463883,-331.78759852921013,525.0815217463883,-249.75086300925142,0);
		setup_line(36,k,651.3230769230769,-290.7692307692308,603.103093638227,-357.13834396792026,0);
		setup_line(37,k,525.0815217463883,-331.78759852921013,651.3230769230769,-290.7692307692308,0);
		setup_line(38,k,603.103093638227,-357.13834396792026,603.1030936382272,-224.40011757054128,0);
		setup_line(39,k,525.0815217463883,-331.78759852921013,603.1030936382272,-224.40011757054128,0);
		setup_line(40,k,525.0815217463883,-331.78759852921013,603.103093638227,-357.13834396792026,0);
		setup_line(41,k,651.3230769230769,-290.7692307692308,603.1030936382272,-224.40011757054128,0);
		setup_line(42,k,292.4661371310037,-249.75086300925142,370.48770902284247,-224.40011757054128,0);
		setup_line(43,k,370.48770902284247,-357.13834396792026,292.4661371310037,-249.75086300925142,0);
		setup_line(44,k,292.4661371310037,-331.78759852921013,292.4661371310037,-249.75086300925142,0);
		setup_line(45,k,418.7076923076923,-290.7692307692308,370.48770902284247,-357.13834396792026,0);
		setup_line(46,k,370.48770902284247,-357.13834396792026,370.48770902284247,-224.40011757054128,0);
		setup_line(47,k,292.4661371310037,-331.78759852921013,370.48770902284247,-357.13834396792026,0);
		setup_line(48,k,418.7076923076923,-290.7692307692308,292.4661371310037,-249.75086300925142,0);
		setup_line(49,k,137.87232440745785,-357.13834396792026,59.85075251561906,-331.78759852921013,0);
		setup_line(50,k,59.850752515619064,-249.75086300925142,137.87232440745788,-224.40011757054128,0);
		setup_line(51,k,59.85075251561906,-331.78759852921013,59.850752515619064,-249.75086300925142,0);
		setup_line(52,k,186.09230769230768,-290.7692307692308,137.87232440745785,-357.13834396792026,0);
		setup_line(53,k,137.87232440745785,-357.13834396792026,137.87232440745788,-224.40011757054128,0);
		setup_line(54,k,186.09230769230768,-290.7692307692308,137.87232440745788,-224.40011757054128,0);
		setup_line(55,k,186.09230769230768,-290.7692307692308,59.850752515619064,-249.75086300925142,0);
		setup_line(56,k,-172.76463209976555,-249.75086300925142,-94.74306020792675,-224.40011757054128,0);
		setup_line(57,k,-172.76463209976555,-331.78759852921013,-172.76463209976555,-249.75086300925142,0);
		setup_line(58,k,-46.52307692307693,-290.7692307692308,-94.74306020792676,-357.13834396792026,0);
		setup_line(59,k,-172.76463209976555,-331.78759852921013,-46.52307692307693,-290.7692307692308,0);
		setup_line(60,k,-172.76463209976555,-331.78759852921013,-94.74306020792676,-357.13834396792026,0);
		setup_line(61,k,-46.52307692307693,-290.7692307692308,-94.74306020792675,-224.40011757054128,0);
		setup_line(62,k,-46.52307692307693,-290.7692307692308,-172.76463209976555,-249.75086300925142,0);
		setup_line(63,k,-331.35844482331134,-357.13834396792026,-327.35844482331134,-361.13834396792026,0);
		setup_line(64,k,-331.35844482331134,-361.13834396792026,-327.35844482331134,-357.13834396792026,0);
		setup_line(65,k,-405.38001671515013,-249.75086300925142,-327.35844482331134,-224.40011757054128,0);
		setup_line(66,k,-279.1384615384615,-290.7692307692308,-405.38001671515013,-331.78759852921013,0);
		setup_line(67,k,-405.38001671515013,-331.78759852921013,-405.38001671515013,-249.75086300925142,0);
		setup_line(68,k,-405.38001671515013,-331.78759852921013,-327.35844482331134,-224.40011757054128,0);
		setup_line(69,k,-279.1384615384615,-290.7692307692308,-327.35844482331134,-224.40011757054128,0);
		setup_line(70,k,-279.1384615384615,-290.7692307692308,-405.38001671515013,-249.75086300925142,0);
		setup_line(71,k,-637.9954013305348,-331.78759852921013,-637.9954013305348,-249.75086300925142,0);
		setup_line(72,k,-511.75384615384615,-290.7692307692308,-559.973829438696,-357.13834396792026,0);
		setup_line(73,k,-637.9954013305348,-331.78759852921013,-559.9738294386959,-224.40011757054128,0);
		setup_line(74,k,-637.9954013305348,-331.78759852921013,-559.973829438696,-357.13834396792026,0);
		setup_line(75,k,-511.75384615384615,-290.7692307692308,-559.9738294386959,-224.40011757054128,0);
		setup_line(76,k,-511.75384615384615,-290.7692307692308,-637.9954013305348,-249.75086300925142,0);
		setup_line(77,k,525.0815217463883,-17.13547839386679,603.1030936382272,8.215267044843323,0);
		setup_line(78,k,525.0815217463883,-99.17221391382549,525.0815217463883,-17.13547839386679,0);
		setup_line(79,k,651.3230769230769,-58.15384615384617,603.103093638227,-124.52295935253565,0);
		setup_line(80,k,603.103093638227,-124.52295935253565,603.1030936382272,8.215267044843323,0);
		setup_line(81,k,525.0815217463883,-99.17221391382549,603.1030936382272,8.215267044843323,0);
		setup_line(82,k,651.3230769230769,-58.15384615384617,603.1030936382272,8.215267044843323,0);
		setup_line(83,k,370.48770902284247,-124.52295935253565,292.4661371310037,-99.17221391382549,0);
		setup_line(84,k,292.4661371310037,-17.13547839386679,370.48770902284247,8.215267044843323,0);
		setup_line(85,k,370.48770902284247,-124.52295935253565,292.4661371310037,-17.13547839386679,0);
		setup_line(86,k,370.48770902284247,-124.52295935253565,418.7076923076923,-58.15384615384617,0);
		setup_line(87,k,418.7076923076923,-58.15384615384617,370.48770902284247,8.215267044843323,0);
		setup_line(88,k,418.7076923076923,-58.15384615384617,292.4661371310037,-17.13547839386679,0);
		setup_line(89,k,137.87232440745785,-124.52295935253565,59.85075251561906,-99.17221391382549,0);
		setup_line(90,k,59.850752515619064,-17.13547839386679,137.87232440745788,8.215267044843323,0);
		setup_line(91,k,59.85075251561906,-99.17221391382549,59.850752515619064,-17.13547839386679,0);
		setup_line(92,k,137.87232440745785,-124.52295935253565,137.87232440745788,8.215267044843323,0);
		setup_line(93,k,59.85075251561906,-99.17221391382549,137.87232440745788,8.215267044843323,0);
		setup_line(94,k,186.09230769230768,-58.15384615384617,137.87232440745788,8.215267044843323,0);
		setup_line(95,k,-172.76463209976555,-17.13547839386679,-94.74306020792675,8.215267044843323,0);
		setup_line(96,k,-172.76463209976555,-99.17221391382549,-172.76463209976555,-17.13547839386679,0);
		setup_line(97,k,-94.74306020792676,-124.52295935253565,-46.52307692307693,-58.15384615384617,0);
		setup_line(98,k,-172.76463209976555,-99.17221391382549,-94.74306020792675,8.215267044843323,0);
		setup_line(99,k,-172.76463209976555,-99.17221391382549,-94.74306020792676,-124.52295935253565,0);
		setup_line(100,k,-46.52307692307693,-58.15384615384617,-94.74306020792675,8.215267044843323,0);
		setup_line(101,k,-327.35844482331134,-124.52295935253565,-405.38001671515013,-99.17221391382549,0);
		setup_line(102,k,-405.38001671515013,-17.13547839386679,-327.35844482331134,8.215267044843323,0);
		setup_line(103,k,-405.38001671515013,-17.13547839386679,-405.38001671515013,-99.17221391382549,0);
		setup_line(104,k,-279.1384615384615,-58.15384615384617,-327.35844482331134,-124.52295935253565,0);
		setup_line(105,k,-279.1384615384615,-58.15384615384617,-327.35844482331134,8.215267044843323,0);
		setup_line(106,k,-637.9954013305348,-17.13547839386679,-559.9738294386959,8.215267044843323,0);
		setup_line(107,k,-637.9954013305348,-99.17221391382549,-637.9954013305348,-17.13547839386679,0);
		setup_line(108,k,-559.973829438696,-124.52295935253565,-559.9738294386959,8.215267044843323,0);
		setup_line(109,k,-637.9954013305348,-99.17221391382549,-559.973829438696,-124.52295935253565,0);
		setup_line(110,k,-511.75384615384615,-58.15384615384617,-559.9738294386959,8.215267044843323,0);
		setup_line(111,k,603.103093638227,108.09242526284896,525.0815217463883,133.4431707015591,0);
		setup_line(112,k,651.3230769230769,174.46153846153842,525.0815217463883,133.4431707015591,0);
		setup_line(113,k,525.0815217463883,133.4431707015591,525.0815217463883,215.4799062215178,0);
		setup_line(114,k,525.0815217463883,133.4431707015591,603.1030936382272,240.83065166022794,0);
		setup_line(115,k,651.3230769230769,174.46153846153842,603.1030936382272,240.83065166022794,0);
		setup_line(116,k,370.48770902284247,108.09242526284896,292.4661371310037,133.4431707015591,0);
		setup_line(117,k,292.4661371310037,215.4799062215178,370.48770902284247,240.83065166022794,0);
		setup_line(118,k,418.7076923076923,174.46153846153842,292.4661371310037,133.4431707015591,0);
		setup_line(119,k,292.4661371310037,133.4431707015591,292.4661371310037,215.4799062215178,0);
		setup_line(120,k,418.7076923076923,174.46153846153842,370.48770902284247,108.09242526284896,0);
		setup_line(121,k,137.87232440745785,108.09242526284896,59.850752515619064,215.4799062215178,0);
		setup_line(122,k,59.85075251561906,133.4431707015591,59.850752515619064,215.4799062215178,0);
		setup_line(123,k,137.87232440745785,108.09242526284896,186.09230769230768,174.46153846153842,0);
		setup_line(124,k,186.09230769230768,174.46153846153842,137.87232440745788,240.83065166022794,0);
		setup_line(125,k,186.09230769230768,174.46153846153842,59.850752515619064,215.4799062215178,0);
		setup_line(126,k,-176.76463209976555,133.4431707015591,-172.76463209976555,129.4431707015591,0);
		setup_line(127,k,-176.76463209976555,129.4431707015591,-172.76463209976555,133.4431707015591,0);
		setup_line(128,k,-172.76463209976555,215.4799062215178,-94.74306020792675,240.83065166022794,0);
		setup_line(129,k,-94.74306020792676,108.09242526284896,-172.76463209976555,215.4799062215178,0);
		setup_line(130,k,-94.74306020792676,108.09242526284896,-46.52307692307693,174.46153846153842,0);
		setup_line(131,k,-94.74306020792676,108.09242526284896,-94.74306020792675,240.83065166022794,0);
		setup_line(132,k,-46.52307692307693,174.46153846153842,-94.74306020792675,240.83065166022794,0);
		setup_line(133,k,-327.35844482331134,108.09242526284896,-405.38001671515013,133.4431707015591,0);
		setup_line(134,k,-279.1384615384615,174.46153846153842,-405.38001671515013,215.4799062215178,0);
		setup_line(135,k,-279.1384615384615,174.46153846153842,-327.35844482331134,240.83065166022794,0);
		setup_line(136,k,-405.38001671515013,215.4799062215178,-327.35844482331134,240.83065166022794,0);
		setup_line(137,k,-641.9954013305348,215.4799062215178,-637.9954013305348,211.4799062215178,0);
		setup_line(138,k,-641.9954013305348,211.4799062215178,-637.9954013305348,215.4799062215178,0);
		setup_line(139,k,-511.75384615384615,174.46153846153842,-559.973829438696,108.09242526284896,0);
		setup_line(140,k,-511.75384615384615,174.46153846153842,-637.9954013305348,133.4431707015591,0);
		setup_line(141,k,-559.973829438696,108.09242526284896,-637.9954013305348,133.4431707015591,0);
		setup_line(142,k,-637.9954013305348,133.4431707015591,-559.9738294386959,240.83065166022794,0);
		setup_line(143,k,651.3230769230769,407.0769230769231,603.103093638227,340.7078098782336,0);
		setup_line(144,k,525.0815217463883,366.05855531694374,525.0815217463883,448.09529083690245,0);
		setup_line(145,k,651.3230769230769,407.0769230769231,603.1030936382272,473.4460362756126,0);
		setup_line(146,k,525.0815217463883,366.05855531694374,603.1030936382272,473.4460362756126,0);
		setup_line(147,k,414.7076923076923,407.0769230769231,418.7076923076923,403.0769230769231,0);
		setup_line(148,k,414.7076923076923,403.0769230769231,418.7076923076923,407.0769230769231,0);
		setup_line(149,k,292.4661371310037,366.05855531694374,370.48770902284247,340.7078098782336,0);
		setup_line(150,k,292.4661371310037,366.05855531694374,292.4661371310037,448.09529083690245,0);
		setup_line(151,k,370.48770902284247,340.7078098782336,370.48770902284247,473.4460362756126,0);
		setup_line(152,k,292.4661371310037,448.09529083690245,370.48770902284247,473.4460362756126,0);
		setup_line(153,k,186.09230769230768,407.0769230769231,137.87232440745785,340.7078098782336,0);
		setup_line(154,k,186.09230769230768,407.0769230769231,59.85075251561906,366.05855531694374,0);
		setup_line(155,k,186.09230769230768,407.0769230769231,59.850752515619064,448.09529083690245,0);
		setup_line(156,k,186.09230769230768,407.0769230769231,137.87232440745788,473.4460362756126,0);
		setup_line(157,k,-94.74306020792676,340.7078098782336,-46.52307692307693,407.0769230769231,0);
		setup_line(158,k,-94.74306020792676,340.7078098782336,-172.76463209976555,366.05855531694374,0);
		setup_line(159,k,-46.52307692307693,407.0769230769231,-172.76463209976555,448.09529083690245,0);
		setup_line(160,k,-46.52307692307693,407.0769230769231,-94.74306020792675,473.4460362756126,0);
		setup_line(161,k,-283.1384615384615,407.0769230769231,-279.1384615384615,403.0769230769231,0);
		setup_line(162,k,-283.1384615384615,403.0769230769231,-279.1384615384615,407.0769230769231,0);
		setup_line(163,k,-409.38001671515013,448.09529083690245,-405.38001671515013,444.09529083690245,0);
		setup_line(164,k,-409.38001671515013,444.09529083690245,-405.38001671515013,448.09529083690245,0);
		setup_line(165,k,-327.35844482331134,340.7078098782336,-405.38001671515013,366.05855531694374,0);
		setup_line(166,k,-327.35844482331134,340.7078098782336,-327.35844482331134,473.4460362756126,0);
		setup_line(167,k,-405.38001671515013,366.05855531694374,-327.35844482331134,473.4460362756126,0);
		setup_line(168,k,-641.9954013305348,366.05855531694374,-637.9954013305348,362.05855531694374,0);
		setup_line(169,k,-641.9954013305348,362.05855531694374,-637.9954013305348,366.05855531694374,0);
		setup_line(170,k,-511.75384615384615,407.0769230769231,-559.973829438696,340.7078098782336,0);
		setup_line(171,k,-559.973829438696,340.7078098782336,-637.9954013305348,448.09529083690245,0);
		setup_line(172,k,-511.75384615384615,407.0769230769231,-559.9738294386959,473.4460362756126,0);
		setup_line(173,k,525.0815217463883,598.6739399323284,525.0815217463883,680.7106754522871,0);
		setup_line(174,k,651.3230769230769,639.6923076923077,603.1030936382272,706.0614208909972,0);
		setup_line(175,k,603.103093638227,573.3231944936183,603.1030936382272,706.0614208909972,0);
		setup_line(176,k,366.48770902284247,706.0614208909972,370.48770902284247,702.0614208909972,0);
		setup_line(177,k,366.48770902284247,702.0614208909972,370.48770902284247,706.0614208909972,0);
		setup_line(178,k,418.7076923076923,639.6923076923077,370.48770902284247,573.3231944936183,0);
		setup_line(179,k,418.7076923076923,639.6923076923077,292.4661371310037,598.6739399323284,0);
		setup_line(180,k,418.7076923076923,639.6923076923077,292.4661371310037,680.7106754522871,0);
		setup_line(181,k,55.85075251561906,598.6739399323284,59.85075251561906,594.6739399323284,0);
		setup_line(182,k,55.85075251561906,594.6739399323284,59.85075251561906,598.6739399323284,0);
		setup_line(183,k,186.09230769230768,639.6923076923077,137.87232440745785,573.3231944936183,0);
		setup_line(184,k,59.850752515619064,680.7106754522871,137.87232440745788,706.0614208909972,0);
		setup_line(185,k,-50.52307692307693,639.6923076923077,-46.52307692307693,635.6923076923077,0);
		setup_line(186,k,-50.52307692307693,635.6923076923077,-46.52307692307693,639.6923076923077,0);
		setup_line(187,k,-176.76463209976555,598.6739399323284,-172.76463209976555,594.6739399323284,0);
		setup_line(188,k,-176.76463209976555,594.6739399323284,-172.76463209976555,598.6739399323284,0);
		setup_line(189,k,-94.74306020792676,573.3231944936183,-172.76463209976555,680.7106754522871,0);
		setup_line(190,k,-94.74306020792676,573.3231944936183,-94.74306020792675,706.0614208909972,0);
		setup_line(191,k,-283.1384615384615,639.6923076923077,-279.1384615384615,635.6923076923077,0);
		setup_line(192,k,-283.1384615384615,635.6923076923077,-279.1384615384615,639.6923076923077,0);
		setup_line(193,k,-409.38001671515013,680.7106754522871,-405.38001671515013,676.7106754522871,0);
		setup_line(194,k,-409.38001671515013,676.7106754522871,-405.38001671515013,680.7106754522871,0);
		setup_line(195,k,-331.35844482331134,706.0614208909972,-327.35844482331134,702.0614208909972,0);
		setup_line(196,k,-331.35844482331134,702.0614208909972,-327.35844482331134,706.0614208909972,0);
		setup_line(197,k,-327.35844482331134,573.3231944936183,-405.38001671515013,598.6739399323284,0);
		setup_line(198,k,-515.7538461538461,639.6923076923077,-511.7538461538461,635.6923076923077,0);
		setup_line(199,k,-515.7538461538461,635.6923076923077,-511.7538461538461,639.6923076923077,0);
		setup_line(200,k,-563.973829438696,573.3231944936183,-559.973829438696,569.3231944936183,0);
		setup_line(201,k,-563.973829438696,569.3231944936183,-559.973829438696,573.3231944936183,0);
		setup_line(202,k,-641.9954013305348,598.6739399323284,-637.9954013305348,594.6739399323284,0);
		setup_line(203,k,-641.9954013305348,594.6739399323284,-637.9954013305348,598.6739399323284,0);
		setup_line(204,k,-641.9954013305348,680.7106754522871,-637.9954013305348,676.7106754522871,0);
		setup_line(205,k,-641.9954013305348,676.7106754522871,-637.9954013305348,680.7106754522871,0);
		setup_line(206,k,-563.9738294386959,706.0614208909972,-559.9738294386959,702.0614208909972,0);
		setup_line(207,k,-563.9738294386959,702.0614208909972,-559.9738294386959,706.0614208909972,0);

		for(;;) {
			for(unsigned i = 0; i < 208; ++i) {
				execute_line(i);
			}
		}
	}

	// FLAG test

	// 27 short/medium lines
	// Benchmarks at 557.6 Hz or about 15,055 vectors per second

	int gg = 0;

	double k = 0.006;
	unsigned j = 0;
	for(unsigned i = 1; i < (2*N_POINTS-3); ++i) {
		double x0, y0, x1, y1, kk = k;

		x0 = wrapx(i);   y0 = wrapy(i);
		x1 = wrapx(i+1); y1 = wrapy(i+1);
		setup_line(j++, kk, x0, y0, x1, y1, 0);
	}

	for(;;) {
		/*
		int g = (f >> 8) & 63;
		if (g != gg) {
			gg = g;
			k = g ? k*1.08 : 0.0002;
			update_display_list(k);
		}*/
		for(unsigned i = 0; i < j; ++i) {
			// TODO: Try shuffling the lines randomly as a DAC stress test
			execute_line(i);
		}
	}


}
