/*
** ###################################################################
**     Processors:          MKE06Z128VLD4
**                          MKE06Z128VLH4
**                          MKE06Z128VLK4
**                          MKE06Z128VQH4
**                          MKE06Z64VLD4
**                          MKE06Z64VLH4
**                          MKE06Z64VLK4
**                          MKE06Z64VQH4
**
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**                          MCUXpresso Compiler
**
**     Reference manual:    MKE06P80M48SF0RM Rev 4
**     Version:             rev. 1.0, 2017-05-19
**     Build:               b170811
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2017 NXP
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     1. Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     2. Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     3. Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2017-05-19)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file MKE06Z4
 * @version 1.0
 * @date 2017-05-19
 * @brief Device specific configuration file for MKE06Z4 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "fsl_device_registers.h"



/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {

#if (DISABLE_WDOG)
  WDOG->CNT = WDOG_UPDATE_KEY1;
  WDOG->CNT = WDOG_UPDATE_KEY2;
  WDOG->TOVAL = 0xFFFFU;
  WDOG->CS1 = (uint8_t) ((WDOG->CS1) & ~WDOG_CS1_EN_MASK) | WDOG_CS1_UPDATE_MASK;
  WDOG->CS2 |= 0;
#endif /* (DISABLE_WDOG) */

}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {

  uint32_t ICSOUTClock; /* Variable to store output clock frequency of the ICS module */
  uint16_t Divider;
  uint16_t Temp;

  Divider = (0x01U) << ((ICS->C2 & ICS_C2_BDIV_MASK) >> ICS_C2_BDIV_SHIFT);

  switch ((ICS->C1 & ICS_C1_CLKS_MASK) >> ICS_C1_CLKS_SHIFT) {
    case 0x0:
      /* FLL */
      if(ICS->C1 & ICS_C1_IREFS_MASK)
      {
          ICSOUTClock = CPU_INT_IRC_CLK_HZ * 1280UL;
      }
      else
      {
          /* Reference Divider */
          Temp = (ICS->C1 & ICS_C1_RDIV_MASK) >> ICS_C1_RDIV_SHIFT;
          Temp = (Temp + 1) * ((OSC->CR & OSC_CR_RANGE_MASK) ? 32 : 1);

          ICSOUTClock = CPU_XTAL_CLK_HZ / Temp * 1280L;
      }
      break;

    case 0x1:
      /* Internal IRC */
      ICSOUTClock = CPU_INT_IRC_CLK_HZ;
      break;

    case 0x2:
      /* External OSC */
      ICSOUTClock = CPU_XTAL_CLK_HZ;
      break;

    default:
      return;
  }
  SystemCoreClock = (ICSOUTClock / Divider);

}
