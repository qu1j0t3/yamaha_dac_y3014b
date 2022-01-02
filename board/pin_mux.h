/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*! @name PORTC2 (number 25), X_INT_RESET
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_X_INT_RESET_FGPIO FGPIOA             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_X_INT_RESET_GPIO GPIOA               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_X_INT_RESET_GPIO_PORT kGPIO_PORTC    /*!<@brief PORTA GPIO port: PORTA */
#define BOARD_INITPINS_X_INT_RESET_GPIO_PIN_MASK (1U << 18U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_X_INT_RESET_PORT PORTC               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_X_INT_RESET_PIN 2U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_X_INT_RESET_PIN_MASK (1U << 2U)      /*!<@brief PORT pin mask */
                                                            /* @} */

/*! @name PORTD6 (number 27), X_INT_HOLD
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_X_INT_HOLD_FGPIO FGPIOA             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_X_INT_HOLD_GPIO GPIOA               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_X_INT_HOLD_GPIO_PORT kGPIO_PORTD    /*!<@brief PORTA GPIO port: PORTA */
#define BOARD_INITPINS_X_INT_HOLD_GPIO_PIN_MASK (1U << 30U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_X_INT_HOLD_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_X_INT_HOLD_PIN 6U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_X_INT_HOLD_PIN_MASK (1U << 6U)      /*!<@brief PORT pin mask */
                                                           /* @} */

/*! @name PORTE4 (number 47), DAC_LOAD
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_DAC_LOAD_FGPIO FGPIOB             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_DAC_LOAD_GPIO GPIOB               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_DAC_LOAD_GPIO_PORT kGPIO_PORTE    /*!<@brief PORTB GPIO port: PORTB */
#define BOARD_INITPINS_DAC_LOAD_GPIO_PIN_MASK (1U << 4U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_DAC_LOAD_PORT PORTE               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_DAC_LOAD_PIN 4U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_DAC_LOAD_PIN_MASK (1U << 4U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PORTE6 (number 20), DAC_SD_X_COEFF
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_DAC_SD_X_COEFF_FGPIO FGPIOB             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_DAC_SD_X_COEFF_GPIO GPIOB               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_DAC_SD_X_COEFF_GPIO_PORT kGPIO_PORTE    /*!<@brief PORTB GPIO port: PORTB */
#define BOARD_INITPINS_DAC_SD_X_COEFF_GPIO_PIN_MASK (1U << 6U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_DAC_SD_X_COEFF_PORT PORTE               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_DAC_SD_X_COEFF_PIN 6U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_DAC_SD_X_COEFF_PIN_MASK (1U << 6U)      /*!<@brief PORT pin mask */
                                                               /* @} */

/*! @name PORTE5 (number 21), DAC_CLOCK
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_DAC_CLOCK_FGPIO FGPIOB             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_DAC_CLOCK_GPIO GPIOB               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_DAC_CLOCK_GPIO_PORT kGPIO_PORTE    /*!<@brief PORTB GPIO port: PORTB */
#define BOARD_INITPINS_DAC_CLOCK_GPIO_PIN_MASK (1U << 5U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_DAC_CLOCK_PORT PORTE               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_DAC_CLOCK_PIN 5U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_DAC_CLOCK_PIN_MASK (1U << 5U)      /*!<@brief PORT pin mask */
                                                          /* @} */

/*! @name PORTC0 (number 32), TRIGGER
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_TRIGGER_FGPIO FGPIOA             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_TRIGGER_GPIO GPIOA               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_TRIGGER_GPIO_PORT kGPIO_PORTC    /*!<@brief PORTA GPIO port: PORTA */
#define BOARD_INITPINS_TRIGGER_GPIO_PIN_MASK (1U << 16U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_TRIGGER_PORT PORTC               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_TRIGGER_PIN 0U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_TRIGGER_PIN_MASK (1U << 0U)      /*!<@brief PORT pin mask */
                                                        /* @} */

/*! @name PORTH6 (number 4), DAC_SD_Y_COEFF
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_DAC_SD_Y_COEFF_FGPIO FGPIOB             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_DAC_SD_Y_COEFF_GPIO GPIOB               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_DAC_SD_Y_COEFF_GPIO_PORT kGPIO_PORTH    /*!<@brief PORTB GPIO port: PORTB */
#define BOARD_INITPINS_DAC_SD_Y_COEFF_GPIO_PIN_MASK (1U << 30U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_DAC_SD_Y_COEFF_PORT PORTH               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_DAC_SD_Y_COEFF_PIN 6U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_DAC_SD_Y_COEFF_PIN_MASK (1U << 6U)      /*!<@brief PORT pin mask */
                                                               /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
