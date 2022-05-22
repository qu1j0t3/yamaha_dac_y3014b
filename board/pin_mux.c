/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v10.0
processor: MKE06Z128xxx4
package_id: MKE06Z128VLK4
mcu_data: ksdk2_0
processor_version: 10.0.0
board: FRDM-KE06Z
pin_labels:
- {pin_num: '42', pin_signal: PTB0/KBI0_P8/UART0_RX/PWT_IN1/ADC0_SE4, label: 'NOTCS_DAC_POS/J1[2]', identifier: NOTCS_DAC_POS}
- {pin_num: '41', pin_signal: PTB1/KBI0_P9/UART0_TX/ADC0_SE5, label: 'J1[4]/PTB1_IRTX', identifier: IRTX}
- {pin_num: '28', pin_signal: PTD5/KBI0_P29/PWT_IN0, label: 'J1[6]'}
- {pin_num: '2', pin_signal: PTD0/KBI0_P24/FTM2_CH2/SPI1_SCK, label: 'J1[8]/PTD0_D3_PWM2'}
- {pin_num: '61', pin_signal: PTA1/KBI0_P1/FTM0_CH1/I2C0_4WSDAOUT/ACMP0_IN1/ADC0_SE1, label: 'J1[10]/PTA1_D4_T1/PTA1_IRRX', identifier: IRRX;NOTCS_DAC_LIMIT;NOTCS_DAC_1}
- {pin_num: '1', pin_signal: PTD1/KBI0_P25/FTM2_CH3/SPI1_MOSI, label: 'J1[12]/PTD1_D5_PWM3'}
- {pin_num: '23', pin_signal: PTB4/KBI0_P12/FTM2_CH4/SPI0_MISO/ACMP1_IN2/NMI_b, label: 'J1[14]/J2[10]/PTB4_D6_PWM4', identifier: STOP;LIMIT_LOW}
- {pin_num: '45', pin_signal: PTA7/KBI0_P7/FTM2_FLT2/ACMP1_IN1/ADC0_SE3, label: 'J1[16]/J2[17]/PTA7_D7_ANB1', identifier: NOTCS_DAC_Z;Z_ENABLE}
- {pin_num: '62', pin_signal: PTA0/KBI0_P0/FTM0_CH0/I2C0_4WSCLOUT/ACMP0_IN0/ADC0_SE0, label: 'J2[2]/PTA0_D8_T0', identifier: NOTCS_DAC_POS;NOTCS_DAC_0}
- {pin_num: '19', pin_signal: PTH0/KBI1_P24/FTM2_CH0, label: 'J2[4]/PTH0_D9_PWM0', identifier: LIMIT_LOW}
- {pin_num: '22', pin_signal: PTB5/KBI0_P13/FTM2_CH5/SPI0_PCS/ACMP1_OUT, label: 'J2[6]/PTB5_D10_PWM5', identifier: Y_COMP_SEL}
- {pin_num: '39', pin_signal: PTB3/KBI0_P11/SPI0_MOSI/FTM0_CH1/ADC0_SE7, label: 'J2[8]/J2[19]/PTB3_D11_ANB2', identifier: X_COMP_SEL}
- {pin_num: '40', pin_signal: PTB2/KBI0_P10/SPI0_SCK/FTM0_CH0/ADC0_SE6, label: 'J2[12]/J2[1]/PTB2_D13_T3', identifier: TESTINPUT;STOP}
- {pin_num: '60', pin_signal: PTA2/KBI0_P2/UART0_RX/I2C0_SDA, label: 'J2[18]/PTA2_ACCEL_SDA', identifier: ACCEL_SDA}
- {pin_num: '59', pin_signal: PTA3/KBI0_P3/UART0_TX/I2C0_SCL, label: 'J2[20]/PTA3_ACCEL_SCL', identifier: ACCEL_SCL;TESTINPUT}
- {pin_num: '10', pin_signal: VREFH, label: 'J2[16]/VDD_KE06Z'}
- {pin_num: '11', pin_signal: VREFL, label: GND}
- {pin_num: '63', pin_signal: PTC7/KBI0_P23/UART1_TX/CAN0_TX, label: 'J1[1]/PTC7_TXD1/UART1_TX_TGTMCU', identifier: DEBUG_UART_TX}
- {pin_num: '64', pin_signal: PTC6/KBI0_P22/UART1_RX/CAN0_RX, label: 'J1[3]/PTC6_RXD1/UART1_RX_TGTMCU', identifier: DEBUG_UART_RX}
- {pin_num: '26', pin_signal: PTD7/KBI0_P31/UART2_TX, label: 'Y_INT_HOLD/J1[5]/PTD7_TXD2', identifier: Y_INT_HOLD;INT_HOLD}
- {pin_num: '27', pin_signal: PTD6/KBI0_P30/UART2_RX, label: 'X_INT_HOLD/J1[7]/PTD6_RXD2', identifier: X_INT_HOLD}
- {pin_num: '47', pin_signal: PTE4/KBI1_P4, label: 'DAC_LOAD/J1[9]/PTE4_GPIO1', identifier: DAC_LOAD}
- {pin_num: '77', pin_signal: PTC5/KBI0_P21/FTM1_CH1/RTCO, label: 'Z_BLANK/J1[11]/PTC5_T2', identifier: Z_BLANK}
- {pin_num: '20', pin_signal: PTE6/KBI1_P6, label: 'DAC_SD_X_COEFF/J1[13]/PTE6_LED2', identifier: DAC_SD_X_COEFF;NOTCS_DAC_XCOEFF}
- {pin_num: '21', pin_signal: PTE5/KBI1_P5, label: 'DAC_CLOCK/J1[15]/PTE5_LED3', identifier: DAC_CLOCK}
- {pin_num: '31', pin_signal: PTC1/KBI0_P17/FTM2_CH1/ADC0_SE9, label: 'J2[3]/J3[6]/PTC1_A2_PWM1'}
- {pin_num: '43', pin_signal: PTF3/KBI1_P11/UART1_TX, label: 'J2[5]/PTF3_LED5'}
- {pin_num: '44', pin_signal: PTF2/KBI1_P10/UART1_RX, label: 'J2[7]/PTF2_LED6'}
- {pin_num: '35', pin_signal: PTF7/KBI1_P15/ADC0_SE15, label: 'J2[9]/J3[2]/PTF7_A0_ANA0'}
- {pin_num: '36', pin_signal: PTF6/KBI1_P14/ADC0_SE14, label: 'J2[11]/J3[12]/PTF6_A5_ANA1'}
- {pin_num: '46', pin_signal: PTA6/KBI0_P6/FTM2_FLT1/ACMP1_IN0/ADC0_SE2, label: 'J2[15]/PTA6_ANB0'}
- {pin_num: '73', pin_signal: PTG1/KBI1_P17, label: 'J3[1]/PTG1_GPIO6'}
- {pin_num: '72', pin_signal: PTG2/KBI1_P18, label: 'J3[3]/PTG2_GPIO5'}
- {pin_num: '74', pin_signal: PTG0/KBI1_P16, label: 'J3[5]/PTG0_GPIO7'}
- {pin_num: '78', pin_signal: PTC4/KBI0_P20/RTCO/FTM1_CH0/ACMP0_IN2/SWD_CLK, label: 'J3[7]/PTC4_SWD_CLK_GPIO8'}
- {pin_num: '3', pin_signal: PTH7/KBI1_P31/PWT_IN1, label: 'J3[9]/PTH7_GPIO9'}
- {pin_num: '4', pin_signal: PTH6/KBI1_P30, label: 'DAC_SD_Y_COEFF/J3[11]/PTH6_GPIO10', identifier: DAC_SD_Y_COEFF;NOTCS_DAC_YCOEFF;NOTCS_DAC_COEFF}
- {pin_num: '76', pin_signal: PTE0/KBI1_P0/SPI0_SCK/TCLK1/I2C1_SDA, label: 'J4[1]/PTE0_SPI0_SCK', identifier: SPI_CLOCK}
- {pin_num: '75', pin_signal: PTE1/KBI1_P1/SPI0_MOSI/I2C1_SCL, label: 'J4[3]/PTE1_SPI0_MOSI', identifier: SPI_DATAOUT}
- {pin_num: '68', pin_signal: PTE2/KBI1_P2/SPI0_MISO/PWT_IN0, label: 'J4[5]/PTE2_SPI0_MISO'}
- {pin_num: '67', pin_signal: PTE3/KBI1_P3/SPI0_PCS, label: 'J4[7]/PTE3_SPI0_SS', identifier: SPI_SS}
- {pin_num: '58', pin_signal: PTD2/KBI0_P26/SPI1_MISO, label: 'J4[9]/PTD2_SPI1_MISO'}
- {pin_num: '54', pin_signal: PTF1/KBI1_P9/FTM2_CH1, label: 'J4[11]/PTF1_GPIO2'}
- {pin_num: '55', pin_signal: PTF0/KBI1_P8/FTM2_CH0, label: 'J4[13]/PTF0_GPIO3'}
- {pin_num: '71', pin_signal: PTG3/KBI1_P19, label: 'J4[15]/PTG3_GPIO4'}
- {pin_num: '79', pin_signal: PTA5/KBI0_P5/IRQ/TCLK0/RESET_b, label: 'J4[6]/J7[10]/RST_TGTMCU/SW1'}
- {pin_num: '32', pin_signal: PTC0/KBI0_P16/FTM2_CH0/ADC0_SE8, label: 'TRIGGER/J3[4]/PTC0_A1', identifier: TRIGGER}
- {pin_num: '25', pin_signal: PTC2/KBI0_P18/FTM2_CH2/ADC0_SE10, label: 'X_INT_RESET/J3[8]/PTC2_A3', identifier: STOP;STOP_INT}
- {pin_num: '24', pin_signal: PTC3/KBI0_P19/FTM2_CH3/ADC0_SE11, label: 'Y_INT_RESET/J3[10]/PTC3_A4', identifier: Y_INT_RESET;INT_RESET}
- {pin_num: '29', pin_signal: PTI6/IRQ, label: 'J5[1]'}
- {pin_num: '66', pin_signal: PTI2/IRQ, label: 'J5[3]'}
- {pin_num: '6', pin_signal: PTE7/KBI1_P7/TCLK2/FTM1_CH1/CAN0_TX, label: 'J5[5]/U7[1]/PTE7_CONN/CAN_TX', identifier: CAN_TX}
- {pin_num: '18', pin_signal: PTH1/KBI1_P25/FTM2_CH1, label: 'J5[7]'}
- {pin_num: '30', pin_signal: PTI5/IRQ, label: 'J5[15]'}
- {pin_num: '53', pin_signal: PTG4/KBI1_P20/FTM2_CH2/SPI1_SCK, label: 'J5[2]'}
- {pin_num: '7', pin_signal: PTH2/KBI1_P26/BUSOUT/FTM1_CH0/CAN0_RX, label: 'J5[4]/U7[4]/PTH2_CONN/CAN_RX', identifier: CAN_RX}
- {pin_num: '5', pin_signal: PTH5/KBI1_P29, label: 'J5[6]'}
- {pin_num: '17', pin_signal: PTI0/IRQ/UART2_RX, label: 'J5[8]'}
- {pin_num: '16', pin_signal: PTI1/IRQ/UART2_TX, label: 'J5[12]'}
- {pin_num: '15', pin_signal: PTI4/IRQ, label: 'J5[14]'}
- {pin_num: '65', pin_signal: PTI3/IRQ, label: 'J5[16]'}
- {pin_num: '8', pin_signal: VDD8, label: VDD_KE06Z}
- {pin_num: '49', pin_signal: VDD49, label: VDD_KE06Z}
- {pin_num: '70', pin_signal: VDD70, label: VDD_KE06Z}
- {pin_num: '9', pin_signal: VREFH/VDDA, label: VDD_KE06Z}
- {pin_num: '12', pin_signal: VSSA/VSS, label: GND}
- {pin_num: '69', pin_signal: VSS69, label: GND}
- {pin_num: '48', pin_signal: VSS48, label: GND}
- {pin_num: '38', pin_signal: PTF4/KBI1_P12/ADC0_SE12, label: PTF4_THER, identifier: THER_A}
- {pin_num: '37', pin_signal: PTF5/KBI1_P13/ADC0_SE13, label: PTF5_THER, identifier: THER_B}
- {pin_num: '52', pin_signal: PTG5/KBI1_P21/FTM2_CH3/SPI1_MOSI, label: 'D4[1]/PTG5_RED', identifier: LED_RED}
- {pin_num: '51', pin_signal: PTG6/KBI1_P22/FTM2_CH4/SPI1_MISO, label: 'D4[4]/PTG6_GREEN', identifier: LED_GREEN}
- {pin_num: '50', pin_signal: PTG7/KBI1_P23/FTM2_CH5/SPI1_PCS, label: 'D4[3]/PTG7_BLUE', identifier: LED_BLUE}
- {pin_num: '34', pin_signal: PTH3/KBI1_P27/I2C1_SDA, label: PTH3_KEY1}
- {pin_num: '33', pin_signal: PTH4/KBI1_P28/I2C1_SCL, label: PTH4_KEY2}
- {pin_num: '57', pin_signal: PTD3/KBI0_P27/SPI1_PCS, label: PTD3_ACCEL_INT2, identifier: ACCEL_INT2}
- {pin_num: '56', pin_signal: PTD4/KBI0_P28, label: PTD4_ACCEL_INT1, identifier: ACCEL_INT1}
- {pin_num: '14', pin_signal: PTB6/KBI0_P14/I2C0_SDA/XTAL, label: XTAL, identifier: XTAL}
- {pin_num: '13', pin_signal: PTB7/KBI0_P15/I2C0_SCL/EXTAL, label: EXTAL, identifier: EXTAL}
- {pin_num: '80', pin_signal: PTA4/KBI0_P4/ACMP0_OUT/SWD_DIO, label: 'J7[2]/SWD_DIO_TGTMCU'}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '4', peripheral: GPIOB, signal: 'GPIO, 30', pin_signal: PTH6/KBI1_P30, identifier: NOTCS_DAC_COEFF, direction: OUTPUT}
  - {pin_num: '24', peripheral: GPIOA, signal: 'GPIO, 19', pin_signal: PTC3/KBI0_P19/FTM2_CH3/ADC0_SE11, identifier: INT_RESET, direction: OUTPUT}
  - {pin_num: '26', peripheral: GPIOA, signal: 'GPIO, 31', pin_signal: PTD7/KBI0_P31/UART2_TX, identifier: INT_HOLD, direction: OUTPUT}
  - {pin_num: '32', peripheral: GPIOA, signal: 'GPIO, 16', pin_signal: PTC0/KBI0_P16/FTM2_CH0/ADC0_SE8, direction: OUTPUT}
  - {pin_num: '77', peripheral: GPIOA, signal: 'GPIO, 21', pin_signal: PTC5/KBI0_P21/FTM1_CH1/RTCO, direction: OUTPUT}
  - {pin_num: '62', peripheral: GPIOA, signal: 'GPIO, 0', pin_signal: PTA0/KBI0_P0/FTM0_CH0/I2C0_4WSCLOUT/ACMP0_IN0/ADC0_SE0, identifier: NOTCS_DAC_0, direction: OUTPUT}
  - {pin_num: '76', peripheral: GPIOB, signal: 'GPIO, 0', pin_signal: PTE0/KBI1_P0/SPI0_SCK/TCLK1/I2C1_SDA, direction: OUTPUT}
  - {pin_num: '75', peripheral: GPIOB, signal: 'GPIO, 1', pin_signal: PTE1/KBI1_P1/SPI0_MOSI/I2C1_SCL, direction: OUTPUT}
  - {pin_num: '61', peripheral: GPIOA, signal: 'GPIO, 1', pin_signal: PTA1/KBI0_P1/FTM0_CH1/I2C0_4WSDAOUT/ACMP0_IN1/ADC0_SE1, identifier: NOTCS_DAC_1, direction: OUTPUT}
  - {pin_num: '39', peripheral: GPIOA, signal: 'GPIO, 11', pin_signal: PTB3/KBI0_P11/SPI0_MOSI/FTM0_CH1/ADC0_SE7, direction: OUTPUT}
  - {pin_num: '22', peripheral: GPIOA, signal: 'GPIO, 13', pin_signal: PTB5/KBI0_P13/FTM2_CH5/SPI0_PCS/ACMP1_OUT, direction: OUTPUT}
  - {pin_num: '19', peripheral: GPIOB, signal: 'GPIO, 24', pin_signal: PTH0/KBI1_P24/FTM2_CH0, direction: OUTPUT}
  - {pin_num: '45', peripheral: GPIOA, signal: 'GPIO, 7', pin_signal: PTA7/KBI0_P7/FTM2_FLT2/ACMP1_IN1/ADC0_SE3, identifier: Z_ENABLE, direction: OUTPUT}
  - {pin_num: '25', peripheral: KBI0, signal: 'P, 18', pin_signal: PTC2/KBI0_P18/FTM2_CH2/ADC0_SE10, identifier: STOP_INT, direction: INPUT}
  - {pin_num: '40', peripheral: GPIOA, signal: 'GPIO, 10', pin_signal: PTB2/KBI0_P10/SPI0_SCK/FTM0_CH0/ADC0_SE6, identifier: STOP, direction: INPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{

    gpio_pin_config_t NOTCS_DAC_0_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA0 (pin 62) */
    GPIO_PinInit(BOARD_INITPINS_NOTCS_DAC_0_GPIO_PORT, BOARD_INITPINS_NOTCS_DAC_0_PIN, &NOTCS_DAC_0_config);

    gpio_pin_config_t NOTCS_DAC_1_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA1 (pin 61) */
    GPIO_PinInit(BOARD_INITPINS_NOTCS_DAC_1_GPIO_PORT, BOARD_INITPINS_NOTCS_DAC_1_PIN, &NOTCS_DAC_1_config);

    gpio_pin_config_t Z_ENABLE_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA7 (pin 45) */
    GPIO_PinInit(BOARD_INITPINS_Z_ENABLE_GPIO_PORT, BOARD_INITPINS_Z_ENABLE_PIN, &Z_ENABLE_config);

    gpio_pin_config_t STOP_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA10 (pin 40) */
    GPIO_PinInit(BOARD_INITPINS_STOP_GPIO_PORT, BOARD_INITPINS_STOP_PIN, &STOP_config);

    gpio_pin_config_t X_COMP_SEL_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA11 (pin 39) */
    GPIO_PinInit(BOARD_INITPINS_X_COMP_SEL_GPIO_PORT, BOARD_INITPINS_X_COMP_SEL_PIN, &X_COMP_SEL_config);

    gpio_pin_config_t Y_COMP_SEL_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA13 (pin 22) */
    GPIO_PinInit(BOARD_INITPINS_Y_COMP_SEL_GPIO_PORT, BOARD_INITPINS_Y_COMP_SEL_PIN, &Y_COMP_SEL_config);

    gpio_pin_config_t TRIGGER_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA16 (pin 32) */
    GPIO_PinInit(BOARD_INITPINS_TRIGGER_GPIO_PORT, BOARD_INITPINS_TRIGGER_PIN, &TRIGGER_config);

    gpio_pin_config_t INT_RESET_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA19 (pin 24) */
    GPIO_PinInit(BOARD_INITPINS_INT_RESET_GPIO_PORT, BOARD_INITPINS_INT_RESET_PIN, &INT_RESET_config);

    gpio_pin_config_t Z_BLANK_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA21 (pin 77) */
    GPIO_PinInit(BOARD_INITPINS_Z_BLANK_GPIO_PORT, BOARD_INITPINS_Z_BLANK_PIN, &Z_BLANK_config);

    gpio_pin_config_t INT_HOLD_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA31 (pin 26) */
    GPIO_PinInit(BOARD_INITPINS_INT_HOLD_GPIO_PORT, BOARD_INITPINS_INT_HOLD_PIN, &INT_HOLD_config);

    gpio_pin_config_t SPI_CLOCK_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB0 (pin 76) */
    GPIO_PinInit(BOARD_INITPINS_SPI_CLOCK_GPIO_PORT, BOARD_INITPINS_SPI_CLOCK_PIN, &SPI_CLOCK_config);

    gpio_pin_config_t SPI_DATAOUT_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB1 (pin 75) */
    GPIO_PinInit(BOARD_INITPINS_SPI_DATAOUT_GPIO_PORT, BOARD_INITPINS_SPI_DATAOUT_PIN, &SPI_DATAOUT_config);

    gpio_pin_config_t LIMIT_LOW_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB24 (pin 19) */
    GPIO_PinInit(BOARD_INITPINS_LIMIT_LOW_GPIO_PORT, BOARD_INITPINS_LIMIT_LOW_PIN, &LIMIT_LOW_config);

    gpio_pin_config_t NOTCS_DAC_COEFF_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB30 (pin 4) */
    GPIO_PinInit(BOARD_INITPINS_NOTCS_DAC_COEFF_GPIO_PORT, BOARD_INITPINS_NOTCS_DAC_COEFF_PIN, &NOTCS_DAC_COEFF_config);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
