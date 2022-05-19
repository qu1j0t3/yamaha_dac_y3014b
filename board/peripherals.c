/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v10.0
processor: MKE06Z128xxx4
package_id: MKE06Z128VLK4
mcu_data: ksdk2_0
processor_version: 10.0.0
board: FRDM-KE06Z
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: c25122d4-2740-4d50-a160-3c5a4da67c84
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'uart_cmsis_common'
- type_id: 'uart_cmsis_common_9cb8e302497aa696fdbb5a4fd622c2a8'
- global_USART_CMSIS_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * FTM0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM0'
- type: 'tpm'
- mode: 'EdgeAligned_FTM'
- custom_name_enabled: 'false'
- type_id: 'tpm_e7472ea12d53461b8d293488f3ed72ec'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'FTM0'
- config_sets:
  - tpm_main_config:
    - tpm_config:
      - clockSource: 'kTPM_SystemClock'
      - tpmSrcClkFreq: 'BOARD_BootClockRUN'
      - prescale: 'kTPM_Prescale_Divide_1'
      - timerFrequency: '10000'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - tpm_interrupt:
      - IRQn: 'FTM0_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
    - quick_selection: 'QuickSelectionDefault'
  - tpm_edge_aligned_mode:
    - tpm_edge_aligned_channels_config: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const tpm_config_t FTM0_config = {
  .prescale = kTPM_Prescale_Divide_1,
};

static void FTM0_init(void) {
  TPM_Init(FTM0_PERIPHERAL, &FTM0_config);
  TPM_SetTimerPeriod(FTM0_PERIPHERAL, ((FTM0_CLOCK_SOURCE/ (1U << (FTM0_PERIPHERAL->SC & TPM_SC_PS_MASK))) / 10000) + 1);
  TPM_StartTimer(FTM0_PERIPHERAL, kTPM_SystemClock);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  FTM0_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
