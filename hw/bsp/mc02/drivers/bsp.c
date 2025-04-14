#include "bsp.h"

#include "bsp_dwt.h"
#include "bsp_uart.h"
#include "main.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_it.h"
extern TIM_HandleTypeDef htim23;
void bsp_init(void) {
  /* MPU Configuration--------------------------------------------------------*/
  mpu_conf();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM12_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_OCTOSPI2_Init();
  MX_USB_OTG_HS_PCD_Init();
  MX_TIM7_Init();
  MX_SPI6_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();

  DWT_Init(550);  // 喵喵时钟频率550MHz
  HAL_TIM_Base_Stop_IT(&htim23);
  bsp_uart_init();
}
