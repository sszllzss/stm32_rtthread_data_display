/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define S3_Pin GPIO_PIN_2
#define S3_GPIO_Port GPIOE
#define S2_Pin GPIO_PIN_3
#define S2_GPIO_Port GPIOE
#define S1_Pin GPIO_PIN_4
#define S1_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOE
#define T_MISO_Pin GPIO_PIN_8
#define T_MISO_GPIO_Port GPIOF
#define T_MOSI_Pin GPIO_PIN_9
#define T_MOSI_GPIO_Port GPIOF
#define T_INT_Pin GPIO_PIN_10
#define T_INT_GPIO_Port GPIOF
#define OV_D0_Pin GPIO_PIN_0
#define OV_D0_GPIO_Port GPIOC
#define OV_D1_Pin GPIO_PIN_1
#define OV_D1_GPIO_Port GPIOC
#define OV_D2_Pin GPIO_PIN_2
#define OV_D2_GPIO_Port GPIOC
#define OV_D3_Pin GPIO_PIN_3
#define OV_D3_GPIO_Port GPIOC
#define S4_Pin GPIO_PIN_0
#define S4_GPIO_Port GPIOA
#define T_KEY_Pin GPIO_PIN_1
#define T_KEY_GPIO_Port GPIOA
#define OV_D4_Pin GPIO_PIN_4
#define OV_D4_GPIO_Port GPIOC
#define OV_D5_Pin GPIO_PIN_5
#define OV_D5_GPIO_Port GPIOC
#define LCD_BL_Pin GPIO_PIN_0
#define LCD_BL_GPIO_Port GPIOB
#define T_SCK_Pin GPIO_PIN_1
#define T_SCK_GPIO_Port GPIOB
#define T_CS_Pin GPIO_PIN_2
#define T_CS_GPIO_Port GPIOB
#define Flash_CS_Pin GPIO_PIN_12
#define Flash_CS_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_6
#define NRF_CE_GPIO_Port GPIOG
#define NRF_CS_Pin GPIO_PIN_7
#define NRF_CS_GPIO_Port GPIOG
#define NRF_IRQ_Pin GPIO_PIN_8
#define NRF_IRQ_GPIO_Port GPIOG
#define OV_D6_Pin GPIO_PIN_6
#define OV_D6_GPIO_Port GPIOC
#define OV_D7_Pin GPIO_PIN_7
#define OV_D7_GPIO_Port GPIOC
#define FIFO_VSYNC_Pin GPIO_PIN_8
#define FIFO_VSYNC_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOA
#define OV_SCL_Pin GPIO_PIN_3
#define OV_SCL_GPIO_Port GPIOD
#define FIFO_WRST_Pin GPIO_PIN_6
#define FIFO_WRST_GPIO_Port GPIOD
#define TM1638_STB_Pin GPIO_PIN_7
#define TM1638_STB_GPIO_Port GPIOD
#define TM1638_DIO_Pin GPIO_PIN_9
#define TM1638_DIO_GPIO_Port GPIOG
#define TM1638_CLK_Pin GPIO_PIN_10
#define TM1638_CLK_GPIO_Port GPIOG
#define DQ_18B20_Pin GPIO_PIN_11
#define DQ_18B20_GPIO_Port GPIOG
#define OV_SDA_Pin GPIO_PIN_13
#define OV_SDA_GPIO_Port GPIOG
#define FIFO_RRST_Pin GPIO_PIN_14
#define FIFO_RRST_GPIO_Port GPIOG
#define FIFO_OE_Pin GPIO_PIN_15
#define FIFO_OE_GPIO_Port GPIOG
#define FIFO_WEN_Pin GPIO_PIN_3
#define FIFO_WEN_GPIO_Port GPIOB
#define FIFO_RCLK_Pin GPIO_PIN_4
#define FIFO_RCLK_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOB
#define VS1838B_OUT_Pin GPIO_PIN_9
#define VS1838B_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
