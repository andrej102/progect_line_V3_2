/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32h7xx_hal.h"

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
#define LED_G_Pin GPIO_PIN_2
#define LED_G_GPIO_Port GPIOE
#define BUT_U_Pin GPIO_PIN_3
#define BUT_U_GPIO_Port GPIOE
#define U2_CTS_Pin GPIO_PIN_0
#define U2_CTS_GPIO_Port GPIOA
#define U2_RTS_Pin GPIO_PIN_1
#define U2_RTS_GPIO_Port GPIOA
#define U2_TX_Pin GPIO_PIN_2
#define U2_TX_GPIO_Port GPIOA
#define U2_RX_Pin GPIO_PIN_3
#define U2_RX_GPIO_Port GPIOA
#define ESP_RDY_Pin GPIO_PIN_5
#define ESP_RDY_GPIO_Port GPIOA
#define LINE1_ST_TIM3_CH1_Pin GPIO_PIN_6
#define LINE1_ST_TIM3_CH1_GPIO_Port GPIOA
#define LED_IR_TIM3_CH2_Pin GPIO_PIN_7
#define LED_IR_TIM3_CH2_GPIO_Port GPIOA
#define LINE1_REF2_COMP1_INM_Pin GPIO_PIN_4
#define LINE1_REF2_COMP1_INM_GPIO_Port GPIOC
#define LINE1_REF_COM1_INM_Pin GPIO_PIN_1
#define LINE1_REF_COM1_INM_GPIO_Port GPIOB
#define LINE1_VIDEO_COMP1_INP_Pin GPIO_PIN_2
#define LINE1_VIDEO_COMP1_INP_GPIO_Port GPIOB
#define LINE1_EOS_Pin GPIO_PIN_14
#define LINE1_EOS_GPIO_Port GPIOE
#define SW_U_Pin GPIO_PIN_15
#define SW_U_GPIO_Port GPIOE
#define U3_TX_Pin GPIO_PIN_10
#define U3_TX_GPIO_Port GPIOB
#define U3_RX_Pin GPIO_PIN_11
#define U3_RX_GPIO_Port GPIOB
#define OV_CNT_Pin GPIO_PIN_12
#define OV_CNT_GPIO_Port GPIOB
#define S0_Pin GPIO_PIN_8
#define S0_GPIO_Port GPIOD
#define S1_Pin GPIO_PIN_9
#define S1_GPIO_Port GPIOD
#define S2_Pin GPIO_PIN_10
#define S2_GPIO_Port GPIOD
#define S3_Pin GPIO_PIN_11
#define S3_GPIO_Port GPIOD
#define TRAY_DET_Pin GPIO_PIN_15
#define TRAY_DET_GPIO_Port GPIOD
#define VBUS_Pin GPIO_PIN_9
#define VBUS_GPIO_Port GPIOA
#define USBM_Pin GPIO_PIN_11
#define USBM_GPIO_Port GPIOA
#define USBP_Pin GPIO_PIN_12
#define USBP_GPIO_Port GPIOA
#define SWDCLK_Pin GPIO_PIN_13
#define SWDCLK_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_14
#define SWDIO_GPIO_Port GPIOA
#define MUX1_Pin GPIO_PIN_0
#define MUX1_GPIO_Port GPIOD
#define MUX2_Pin GPIO_PIN_1
#define MUX2_GPIO_Port GPIOD
#define MUX3_Pin GPIO_PIN_2
#define MUX3_GPIO_Port GPIOD
#define MUX4_Pin GPIO_PIN_3
#define MUX4_GPIO_Port GPIOD
#define MUX5_Pin GPIO_PIN_4
#define MUX5_GPIO_Port GPIOD
#define MUX6_Pin GPIO_PIN_5
#define MUX6_GPIO_Port GPIOD
#define LED_BLUE_TIM3_CH2_Pin GPIO_PIN_5
#define LED_BLUE_TIM3_CH2_GPIO_Port GPIOB
#define U1_TX_Pin GPIO_PIN_6
#define U1_TX_GPIO_Port GPIOB
#define U1_RX_Pin GPIO_PIN_7
#define U1_RX_GPIO_Port GPIOB
#define LINE1_CLK_TIM17_CH1_Pin GPIO_PIN_9
#define LINE1_CLK_TIM17_CH1_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_0
#define LED_R_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_1
#define LED_B_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
