/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BATT_STATUS1_Pin GPIO_PIN_13
#define BATT_STATUS1_GPIO_Port GPIOC
#define UART2_TX_Pin GPIO_PIN_0
#define UART2_TX_GPIO_Port GPIOA
#define UART2_RX_Pin GPIO_PIN_1
#define UART2_RX_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_2
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_3
#define GPS_RX_GPIO_Port GPIOA
#define GPIO1_Pin GPIO_PIN_4
#define GPIO1_GPIO_Port GPIOA
#define GPS_RST_Pin GPIO_PIN_6
#define GPS_RST_GPIO_Port GPIOA
#define GPIO3_Pin GPIO_PIN_0
#define GPIO3_GPIO_Port GPIOB
#define BATT_GPOUT_Pin GPIO_PIN_1
#define BATT_GPOUT_GPIO_Port GPIOB
#define GPIO4_Pin GPIO_PIN_2
#define GPIO4_GPIO_Port GPIOB
#define MODEM_TX_Pin GPIO_PIN_10
#define MODEM_TX_GPIO_Port GPIOB
#define MODEM_RX_Pin GPIO_PIN_11
#define MODEM_RX_GPIO_Port GPIOB
#define MODEM_RST_Pin GPIO_PIN_12
#define MODEM_RST_GPIO_Port GPIOB
#define GPIO7_Pin GPIO_PIN_15
#define GPIO7_GPIO_Port GPIOB
#define GPIO2_Pin GPIO_PIN_8
#define GPIO2_GPIO_Port GPIOA
#define UART1_TX_Pin GPIO_PIN_9
#define UART1_TX_GPIO_Port GPIOA
#define UART1_RX_Pin GPIO_PIN_10
#define UART1_RX_GPIO_Port GPIOA
#define UART2_RTS_Pin GPIO_PIN_15
#define UART2_RTS_GPIO_Port GPIOA
#define GPIO5_Pin GPIO_PIN_3
#define GPIO5_GPIO_Port GPIOB
#define UART2_CTS_Pin GPIO_PIN_7
#define UART2_CTS_GPIO_Port GPIOB
#define BATT_STATUS2_Pin GPIO_PIN_8
#define BATT_STATUS2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
