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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void incrementLowerDisplay();

void switchToggle(GPIO_TypeDef *GPIOx_Switch,
										 uint16_t GPIO_Pin_Switch,
										 GPIO_TypeDef *GPIOx_LED,
										 uint16_t GPIO_Pin_LED,
										 uint8_t *name);

void printTemp();

void setDisplay(uint8_t display);

void displayOneNumber(uint16_t number, uint8_t display);

void setDecimal(uint8_t flag);

void setNumber(uint8_t number);

void toggleAndPrintGpioState(GPIO_TypeDef*, uint16_t, uint8_t*);

void delayUs(uint32_t us);

int __io_putchar(int);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DisplayOne_Pin GPIO_PIN_4
#define DisplayOne_GPIO_Port GPIOE
#define Middle_Pin GPIO_PIN_5
#define Middle_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DisplayTwo_Pin GPIO_PIN_0
#define DisplayTwo_GPIO_Port GPIOF
#define TopRight_Pin GPIO_PIN_1
#define TopRight_GPIO_Port GPIOF
#define BottomRight_Pin GPIO_PIN_2
#define BottomRight_GPIO_Port GPIOF
#define Decimal_Pin GPIO_PIN_8
#define Decimal_GPIO_Port GPIOF
#define BottomMid_Pin GPIO_PIN_9
#define BottomMid_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define Trig_Pin GPIO_PIN_1
#define Trig_GPIO_Port GPIOA
#define BlueLED_Pin GPIO_PIN_4
#define BlueLED_GPIO_Port GPIOA
#define YellowLED_Pin GPIO_PIN_5
#define YellowLED_GPIO_Port GPIOA
#define RedLED_Pin GPIO_PIN_6
#define RedLED_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define TopMid_Pin GPIO_PIN_0
#define TopMid_GPIO_Port GPIOG
#define BottomLeft_Pin GPIO_PIN_1
#define BottomLeft_GPIO_Port GPIOG
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define USB_FS_DM_Pin GPIO_PIN_11
#define USB_FS_DM_GPIO_Port GPIOA
#define USB_FS_DP_Pin GPIO_PIN_12
#define USB_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TopLeft_Pin GPIO_PIN_0
#define TopLeft_GPIO_Port GPIOD
#define DisplayThree_Pin GPIO_PIN_1
#define DisplayThree_GPIO_Port GPIOD
#define SwtichTwo_Pin GPIO_PIN_6
#define SwtichTwo_GPIO_Port GPIOD
#define SwitchOne_Pin GPIO_PIN_7
#define SwitchOne_GPIO_Port GPIOD
#define DisplayFour_Pin GPIO_PIN_9
#define DisplayFour_GPIO_Port GPIOG
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define Switch1_Pin GPIO_PIN_12
#define Switch1_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_1
#define LED_YELLOW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#define Blinky 0
#define Switch 1
#define printLED 1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
