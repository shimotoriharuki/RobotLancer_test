/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define SPI4_SCK_IMU_Pin GPIO_PIN_2
#define SPI4_SCK_IMU_GPIO_Port GPIOE
#define GPIO_OUT_LED_B_Pin GPIO_PIN_3
#define GPIO_OUT_LED_B_GPIO_Port GPIOE
#define GPIO_OUT_LED_R_Pin GPIO_PIN_4
#define GPIO_OUT_LED_R_GPIO_Port GPIOE
#define SPI4_MISO_IMU_Pin GPIO_PIN_5
#define SPI4_MISO_IMU_GPIO_Port GPIOE
#define SPI4_MOSI_IMU_Pin GPIO_PIN_6
#define SPI4_MOSI_IMU_GPIO_Port GPIOE
#define GPIO_OUT_CS_IMU_Pin GPIO_PIN_13
#define GPIO_OUT_CS_IMU_GPIO_Port GPIOC
#define TIM2_ENCORDER_A_Pin GPIO_PIN_0
#define TIM2_ENCORDER_A_GPIO_Port GPIOA
#define TIM2_ENCORDER_B_Pin GPIO_PIN_1
#define TIM2_ENCORDER_B_GPIO_Port GPIOA
#define TIM3_DRIVE1_L_Pin GPIO_PIN_6
#define TIM3_DRIVE1_L_GPIO_Port GPIOA
#define TIM3_DRIVE1_H_Pin GPIO_PIN_7
#define TIM3_DRIVE1_H_GPIO_Port GPIOA
#define GIPO_OUT_DRIVE1_SR_Pin GPIO_PIN_5
#define GIPO_OUT_DRIVE1_SR_GPIO_Port GPIOC
#define TIM3_DRIVE2_L_Pin GPIO_PIN_0
#define TIM3_DRIVE2_L_GPIO_Port GPIOB
#define TIM3_DRIVE2_H_Pin GPIO_PIN_1
#define TIM3_DRIVE2_H_GPIO_Port GPIOB
#define GIPO_OUT_DRIVE1_P_Pin GPIO_PIN_2
#define GIPO_OUT_DRIVE1_P_GPIO_Port GPIOB
#define GIPO_OUT_DRIVE2_P_Pin GPIO_PIN_7
#define GIPO_OUT_DRIVE2_P_GPIO_Port GPIOE
#define GIPO_OUT_DRIVE_2SR_Pin GPIO_PIN_8
#define GIPO_OUT_DRIVE_2SR_GPIO_Port GPIOE
#define TIM1_SERVO1_H_Pin GPIO_PIN_9
#define TIM1_SERVO1_H_GPIO_Port GPIOE
#define GIPO_OUT_SERVO1_SR_Pin GPIO_PIN_10
#define GIPO_OUT_SERVO1_SR_GPIO_Port GPIOE
#define TIM1_SERVO1_L_Pin GPIO_PIN_11
#define TIM1_SERVO1_L_GPIO_Port GPIOE
#define GPIO_OUT_LED_RE12_Pin GPIO_PIN_12
#define GPIO_OUT_LED_RE12_GPIO_Port GPIOE
#define GPIO_OUT_LED_G_Pin GPIO_PIN_13
#define GPIO_OUT_LED_G_GPIO_Port GPIOE
#define TIM1_SERVO1_P_Pin GPIO_PIN_14
#define TIM1_SERVO1_P_GPIO_Port GPIOE
#define GPIO_OUT_LED_BE15_Pin GPIO_PIN_15
#define GPIO_OUT_LED_BE15_GPIO_Port GPIOE
#define GPIO_IN_R_SW4_Pin GPIO_PIN_10
#define GPIO_IN_R_SW4_GPIO_Port GPIOB
#define GPIO_IN_R_SW8_Pin GPIO_PIN_12
#define GPIO_IN_R_SW8_GPIO_Port GPIOB
#define GPIO_IN_R_SW2_Pin GPIO_PIN_13
#define GPIO_IN_R_SW2_GPIO_Port GPIOB
#define TIM12_LINE_SENSOR_Pin GPIO_PIN_14
#define TIM12_LINE_SENSOR_GPIO_Port GPIOB
#define GPIO_IN_R_SW1_Pin GPIO_PIN_15
#define GPIO_IN_R_SW1_GPIO_Port GPIOB
#define GPIO_OUT_DRIVE_3P_Pin GPIO_PIN_10
#define GPIO_OUT_DRIVE_3P_GPIO_Port GPIOD
#define GPIO_OUT_DRIVE_3SR_Pin GPIO_PIN_11
#define GPIO_OUT_DRIVE_3SR_GPIO_Port GPIOD
#define TIM4_DRIVE3_L_Pin GPIO_PIN_12
#define TIM4_DRIVE3_L_GPIO_Port GPIOD
#define TIM4_DRIVE3_H_Pin GPIO_PIN_13
#define TIM4_DRIVE3_H_GPIO_Port GPIOD
#define TIM4_DRIVE4_L_Pin GPIO_PIN_14
#define TIM4_DRIVE4_L_GPIO_Port GPIOD
#define TIM4_DRIVE4_H_Pin GPIO_PIN_15
#define TIM4_DRIVE4_H_GPIO_Port GPIOD
#define TIM8_SERVO2_L_Pin GPIO_PIN_6
#define TIM8_SERVO2_L_GPIO_Port GPIOC
#define TIM8_SERVO2_H_Pin GPIO_PIN_7
#define TIM8_SERVO2_H_GPIO_Port GPIOC
#define I2C3_SCL_LCD_Pin GPIO_PIN_8
#define I2C3_SCL_LCD_GPIO_Port GPIOA
#define GPIO_OUT_SERVO2_SR_Pin GPIO_PIN_9
#define GPIO_OUT_SERVO2_SR_GPIO_Port GPIOA
#define GPIO_OUT_DRIVE4_P_Pin GPIO_PIN_11
#define GPIO_OUT_DRIVE4_P_GPIO_Port GPIOA
#define GPIO_OUT_DRIVE4_SR_Pin GPIO_PIN_12
#define GPIO_OUT_DRIVE4_SR_GPIO_Port GPIOA
#define GPIO_IN_SD_INSERT_Pin GPIO_PIN_15
#define GPIO_IN_SD_INSERT_GPIO_Port GPIOA
#define GPIO_IN_T_SW3_Pin GPIO_PIN_3
#define GPIO_IN_T_SW3_GPIO_Port GPIOD
#define GPIO_IN_T_SW2_Pin GPIO_PIN_4
#define GPIO_IN_T_SW2_GPIO_Port GPIOD
#define GPIO_IN_T_SW1_Pin GPIO_PIN_5
#define GPIO_IN_T_SW1_GPIO_Port GPIOD
#define GPIO_IN_C_SW4_Pin GPIO_PIN_6
#define GPIO_IN_C_SW4_GPIO_Port GPIOD
#define GPIO_IN_C_SW1_Pin GPIO_PIN_7
#define GPIO_IN_C_SW1_GPIO_Port GPIOD
#define I2C3_SDA_LCD_Pin GPIO_PIN_4
#define I2C3_SDA_LCD_GPIO_Port GPIOB
#define GPIO_IN_C_SW2_Pin GPIO_PIN_5
#define GPIO_IN_C_SW2_GPIO_Port GPIOB
#define I2C1_SCL_CURRENT_Pin GPIO_PIN_6
#define I2C1_SCL_CURRENT_GPIO_Port GPIOB
#define I2C1_SDA_CURRENT_Pin GPIO_PIN_7
#define I2C1_SDA_CURRENT_GPIO_Port GPIOB
#define TIM10_BZ_Pin GPIO_PIN_8
#define TIM10_BZ_GPIO_Port GPIOB
#define GPIO_IN_C_SW3_Pin GPIO_PIN_9
#define GPIO_IN_C_SW3_GPIO_Port GPIOB
#define GPIO_IN_C_SW5_Pin GPIO_PIN_0
#define GPIO_IN_C_SW5_GPIO_Port GPIOE
#define GPIO_OUT_LED_GE1_Pin GPIO_PIN_1
#define GPIO_OUT_LED_GE1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
