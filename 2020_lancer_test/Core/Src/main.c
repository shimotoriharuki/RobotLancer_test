/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "INA260.h"
#include "HAL_SDcard_lib.h"
#include "ICM_20648.h"
#include "AQM0802.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DRIVE_MAX_RATIO 1200

#define BUFF_SIZE 128
#define OVER_WRITE 0
#define ADD_WRITE 1
#define DATA_SIZE 10000

#define AD_NUM 9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
float current;
float voltage;

_Bool error = 0;
int timer, timer2, LCDtimer;
uint16_t ad[AD_NUM];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM12_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//*********************************************************
//SWO printf
//*********************************************************
int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for(DataIdx=0; DataIdx<len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

//************************************************************************/
//*	Drive motor1 controller
//************************************************************************/
void DriveMotor1_ctrl(float power){
	float PulseWidth = 0;   //PWMpulse

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);	//Drive1_SR
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, DRIVE_MAX_RATIO);	//Drive1_PWML

	if(power < 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);	//Drive1_PHASE
		PulseWidth = power * -1;
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);	//Drive1_PHASE
		PulseWidth = power;
	}

	if(PulseWidth > DRIVE_MAX_RATIO)	PulseWidth = DRIVE_MAX_RATIO;

	if(error == 1){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);	//Drive1_PWMH
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PulseWidth);	//Drive1_PWMH
	}

}

//************************************************************************/
//*	Drive motor2 controller
//************************************************************************/
void DriveMotor2_ctrl(float power){
	float PulseWidth = 0;   //PWMpulse

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);	//Drive2_SR
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, DRIVE_MAX_RATIO);	//Drive2_PWML

	if(power < 0){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	//Drive2_PHASE
		PulseWidth = power * -1;
	}
	else{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	//Drive2_PHASE
		PulseWidth = power;
	}

	if(PulseWidth > DRIVE_MAX_RATIO)	PulseWidth = DRIVE_MAX_RATIO;

	if(error == 1){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);	//Drive2_PWMH
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PulseWidth);	//Drive2_PWMH
	}

}

//************************************************************************/
//*	Drive motor3 controller
//************************************************************************/
void DriveMotor3_ctrl(float power){
	float PulseWidth = 0;   //PWMpulse

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);	//Drive3_SR
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, DRIVE_MAX_RATIO);	//Drive3_PWML

	if(power < 0){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);	//Drive3_PHASE
		PulseWidth = power * -1;
	}
	else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);	//Drive3_PHASE
		PulseWidth = power;
	}

	if(PulseWidth > DRIVE_MAX_RATIO)	PulseWidth = DRIVE_MAX_RATIO;

	if(error == 1){
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);	//Drive3_PWMH
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PulseWidth);	//Drive3_PWMH
	}

}

//************************************************************************/
//*	Drive motor4 controller
//************************************************************************/
void DriveMotor4_ctrl(float power){
	float PulseWidth = 0;   //PWMpulse

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);	//Drive4_SR
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, DRIVE_MAX_RATIO);	//Drive4_PWML

	if(power < 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);	//Drive4_PHASE
		PulseWidth = power * -1;
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);	//Drive4_PHASE
		PulseWidth = power;
	}

	if(PulseWidth > DRIVE_MAX_RATIO)	PulseWidth = DRIVE_MAX_RATIO;

	if(error == 1){
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);	//Drive4_PWMH
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PulseWidth);	//Drive4_PWMH
	}

}

//************************************************************************/
//*	Servo motor1 controller
//************************************************************************/
void ServoMotor1_ctrl(float power){
	float PulseWidth = 0;   //PWMpulse

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);	//Servo1_SR
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRIVE_MAX_RATIO);	//Servo1_PWML

	if(power < 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);	//Servo1_PHASE
		PulseWidth = power * -1;
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, DRIVE_MAX_RATIO);	//Servo1_PHASE
		PulseWidth = power;
	}

	if(PulseWidth > DRIVE_MAX_RATIO)	PulseWidth = DRIVE_MAX_RATIO;

	if(error == 1){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//Servo1_PWMH
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PulseWidth);	//Servo1_PWMH
	}

}

//************************************************************************/
//*	Servo motor2 controller
//************************************************************************/
void ServoMotor2_ctrl(float power){
	float PulseWidth = 0;   //PWMpulse

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);	//Servo2_SR
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, DRIVE_MAX_RATIO);	//Servo2_PWML

	if(power < 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//Servo2_PHASE
		PulseWidth = power * -1;
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRIVE_MAX_RATIO);	//Servo2_PHASE
		PulseWidth = power;
	}

	if(PulseWidth > DRIVE_MAX_RATIO)	PulseWidth = DRIVE_MAX_RATIO;

	if(error == 1){
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);	//Servo2_PWMH
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PulseWidth);	//Servo2_PWMH
	}

}
//************************************************************************/
//*	Full color LED1 function
//************************************************************************/
void LED1(_Bool red, _Bool green, _Bool blue){
	if(red)	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
	else	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);

	if(green)	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	else	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);

	if(blue)	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	else	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);


}
//************************************************************************/
//*	Full color LED2 function
//************************************************************************/
void LED2(_Bool red, _Bool green, _Bool blue){
	if(red)	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
	else	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);

	if(green)	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
	else	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);

	if(blue)	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	else	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);


}
//************************************************************************/
//Cross SW function
//************************************************************************/
_Bool C_SW(char num){
	_Bool ret = 0;

	switch(num){
	case 1:
		ret = !HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7);
		break;

	case 2:
		ret = !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
		break;

	case 3:
		ret = !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9);
		break;

 	case 4:
		ret = !HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6);
		break;

	case 5:
		ret = !HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0);
		break;
	}

	return ret;

}

//************************************************************************/
//*	Tact SW function
//************************************************************************/
uint8_t T_SW(void){
	uint8_t ret = 0;

	if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5)) ret |= 0x01;
	if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)) ret |= 0x02;
	if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)) ret |= 0x04;

	return ret;
}
//************************************************************************/
//*	Rotary SW function
//************************************************************************/
uint8_t R_SW(void){
	uint8_t ret = 0;

	if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)) ret |= 0x01;
	if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)) ret |= 0x02;
	if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)) ret |= 0x04;
	if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)) ret |= 0x08;

	return ret;
}

//************************************************************************/
//*	イニシャライズ
//************************************************************************/
void init(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	//Drive1_PWML
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	//Drive1_PWMH

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	//Drive2_PWML
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);	//Drive2_PWMH

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);	//Drive3_PWML
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);	//Drive3_PWMH

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	//Drive4_PWML
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	//Drive4_PWMH

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	//Servo1_PHASE
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	//Servo1_PWML
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	//Servo1_PWMH

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	//Servo2_PHASE
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);	//Servo2_PWML
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);	//Servo2_PWMH

	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);	//BZ_PWM
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);	//Sensor_PWM
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 1000);

	HAL_TIM_Base_Start_IT(&htim6);	//	Timer interrupt start

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);	//	Encoder start

	HAL_ADC_Start_DMA(&hadc1,  (uint32_t *)ad, AD_NUM);	//ADC Start

	lcd_init();
	IMU_init();
	INA260_init();

	if(sd_mount() == 1){
		printf("mount success\r\n");
	}
	else{
		printf("error\r\n");
	}


}

//************************************************************************/
//*	//************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	read_gyro_data();
	read_accel_data();
	timer++;
	timer2++;
	LCDtimer++;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
	//
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int data[DATA_SIZE];
	int temp[DATA_SIZE];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SDIO_SD_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_SPI4_Init();
  MX_TIM12_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

//*****************LCD check********************//
	  if(LCDtimer > 100){
		  LCDtimer = 0;
		  lcd_clear();
		  lcd_locate(0,0);
		  lcd_printf("Rsw: %2d", R_SW());
		  lcd_locate(0,1);
		  lcd_printf("Tsw: %2d", T_SW());
	  }

//*****************Various check********************//
	  if(C_SW(1)){	//IMU & Encoder check
		  LED1(0, 0, 1);
		  LED2(1, 1, 0);
		  printf("xa:%5d ya:%5d za:%5d xg:%5d yg:%5d zg:%5d enc:%5d\r\n", xa, ya, za, xg, yg, zg, (int)TIM2 -> CNT);

	  }
	  else if(C_SW(2)){	//SDcard check
		  LED1(1, 0, 0);
		  LED2(0, 1, 1);

		  for(int i = 0; i < DATA_SIZE; i++){
			  data[i] = i;
		  }
		  f_unlink("sdio/write1.txt");
		  f_unlink("sdio/write2.txt");

		  timer = 0;
		  sd_write_array_int("sdio", "write1.txt", DATA_SIZE, data, ADD_WRITE);
		  sd_read_array_int("sdio", "write1.txt", DATA_SIZE, temp);
		  sd_write_array_int("sdio", "write2.txt", DATA_SIZE, temp, ADD_WRITE);

		  printf("%4d round timer: %6d\r\n", DATA_SIZE, timer);

	  }
	  else if(C_SW(3)){	//ADC check
		  LED1(0, 1, 1);
		  LED2(1, 0, 0);
		  printf("ad0:%5d ad1:%5d ad2:%5d ad3:%5d ad4:%5d ad5:%5d ad6:%5d ad7:%5d ad8:%5d\r\n", (int)ad[0], (int)ad[1], (int)ad[2], (int)ad[3], (int)ad[4], (int)ad[5], (int)ad[6], (int)ad[7], (int)ad[8]);

	  }
	  else if(C_SW(4)){	//Voltage & Current sensor check
		  LED1(1, 0, 1);
		  LED2(0, 1, 0);

		  current = INA260_read(0x01) * 0.00125;
		  voltage = INA260_read(0x02) * 0.00125;
		  printf("V: %5.2f I: %5.2f\r\n", voltage, current);

	  }
	  else if(C_SW(5)){	//BZ check
		  LED1(1, 1, 0);
		  LED2(0, 0, 1);

		  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 7000);
	  }

	  else{
		  LED1(0, 0, 0);
		  LED2(1, 1, 1);
		  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
	  }


//*****************Motor check********************//
	  if(timer2 > 2000){
		  timer2 = 0;
	  }
	  else if(timer2 > 1000){
		  DriveMotor1_ctrl(-400);
		  DriveMotor2_ctrl(0);
		  DriveMotor3_ctrl(0);
		  DriveMotor4_ctrl(0);

		  ServoMotor1_ctrl(-100);
		  ServoMotor2_ctrl(-200);
	  }
	  else {
		  DriveMotor1_ctrl(400);
		  DriveMotor2_ctrl(400);
		  DriveMotor3_ctrl(200);
		  DriveMotor4_ctrl(-200);

		  ServoMotor1_ctrl(100);
		  ServoMotor2_ctrl(200);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1200;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 89;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 2;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1200;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 2;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 60000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 89;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_OUT_LED_B_Pin|GPIO_OUT_LED_R_Pin|GIPO_OUT_DRIVE2_P_Pin|GIPO_OUT_DRIVE_2SR_Pin 
                          |GIPO_OUT_SERVO1_SR_Pin|GPIO_OUT_LED_RE12_Pin|GPIO_OUT_LED_G_Pin|GPIO_OUT_LED_BE15_Pin 
                          |GPIO_OUT_LED_GE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_OUT_CS_IMU_Pin|GIPO_OUT_DRIVE1_SR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GIPO_OUT_DRIVE1_P_GPIO_Port, GIPO_OUT_DRIVE1_P_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_OUT_DRIVE_3P_Pin|GPIO_OUT_DRIVE_3SR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_OUT_SERVO2_SR_Pin|GPIO_OUT_DRIVE4_P_Pin|GPIO_OUT_DRIVE4_SR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO_OUT_LED_B_Pin GPIO_OUT_LED_R_Pin GIPO_OUT_DRIVE2_P_Pin GIPO_OUT_DRIVE_2SR_Pin 
                           GIPO_OUT_SERVO1_SR_Pin GPIO_OUT_LED_RE12_Pin GPIO_OUT_LED_G_Pin GPIO_OUT_LED_BE15_Pin 
                           GPIO_OUT_LED_GE1_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT_LED_B_Pin|GPIO_OUT_LED_R_Pin|GIPO_OUT_DRIVE2_P_Pin|GIPO_OUT_DRIVE_2SR_Pin 
                          |GIPO_OUT_SERVO1_SR_Pin|GPIO_OUT_LED_RE12_Pin|GPIO_OUT_LED_G_Pin|GPIO_OUT_LED_BE15_Pin 
                          |GPIO_OUT_LED_GE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_OUT_CS_IMU_Pin GIPO_OUT_DRIVE1_SR_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT_CS_IMU_Pin|GIPO_OUT_DRIVE1_SR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GIPO_OUT_DRIVE1_P_Pin */
  GPIO_InitStruct.Pin = GIPO_OUT_DRIVE1_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GIPO_OUT_DRIVE1_P_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_IN_R_SW4_Pin GPIO_IN_R_SW8_Pin GPIO_IN_R_SW2_Pin GPIO_IN_R_SW1_Pin */
  GPIO_InitStruct.Pin = GPIO_IN_R_SW4_Pin|GPIO_IN_R_SW8_Pin|GPIO_IN_R_SW2_Pin|GPIO_IN_R_SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_OUT_DRIVE_3P_Pin GPIO_OUT_DRIVE_3SR_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT_DRIVE_3P_Pin|GPIO_OUT_DRIVE_3SR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_OUT_SERVO2_SR_Pin GPIO_OUT_DRIVE4_P_Pin GPIO_OUT_DRIVE4_SR_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT_SERVO2_SR_Pin|GPIO_OUT_DRIVE4_P_Pin|GPIO_OUT_DRIVE4_SR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_IN_SD_INSERT_Pin */
  GPIO_InitStruct.Pin = GPIO_IN_SD_INSERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_IN_SD_INSERT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_IN_T_SW3_Pin GPIO_IN_T_SW2_Pin GPIO_IN_T_SW1_Pin GPIO_IN_C_SW4_Pin 
                           GPIO_IN_C_SW1_Pin */
  GPIO_InitStruct.Pin = GPIO_IN_T_SW3_Pin|GPIO_IN_T_SW2_Pin|GPIO_IN_T_SW1_Pin|GPIO_IN_C_SW4_Pin 
                          |GPIO_IN_C_SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_IN_C_SW2_Pin GPIO_IN_C_SW3_Pin */
  GPIO_InitStruct.Pin = GPIO_IN_C_SW2_Pin|GPIO_IN_C_SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_IN_C_SW5_Pin */
  GPIO_InitStruct.Pin = GPIO_IN_C_SW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_IN_C_SW5_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
