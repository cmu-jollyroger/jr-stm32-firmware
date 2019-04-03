/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
	* COPYRIGHT(c) 2019 Haowen Shi
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32xxx_hal.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "i2c_shared.h"
#include "vl53l1_api.h"
#include "X-NUCLEO-53L1A1.h"

#include "MeEncoderNew.h"
/* USER CODE END Includes */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/** @brief struct of all TOF sensors and their states */
struct {
	VL53L1_Dev_t dev;
	uint8_t valid;
	GPIO_TypeDef *port;
	uint16_t mask;
} sensors[NUM_TOFS];

VL53L1_DEV Dev = &sensors[0].dev;

/** @brief struct containing TOF XSHUT pin configurations */
struct _gpio {
	GPIO_TypeDef *port;
	uint16_t mask;
} xshut[NUM_TOFS] = {
	{TOF_0_XSHUT_GPIO_Port, TOF_0_XSHUT_Pin},
	{TOF_1_XSHUT_GPIO_Port, TOF_1_XSHUT_Pin},
	{TOF_2_XSHUT_GPIO_Port, TOF_2_XSHUT_Pin},
	{TOF_3_XSHUT_GPIO_Port, TOF_3_XSHUT_Pin},
	{TOF_4_XSHUT_GPIO_Port, TOF_4_XSHUT_Pin},
	{TOF_5_XSHUT_GPIO_Port, TOF_5_XSHUT_Pin},
};

int status;
volatile int IntCount;
#define isAutonomousExample 1  /* Allow to select either autonomous ranging or fast ranging example */
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

// Test functions
void DCMotorTest(void);
void AutonomousLowPowerRangingTest(void); /* see Autonomous ranging example implementation in USER CODE BEGIN 4 section */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
  return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		if (GPIO_Pin==VL53L1X_INT_Pin)
		{
			IntCount++;
		}
}

void VL53L1_TOF_Config() {
	/* Configure each TOF */
	GPIO_InitTypeDef GPIO_InitStruct;
	for (int i = 0; i < NUM_TOFS; i++) {
		/*Configure GPIO pin : TOF_n_XSHUT_Pin */
		GPIO_InitStruct.Pin = xshut[i].mask;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(xshut[i].port, &GPIO_InitStruct);
		
		sensors[i].port = xshut[i].port;
		sensors[i].mask = xshut[i].mask;
		sensors[i].valid = 0;
		sensors[i].dev.I2cHandle = &hi2c1;
		sensors[i].dev.I2cDevAddr = 0x29 << 1; // default address
	}
	
	printf("configured %d TOF sensors\r\n", NUM_TOFS);
}

/** @brief TOF initialization */
void VL53L1_TOF_Init() {
	VL53L1_Error err;
	
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	
	/* pull xshut to low for all sensors */
	for (int i = 0; i < NUM_TOFS; i++) {
		HAL_GPIO_WritePin(xshut[i].port, xshut[i].mask, GPIO_PIN_RESET);
	}
	
	for (int i = 0; i < NUM_TOFS; i++) {
		HAL_GPIO_WritePin(xshut[i].port, xshut[i].mask, GPIO_PIN_SET);
		HAL_Delay(10);
		
		Dev = &sensors[i].dev;
		Dev->I2cDevAddr = 0x29 << 1; // default address before configuring
		
		err = VL53L1_WaitDeviceBooted(Dev);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_WaitDeviceBooted(%d) failed\r\n", i);
			continue;
		} else {
			printf("[OK] VL53L1_WaitDeviceBooted(%d)\r\n", i);
		}
		
		err = VL53L1_SetDeviceAddress(Dev, (0x29 + i + 1) << 1);
		Dev->I2cDevAddr = (0x29 + i + 1) << 1; //change address even in case of error to reduce cross-talk
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_SetDeviceAddress(%d) failed\r\n", i);
			continue;
		} else {
			printf("[OK] VL53L1_SetDeviceAddress(%d)\r\n", i);
		}
		
		err = VL53L1_DataInit(Dev);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_DataInit(%d) failed\r\n", i);
			continue;
		} else {
			printf("[OK] VL53L1_DataInit(%d)\r\n", i);
		}
		
		err = VL53L1_StaticInit(Dev);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_StaticInit(%d) failed\r\n", i);
			continue;
		} else {
			printf("[OK] VL53L1_StaticInit(%d)\r\n", i);
		}
		
		err = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_SetDistanceMode(%d) failed\r\n", i);
			continue;
		} else {
			printf("[OK] VL53L1_SetDistanceMode(%d)\r\n", i);
		}
		
		err = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 500);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_SetInterMeasurementPeriodMilliSeconds(%d) failed\r\n", i);
			continue;
		} else {
			printf("[OK] VL53L1_SetInterMeasurementPeriodMilliSeconds(%d)\r\n", i);
		}
		
		err = VL53L1_StartMeasurement(Dev);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_StartMeasurement(%d) failed : %d\r\n", i, err);
			continue;
		} else {
			printf("[OK] VL53L1_StartMeasurement(%d)\r\n", i);
		}
		
		printf("[OK] Initialized TOF #%d\r\n", i);
		
		uint8_t byteData;
		uint16_t wordData;
		
		VL53L1_RdByte(Dev, 0x010F, &byteData);
		printf("   - VL53L1X Model_ID: %02X\r\n", byteData);
		VL53L1_RdByte(Dev, 0x0110, &byteData);
		printf("   - VL53L1X Module_Type: %02X\r\n", byteData);
		VL53L1_RdWord(Dev, 0x010F, &wordData);
		printf("   - VL53L1X: %02X\r\n", wordData);
		sensors[i].valid = 1;
	}
	
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
	
  uint8_t ToFSensor;

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

	MX_USART1_UART_Init();
	
	// this is for ST link virtual COM
  MX_USART2_UART_Init();

	MX_I2C1_Init();
	__HAL_RCC_I2C1_FORCE_RESET();
	HAL_Delay(2);
	__HAL_RCC_I2C1_RELEASE_RESET();

  XNUCLEO53L1A1_Init();
	
	/* Initialize all TOF sensors */
	VL53L1_TOF_Config();
	
	HAL_Delay(300);
	
	VL53L1_TOF_Init();
	
	HAL_Delay(300);
	
	MeEncoderNew_Init();
	
	/* USER CODE END 1 */
  
	/* USER CODE BEGIN 2 */
	
	printf("starting DC motor test...\r\n");
	DCMotorTest();

  printf("[info] Starting VL53L1X TOF Readings...\r\n");
	
	/* Sequential ranging loop */
	static VL53L1_RangingMeasurementData_t RangingData;
	while (1) {
		for (ToFSensor = 0; ToFSensor < NUM_TOFS; ToFSensor++) {
			Dev = &sensors[ToFSensor].dev;
			
			status = VL53L1_StartMeasurement(Dev);
		  status = VL53L1_WaitMeasurementDataReady(Dev);
			if(!status)
			{
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if(status==0){
					printf("%d,%d,%d,%d,%.2f,%.2f\r\n", ToFSensor, ToFSensor,RangingData.RangeStatus,RangingData.RangeMilliMeter,
									(RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			}
		}
	}
	
  /* USER CODE END 2 */
}

/** System Clock Configuration
*/
#ifdef STM32F401xE

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
#endif

#ifdef STM32L476xx
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
#endif
#ifdef STM32F401xE

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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

}
#endif
#ifdef STM32L476xx
/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

}
#endif
/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VL53L1X_INT_Pin */
  GPIO_InitStruct.Pin = VL53L1X_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VL53L1X_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

float speed_run = 70.;

void chassis_fwd() {
	move(700, speed_run, 0, 0);
	move(700, speed_run, 0, 1);
	move(-700, speed_run, 0, 2);
	move(-700, speed_run, 0, 3);
	HAL_Delay(3000);
}

void chassis_bkw() {
	move(-700, speed_run, 0, 0);
	move(-700, speed_run, 0, 1);
	move(700, speed_run, 0, 2);
	move(700, speed_run, 0, 3);
	HAL_Delay(3000);
}

void chassis_left() {
	move(700, speed_run, 0, 0);
	move(-700, speed_run, 0, 1);
	move(-700, speed_run, 0, 2);
	move(700, speed_run, 0, 3);
	HAL_Delay(3000);
}

void chassis_right() {
	move(-700, speed_run, 0, 0);
	move(700, speed_run, 0, 1);
	move(700, speed_run, 0, 2);
	move(-700, speed_run, 0, 3);
	HAL_Delay(3000);
}


/* DC Motor test */
void DCMotorTest(void) {
	float sP = .05;
	float sI = 0.01;
	float sD = 0.1;
	
	float pP = 0.1;
	float pI = 0.;
	float pD = 0.;
	
	setSpeedPID(sP, sI, sD, 0);
	setSpeedPID(sP, sI, sD, 1);
	setSpeedPID(sP, sI, sD, 2);
	setSpeedPID(sP, sI, sD, 3);
	
	setPosPID(pP, pI, pD, 0);
	setPosPID(pP, pI, pD, 1);
	setPosPID(pP, pI, pD, 2);
	setPosPID(pP, pI, pD, 3);
	
	getSpeedPID(&sP, &sI, &sD, 0);
	getPosPID(&pP, &pI, &pD, 0);
	
	printf("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", sP, sI, sD, pP, pI, pD);
	
//	setSpeedPID(.05, 0, .1, 0);
//	setSpeedPID(.05, 0, .1, 1);
//	setSpeedPID(.05, 0, .1, 2);
//	setSpeedPID(.05, 0, .1, 3);
//	
//	setPosPID(.1, .1, .3, 0);
//	setPosPID(.1, .1, .3, 1);
//	setPosPID(.1, .1, .3, 2);
//	setPosPID(.1, .1, .3, 3);

//	while (1) {
//		runSpeed(70, 1, 0);
//		runSpeed(70, 1, 1);
//		runSpeed(70, 1, 2);
//		runSpeed(70, 1, 3);
//	}
	
	while (1) {
		chassis_fwd();
		printf("fwd\r\n");
		chassis_left();
		printf("left\r\n");
		chassis_right();
		printf("right\r\n");
		chassis_bkw();
		printf("bkw\r\n");
		
		//runTurns(2, 150, 1, 0);
		//runTurns(2, 150, 1, 1);
		//runSpeed(150, 1, 0);
		//runSpeed(150, 1, 1);
		//HAL_Delay(3000);
	}
}

/* TOF Autonomous ranging loop*/
void AutonomousLowPowerRangingTest(void)
{
  static VL53L1_RangingMeasurementData_t RangingData;
  printf("Autonomous Ranging Test\r\n");

	if(status){
		printf("VL53L1_StartMeasurement failed \r\n");
		while(1);
	}	
	if (isInterrupt){
		do // interrupt mode
		{
		 __WFI();
		 if(IntCount !=0 ){
				IntCount=0;
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if(status==0){
					printf("%d,%d,%.2f,%.2f\r\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
									RangingData.SignalRateRtnMegaCps/65536.0,RangingData.AmbientRateRtnMegaCps/65336.0);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			}
		}
		while(1);
	}
	else{
		do // polling mode
		{
		  status = VL53L1_WaitMeasurementDataReady(Dev);
			if(!status)
			{
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if(status==0){
					printf("%d,%d,%.2f,%.2f\r\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
									(RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			}
		}
		while (1);
	}
//  return status;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
