/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
/* USART comms to host PC */
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
/* Interrupt pin for TOF sensors (shared) */
#define VL53L1X_INT_Pin GPIO_PIN_4
#define VL53L1X_INT_GPIO_Port GPIOA

/* XSHUT pin for TOF sensors (individual) */
//#define TOF_0_XSHUT_Pin GPIO_PIN_8
//#define TOF_0_XSHUT_GPIO_Port GPIOA
//#define TOF_1_XSHUT_Pin GPIO_PIN_10
//#define TOF_1_XSHUT_GPIO_Port GPIOB
//#define TOF_2_XSHUT_Pin GPIO_PIN_4
//#define TOF_2_XSHUT_GPIO_Port GPIOB
//#define TOF_3_XSHUT_Pin GPIO_PIN_5
//#define TOF_3_XSHUT_GPIO_Port GPIOB
//#define TOF_4_XSHUT_Pin GPIO_PIN_3
//#define TOF_4_XSHUT_GPIO_Port GPIOB
//#define TOF_5_XSHUT_Pin GPIO_PIN_10
//#define TOF_5_XSHUT_GPIO_Port GPIOA

#define TOF_0_XSHUT_Pin GPIO_PIN_10
#define TOF_0_XSHUT_GPIO_Port GPIOC
#define TOF_1_XSHUT_Pin GPIO_PIN_12
#define TOF_1_XSHUT_GPIO_Port GPIOC
#define TOF_2_XSHUT_Pin GPIO_PIN_15
#define TOF_2_XSHUT_GPIO_Port GPIOA
#define TOF_3_XSHUT_Pin GPIO_PIN_5
#define TOF_3_XSHUT_GPIO_Port GPIOB
#define TOF_4_XSHUT_Pin GPIO_PIN_3
#define TOF_4_XSHUT_GPIO_Port GPIOB
#define TOF_5_XSHUT_Pin GPIO_PIN_10
#define TOF_5_XSHUT_GPIO_Port GPIOA
/* LED 2 */
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define NUM_TOFS (6)

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
