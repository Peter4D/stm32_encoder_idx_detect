/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "sw_timer.h"
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
#define TIM2_CNT 100
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define out0_Pin GPIO_PIN_0
#define out0_GPIO_Port GPIOA
#define out1_Pin GPIO_PIN_1
#define out1_GPIO_Port GPIOA
#define out2_Pin GPIO_PIN_2
#define out2_GPIO_Port GPIOA
#define out3_Pin GPIO_PIN_3
#define out3_GPIO_Port GPIOA
#define END_SW0_Pin GPIO_PIN_12
#define END_SW0_GPIO_Port GPIOB
#define END_SW1_Pin GPIO_PIN_13
#define END_SW1_GPIO_Port GPIOB
#define END_SW2_Pin GPIO_PIN_14
#define END_SW2_GPIO_Port GPIOB
#define END_SW3_Pin GPIO_PIN_15
#define END_SW3_GPIO_Port GPIOB
#define idx_int0_Pin GPIO_PIN_6
#define idx_int0_GPIO_Port GPIOB
#define idx_int0_EXTI_IRQn EXTI9_5_IRQn
#define idx_int1_Pin GPIO_PIN_7
#define idx_int1_GPIO_Port GPIOB
#define idx_int1_EXTI_IRQn EXTI9_5_IRQn
#define idx_int2_Pin GPIO_PIN_8
#define idx_int2_GPIO_Port GPIOB
#define idx_int2_EXTI_IRQn EXTI9_5_IRQn
#define idx_int3_Pin GPIO_PIN_9
#define idx_int3_GPIO_Port GPIOB
#define idx_int3_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

#define LED_TICK_TM     (2000u) // ms

#define TASK_PERIODE    (2000u)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
