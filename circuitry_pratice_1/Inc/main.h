/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define userButton_Pin GPIO_PIN_0
#define userButton_GPIO_Port GPIOA
#define userButton_EXTI_IRQn EXTI0_1_IRQn
#define button1_Pin GPIO_PIN_1
#define button1_GPIO_Port GPIOA
#define button1_EXTI_IRQn EXTI0_1_IRQn
#define pwmChannel1_Pin GPIO_PIN_2
#define pwmChannel1_GPIO_Port GPIOA
#define pwmChannel2_Pin GPIO_PIN_3
#define pwmChannel2_GPIO_Port GPIOA
#define DacOut1_Pin GPIO_PIN_4
#define DacOut1_GPIO_Port GPIOA
#define redLed_Pin GPIO_PIN_5
#define redLed_GPIO_Port GPIOA
#define button2_Pin GPIO_PIN_2
#define button2_GPIO_Port GPIOB
#define button2_EXTI_IRQn EXTI2_3_IRQn
#define button3_Pin GPIO_PIN_3
#define button3_GPIO_Port GPIOB
#define button3_EXTI_IRQn EXTI2_3_IRQn
#define greenLed_Pin GPIO_PIN_4
#define greenLed_GPIO_Port GPIOB
#define button5_Pin GPIO_PIN_6
#define button5_GPIO_Port GPIOB
#define button5_EXTI_IRQn EXTI4_15_IRQn
#define button4_Pin GPIO_PIN_7
#define button4_GPIO_Port GPIOB
#define button4_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */
void buttonPress(void);
void pwmInterrupt(void);
void pwmSetvalue(uint16_t value);
static int b0, b1, b2, b3, b4, b5;
static int period = 0;
static const uint16_t sine_wave_array[32] = {2047, 1648, 1264, 910, 600,  345,   
																						156, 39,  0,  39,  156,  345,  
																						600, 910, 1264, 1648, 2048, 2447,  
																						2831, 3185, 3495, 3750, 3939, 4056,  
																						4095, 4056, 3939, 3750, 3495, 3185,  
																						2831, 2447}; 
/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
