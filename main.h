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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define TXC_Pin GPIO_PIN_13
#define TXC_GPIO_Port GPIOC
#define RXC_Pin GPIO_PIN_14
#define RXC_GPIO_Port GPIOC
#define TXD_Pin GPIO_PIN_15
#define TXD_GPIO_Port GPIOC
#define TXD0_Pin GPIO_PIN_0
#define TXD0_GPIO_Port GPIOA
#define TXD1_Pin GPIO_PIN_1
#define TXD1_GPIO_Port GPIOA
#define TXD2_Pin GPIO_PIN_2
#define TXD2_GPIO_Port GPIOA
#define TXD3_Pin GPIO_PIN_3
#define TXD3_GPIO_Port GPIOA
#define TXD4_Pin GPIO_PIN_4
#define TXD4_GPIO_Port GPIOA
#define TXD5_Pin GPIO_PIN_5
#define TXD5_GPIO_Port GPIOA
#define TXD6_Pin GPIO_PIN_6
#define TXD6_GPIO_Port GPIOA
#define TXD7_Pin GPIO_PIN_7
#define TXD7_GPIO_Port GPIOA
#define RXD_Pin GPIO_PIN_0
#define RXD_GPIO_Port GPIOB
#define RXD10_Pin GPIO_PIN_10
#define RXD10_GPIO_Port GPIOB
#define RXD11_Pin GPIO_PIN_11
#define RXD11_GPIO_Port GPIOB
#define RXD12_Pin GPIO_PIN_12
#define RXD12_GPIO_Port GPIOB
#define RXD13_Pin GPIO_PIN_13
#define RXD13_GPIO_Port GPIOB
#define RXD14_Pin GPIO_PIN_14
#define RXD14_GPIO_Port GPIOB
#define RXD15_Pin GPIO_PIN_15
#define RXD15_GPIO_Port GPIOB
#define TXValid_Pin GPIO_PIN_3
#define TXValid_GPIO_Port GPIOB
#define TXIC_Pin GPIO_PIN_4
#define TXIC_GPIO_Port GPIOB
#define Alive_Pin GPIO_PIN_5
#define Alive_GPIO_Port GPIOB
#define Busy_Pin GPIO_PIN_6
#define Busy_GPIO_Port GPIOB
#define RXValid_Pin GPIO_PIN_7
#define RXValid_GPIO_Port GPIOB
#define RXD8_Pin GPIO_PIN_8
#define RXD8_GPIO_Port GPIOB
#define RXD9_Pin GPIO_PIN_9
#define RXD9_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
	void phy_layer();		
	void interface();
	void phy_TX();
	void phy_RX();
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
