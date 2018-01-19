/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int ADC_Value;
uint16_t vetorB[10] = {0xF90, 0x900, 0x788, 0xD88, 0x918, 0xC98, 0xE98, 0x980, 0xF98, 0xD98};
uint16_t vetorC[10] = {0xAD8, 0x840, 0x2D4, 0xA54, 0x84C, 0xA1C, 0xA9C, 0x850, 0xADC, 0xA5C};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void writeToLeftDisplays(int);
void setPWM(TIM_HandleTypeDef, uint32_t, uint16_t);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	int temperatura, temperaturaAn, referencia, count;
	bool apertado, controleAtivo;
	apertado = false;
	controleAtivo = false;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_ADC2_Init();
//  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	MX_TIM4_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	count = 0;
  while (1)
  {
		
		if (!apertado && HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) { //Se botão apertado
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1); // porta para ativar lâmpada
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); //led para indicar ligado
			controleAtivo = !controleAtivo; //liga e desliga o sistema de controle e exibição
			apertado = true;
		}
		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) apertado = false;
		
		
		/*
		1000 => 0.714 V
		2000 => 1.445 V
		3000 => 2.140 V
		
		V = 7 + 713*((float)N/1000) (em mV)
		T = V/10 (V em mV)
	*/
		
		if (controleAtivo) { //sistema de controle e exibição
			if (HAL_ADC_PollForConversion(&hadc1, 300) == HAL_OK) temperatura = HAL_ADC_GetValue(&hadc1);
			//if (HAL_ADC_PollForConversion(&hadc2, 300) == HAL_OK) referencia = HAL_ADC_GetValue(&hadc2);
			setPWM(htim4, TIM_CHANNEL_4, count*4095/30); //teste do PWM
			temperatura = temperatura * 3300 / 0xFFF; //int 12 bits para tensão
			temperatura = temperatura/18;
			if (count == 0) { //atualiza a temperatura no display a cada 30 iterações do while
				if (temperatura > 99) {
					
				writeToLeftDisplays(99);	
				} else {
					
				writeToLeftDisplays(temperatura);	
				}
			}
			count++;
			if (count == 31) count = 0;
			
			//temperatura = 25;
			//referencia = (7 + 0.713*referencia)/10;
			//writeTo7seg((count > 1) ? temperatura : referencia, count);
			//count++;
			//if (count == 4) count = 0;
			//setPWM(htim4, TIM_CHANNEL_3, 4095, count << 9); 
		}
		else { //Desligando
//			writeToLeftDisplays(0);
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_All & ~GPIO_PIN_15 & ~GPIO_PIN_14, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_All & ~GPIO_PIN_15 & ~GPIO_PIN_14, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
			
		}
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		HAL_Delay(50);
		//writeToLeftDisplays(39);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

void writeToLeftDisplays(int number) {
	
		int dezena = number / 10;
	  int unidade = number % 10;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_All & ~GPIO_PIN_1 & ~GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, vetorB[dezena] << 4, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, vetorC[unidade] << 2, GPIO_PIN_SET);
	
//	for (int i = 0; i < 100; i++) {
//		int dezena = i / 10;
//	  int unidade = i % 10;
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_All, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, vetorB[dezena] << 4, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOC, vetorC[unidade] << 2, GPIO_PIN_SET);
//		HAL_Delay(200);
//		
//	}
	
}

void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t pulse) { 
  HAL_TIM_PWM_Stop(&timer, channel);    
// stop generation of pwm
  TIM_OC_InitTypeDef sConfigOC; 
  timer.Init.Period = 4095;           
// set the period duration
  HAL_TIM_PWM_Init(&timer);  
// reinititialise with new period value
  sConfigOC.OCMode = TIM_OCMODE_PWM1; 
  sConfigOC.Pulse = pulse;              
// set the pulse duration 
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; 
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; 
  HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel); 
  HAL_TIM_PWM_Start(&timer, channel);   
// start pwm generation
} 
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
