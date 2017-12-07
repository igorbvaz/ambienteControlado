#include "stm32f4_discovery.h"
#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx_adc.h"
#include "string.h"

GPIO_InitTypeDef  GPIO_InitStructure;
ADC_InitTypeDef  ADC_InitStructure;
void Delay(__IO uint32_t nCount); 
	
bool push;
bool I1,I2,LED1,LED2;
int valor_ADC;
extern void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);

int adc_convert();
void configureGPIO();
void configureADC();
char decodifica_7seg(int);


/*MAIN*/	
int main(void)
{
//	bool apertado = 0;
	int adc_value = 0;
	int temperatura = 0;
	char exibir[26];
  bool asd = false;
  /* Configure PC1 in output pushpull mode */
  configureGPIO();
	
	// ADC configuration
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // PB1 (ADC1)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	configureADC();

	GPIO_SetBits(GPIOA, decodifica_7seg(4));
	
	/* Initiate LED and BUTTON */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);   /* Push Button*/
//  STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
//	STM_EVAL_LEDInit(LED5);
	
//  push = false;
//  STM_EVAL_LEDOff(LED3);
//	STM_EVAL_LEDOff(LED4);
//	STM_EVAL_LEDOff(LED5);
  while (1)       /*Infinite loop*/
  {	
//		if (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET && !apertado ) {
//			if (!push) {
//			GPIO_SetBits(GPIOC, GPIO_Pin_1);
//			STM_EVAL_LEDOn(LED3);
//			push = !push;
//			} else {
//			
//			GPIO_ResetBits(GPIOC, GPIO_Pin_1);
//			STM_EVAL_LEDOff(LED3);
//			push = !push;
//			}
//			apertado = true;
//		}
//		if (STM_EVAL_PBGetState(BUTTON_USER) == Bit_RESET)
//			apertado = false;
	/*
		1000 => 0.714 V
		2000 => 1.445 V
		3000 => 2.140 V
		
		V = 7 + 713*((float)N/1000) (em mV)
		T = (V - 500)/10
	*/
	
	ADC_SoftwareStartConv(ADC1);//Start the conversion
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
  adc_value = ADC_GetConversionValue(ADC1);
	temperatura = (int) (7.0 + 0.713*(adc_value) - 500)/10;
	//temperatura = (int) (7.0 + 0.713*(adc_convert()) - 500)/10;
	
	
	if (!asd) {
		
		STM_EVAL_LEDOn(LED4);
		
	} else {
		
		STM_EVAL_LEDOff(LED4);
		
	}
	asd = !asd;
	
	sprintf(exibir, "%d", temperatura);
	
	
		
/************************/		
		
		//STM_EVAL_PBGetState(BUTTON_USER);
		Delay(168e4); 
	}  /*end of main */
}
/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void configureGPIO() {
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); /* GPIOC Periph clock enable */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); /* GPIOC Periph clock enable */
	GPIO_InitStructure.GPIO_Pin = 0x3FFF; //Pinos PA0 a PA13
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

void configureADC() {
	//ini
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); /* ADC Clock */
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit number giving a maximum value of 4096
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;// conversion is synchronous with TIM1 and CC1 
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//The scan is configured in one channel
	ADC_Init(ADC1,&ADC_InitStructure);//Initialize ADC with the previous configuration
	//Enable ADC conversion
	ADC_Cmd(ADC1,ENABLE);
	
	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1,ADC_Channel_9,1,ADC_SampleTime_144Cycles); // PB1 = IN_9 
	
}
int adc_convert(){
 ADC_SoftwareStartConv(ADC1);//Start the conversion
 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
 return ADC_GetConversionValue(ADC1); //Return the converted data
}

char decodifica_7seg(int valor) {
	
	int vetor[10] = {0x7E, 0x48, 0x3D, 0x6D, 0x4B, 0x67, 0x77, 0x4C, 0x7F, 0x6F};
	return vetor[valor];

}
