#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include <math.h>

#define _DEBUG

#include "definitions.h"
// #include "motorcontrolsettings.h"

// PWM Frequency = 72000000/BLDC_CHOPPER_PERIOD
#define BLDC_CHOPPER_PERIOD 4500
// Dead time = BLDC_NOL/72000000  (on 72MHz: 7 is 98ns)
// (on 72MHz: 72 is 1000ns)
//#define BLDC_NOL 72
// #define BLDC_NOL 72

unsigned char phase;
unsigned char run;

unsigned short commcounter;
unsigned short step;
unsigned short runningdc;
unsigned short potvalue;

void delay(unsigned long time);
void commutate(void);
void commutate2(uint16_t hallpos);

void SetSysClockTo72(void)
{
	ErrorStatus HSEStartUpStatus;
	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		//FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		//FLASH_SetLatency( FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(0x00010000, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}
	else
	{ /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */

		/* Go to infinite loop */
		while (1)
		{
		}
	}
}

void led_init()
{
	/* Configure the GPIO_LED pin */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//GPIO_SetBits(GPIOC, GPIO_Pin_13); // Set C13 to High level ("1")
	GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Set C13 to Low level ("0")
}

void tim1_init()
{
	// PB13-15 will be normal gpio
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//initialize Tim1 PWM outputs
	// pin_12 for etr current chopping
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM1->SMCR = b15 + b4 + b5 + b6; // make ETR input active low
	TIM1->CR2 = 0;
	uint16_t init = 350;
	TIM1->CCR1 = init;
	TIM1->CCR2 = init;
	TIM1->CCR3 = init;
	// TIM1->CCR4 = 1100;
	TIM1->ARR = BLDC_CHOPPER_PERIOD;
	TIM1->CR1 = 0x0001;

	// note: b15 b7 and b7 are to enable ETR  based current limit
	TIM1->CCMR1 = 0x6868 + b15 + b7;
	TIM1->CCMR2 = 0x6868 + b7;
	// // b4 for cc4 and b7 for brk interrupt
	// //TIM1->DIER = b4+b7;  // enable cc4 interrupt
	TIM1->DIER = b6;
}

void usart3_init(void)
{
	/* Enable USART3 and GPIOB clock */
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* NVIC Configuration */
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure the GPIOs */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USART3 Rx (PB.11) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure the USART3 */
	USART_InitTypeDef USART_InitStructure;

	/* USART3 configuration ------------------------------------------------------*/
	/* USART3 configured as follow:
              - BaudRate = 115200 baud
              - Word Length = 8 Bits
              - One Stop Bit
              - No parity
              - Hardware flow control disabled (RTS and CTS signals)
              - Receive and transmit enabled
              - USART Clock disabled
              - USART CPOL: Clock is active low
              - USART CPHA: Data is captured on the middle
              - USART LastBit: The clock pulse of the last data bit is not output to
                               the SCLK pin
        */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);

	/* Enable USART3 */
	USART_Cmd(USART3, ENABLE);

	/* Enable the USART3 Receive interrupt: this interrupt is generated when the
             USART1 receive data register is not empty */
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USARTSend(const unsigned char *pucBuffer)
{
	while (*pucBuffer)
	{
		USART_SendData(USART3, *pucBuffer++);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		{
		}
	}
}

void HallSensorsGetPosition(void)
{
	uint8_t hallpos = (uint8_t)((GPIO_ReadInputData(GPIOB) & (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)) >> 7);
	switch (hallpos)
	{
	case 0b101:
		phase = 0;
		break;
	case 0b001:
		phase = 1;
		break;
	case 0b011:
		phase = 2;
		break;
	case 0b010:
		phase = 3;
		break;
	case 0b110:
		phase = 4;
		break;
	case 0b100:
		phase = 5;
		break;

	default:
		phase = 0;
		break;
	}

#ifdef _DEBUG
	char buffer[80] = {'\0'};
	sprintf(buffer, "\r\nRaw hall: %d\r\nPhase: %d\r\n", hallpos, phase);
	USARTSend(buffer);
#endif
}

void HallSensorsInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	// Init GPIO
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Init NVIC
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	// Tell system that you will use EXTI_Lines */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

	// EXTI
	EXTI_InitStruct.EXTI_Line = EXTI_Line7 | EXTI_Line8 | EXTI_Line9;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStruct);
}

void EXTI9_5_IRQHandler(void)
{
	if ((EXTI_GetITStatus(EXTI_Line7) | EXTI_GetITStatus(EXTI_Line8) | EXTI_GetITStatus(EXTI_Line9)) != RESET)
	{
		// Clear interrupt flags
		EXTI_ClearITPendingBit(EXTI_Line7);
		EXTI_ClearITPendingBit(EXTI_Line8);
		EXTI_ClearITPendingBit(EXTI_Line9);

		// Commutation
		HallSensorsGetPosition();
		commutate();
	}
}

void commutate(void)
{
	// Hall_CBA
	switch (phase)
	{
	case 0: // phase AB
		// enable all 6 except AN
		// invert AN
		TIM1->CCER = b10 + b8 + b6 + b4 + b0 + b3;
		TIM1->CCMR1 = 0x4868 + b15 + b7; // B low, A PWM
		TIM1->CCMR2 = 0x6858 + b7;		 // force C ref high (phc en low)
		break;
	case 1: // phase AC
		// enable all 6 except AN
		// invert AN
		TIM1->CCER = b10 + b8 + b6 + b4 + b0 + b3;
		TIM1->CCMR1 = 0x5868 + b15 + b7; // force B high and A PWM
		TIM1->CCMR2 = 0x6848 + b7;		 // force C ref low
		break;
	case 2: // phase BC
		// enable all 6 except BN
		// invert BN
		TIM1->CCER = b10 + b8 + b4 + b2 + b0 + b7;
		TIM1->CCMR1 = 0x6858 + b15 + b7; // force B PWM and A high
		TIM1->CCMR2 = 0x6848 + b7;		 // force C ref low
		break;
	case 3: // phase BA
		// enable all 6 except BN
		// invert BN
		TIM1->CCER = b10 + b8 + b4 + b2 + b0 + b7;
		TIM1->CCMR1 = 0x6848 + b15 + b7; // force B PWM and A ref low
		TIM1->CCMR2 = 0x6858 + b7;		 // force C ref high
		break;
	case 4: // phase CA
		// enable all 6 except CN
		// invert CN
		TIM1->CCER = b8 + b6 + b4 + b2 + b0 + b11; // enable all 6 except CN
		TIM1->CCMR1 = 0x5848 + b15 + b7;		   // force B high and A ref low
		TIM1->CCMR2 = 0x6868 + b7;				   // force C PWM
		break;
	case 5: // phase CB
		// enable all 6 except CN
		// invert CN
		TIM1->CCER = b8 + b6 + b4 + b2 + b0 + b11; // enable all 6 except CN
		TIM1->CCMR1 = 0x4858 + b15 + b7;		   // force B low and A high
		TIM1->CCMR2 = 0x6868 + b7;				   // force C PWM
		break;
	} // end of phase switch statement
}

void commutate2(uint16_t hallpos)
{
	switch (phase)
	{
	case 0: // phase AB
		// enable all 6 except AN
		// invert AN
		TIM1->CCER = b10 + b8 + b6 + b4 + b0 + b3;
		TIM1->CCMR1 = 0x4868 + b15 + b7; // B low, A PWM
		TIM1->CCMR2 = 0x6858 + b7;		 // force C ref high (phc en low)
		break;
	case 1: // phase AC
		// enable all 6 except AN
		// invert AN
		TIM1->CCER = b10 + b8 + b6 + b4 + b0 + b3;
		TIM1->CCMR1 = 0x5868 + b15 + b7; // force B high and A PWM
		TIM1->CCMR2 = 0x6848 + b7;		 // force C ref low
		break;
	case 2: // phase BC
		// enable all 6 except BN
		// invert BN
		TIM1->CCER = b10 + b8 + b4 + b2 + b0 + b7;
		TIM1->CCMR1 = 0x6858 + b15 + b7; // force B PWM and A high
		TIM1->CCMR2 = 0x6848 + b7;		 // force C ref low
		break;
	case 3: // phase BA
		// enable all 6 except BN
		// invert BN
		TIM1->CCER = b10 + b8 + b4 + b2 + b0 + b7;
		TIM1->CCMR1 = 0x6848 + b15 + b7; // force B PWM and A ref low
		TIM1->CCMR2 = 0x6858 + b7;		 // force C ref high
		break;
	case 4: // phase CA
		// enable all 6 except CN
		// invert CN
		TIM1->CCER = b8 + b6 + b4 + b2 + b0 + b11; // enable all 6 except CN
		TIM1->CCMR1 = 0x5848 + b15 + b7;		   // force B high and A ref low
		TIM1->CCMR2 = 0x6868 + b7;				   // force C PWM
		break;
	case 5: // phase CB
		// enable all 6 except CN
		// invert CN
		TIM1->CCER = b8 + b6 + b4 + b2 + b0 + b11; // enable all 6 except CN
		TIM1->CCMR1 = 0x4858 + b15 + b7;		   // force B low and A high
		TIM1->CCMR2 = 0x6868 + b7;				   // force C PWM
		break;
	} // end of phase switch statement
}

void motorstartinit(void)
{
	TIM1->CCER = 0;

	HallSensorsGetPosition();
	commutate();

	// b12 to enable brk input
	// b13 for break polarity
	// (b15+b11); set MOE
	TIM1->BDTR = b15 + b11 + b10 + b12 + b13; //set MOE
}

int main(void)
{
	// clock setup
	SetSysClockTo72();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO,
						   ENABLE);
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	//**********************************************

	delay(5000000); //let power supply settle

	led_init();
	HallSensorsInit();

	phase = 0;

	// tim1 setup
	tim1_init();

#ifdef _DEBUG
	usart3_init();
#endif

	delay(5000000); //let power supply settle
	delay(5000000); //let power supply settle
	delay(5000000); //let power supply settle
	motorstartinit();

	while (1)
	{ // backgroung loop
		//TIM1->BDTR= b15+b11+b10+b12+b13;  //  set MOE

		if ((4095 - potvalue) > 200)
			run = 255;
		if ((4095 - potvalue) < 100)
			run = 0;

		// if (run)
		// 	ledon;
		// if (run == 0)
		// 	ledoff;
	} // end of backgroung loop

} // end of main

// void commutate(void)
// {

// 	switch (phase)
// 	{
// 	case 0:									 // phase AB
// 		TIM1->CCER = b3 + b0 + b7 + b4 + b8; //
// 		TIM1->CCMR1 = 0x4868 + b15 + b7;	 // force B active low
// 		TIM1->CCMR2 = 0x6868 + b7;
// 		break;
// 	case 1:									  // phase AC
// 		TIM1->CCER = b3 + b0 + b4 + b8 + b11; //
// 		TIM1->CCMR1 = 0x6868 + b15 + b7;
// 		TIM1->CCMR2 = 0x6848 + b7; // force C active low
// 		break;
// 	case 2: // phase BC
// 		TIM1->CCER = b4 + b7 + b8 + b11 + b0;
// 		TIM1->CCMR1 = 0x6868 + b15 + b7;
// 		TIM1->CCMR2 = 0x6848 + b7; // force C active low
// 		break;
// 	case 3: // phase BA
// 		TIM1->CCER = b4 + b7 + b3 + b0 + b8;
// 		TIM1->CCMR1 = 0x6848 + b15 + b7; // force A active low
// 		TIM1->CCMR2 = 0x6868 + b7;
// 		break;
// 	case 4: // phase CA
// 		TIM1->CCER = b8 + b11 + b3 + b0 + b4;
// 		TIM1->CCMR1 = 0x6848 + b15 + b7; // force A active low
// 		TIM1->CCMR2 = 0x6868 + b7;
// 		break;
// 	case 5: // phase CB
// 		TIM1->CCER = b8 + b11 + b4 + b7 + b0;
// 		TIM1->CCMR1 = 0x4868 + b15 + b7; // force B active low
// 		TIM1->CCMR2 = 0x6868 + b7;
// 		break;
// 	} // end of phase switch statement
// } // end of commutate function

void delay(unsigned long time) // 1000 is 200 usec
{
	while (time > 0)
		time--;
}

// unsigned short readadc(unsigned char chnl)
// {
// 	ADC1->CHSELR = (1 << chnl); // set ADC MUX
// 	ADC1->CR = b2;				// start adc conversion

// 	while ((ADC1->CR & b2) == b2)
// 		; // wait for conversion to complete
// 	return ADC1->DR;
// } // end of readadc function
