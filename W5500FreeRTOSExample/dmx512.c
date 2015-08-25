#include "dmx512.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"

/*
 *hw Configuration for Stm32f103ze
 */

unsigned char TmrFlag;

void vBreak()
{
	//USART_Cmd(UART4, DISABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = DMXPINTX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIOC->CRH &= 0xfffffBff;
	GPIO_Init(DMXPORT, &GPIO_InitStructure);
	GPIO_WriteBit(DMXPORT,DMXPINTX,Bit_RESET);
}

void vIdle()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure UART4 Tx (pa2) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = DMXPINTX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DMXPORT, &GPIO_InitStructure);
	//GPIOC->CRH &= 0xffff0ff;
	//GPIOC->CRH |= 0x0000B00;
//	GPIO_WriteBit(GPIOC,GPIO_Pin_10,Bit_SET);
}

void vSendPacket()
{
	DMA_Cmd(DMXDMAChannel,DISABLE);
	DMA_SetCurrDataCounter(DMXDMAChannel,DMXBUFFERSIZE);
	//USART_Cmd(UART4, ENABLE);
	DMA_Cmd(DMXDMAChannel,ENABLE);
}

void vDelayUS(unsigned int Us)
{
	TmrFlag = 1;
	DMXTIM->ARR = Us;
	DMXTIM->CR1 |= TIM_CR1_CEN;
	while(TmrFlag);
}

void vInit(unsigned char *DMX512Buffer)
{
	memset(DMX512Buffer,0,DMXBUFFERSIZE);

	/*Us Delay Timer*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/*USART DMA*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/*USART*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
	/**/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure UART4 Tx (pa2) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = DMXPINTX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DMXPORT, &GPIO_InitStructure);

	/* Configure UART4 Rx (pa3) as input floating */
	GPIO_InitStructure.GPIO_Pin = DMXPINRX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DMXPORT, &GPIO_InitStructure);

	/* 75176 select*/
	GPIO_InitStructure.GPIO_Pin = DMXSELPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(DMXSELPORT, &GPIO_InitStructure);
	GPIO_WriteBit(DMXSELPORT,DMXSELPIN,Bit_SET);

	/*us delay function for dmx512*/
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*for dmx512 Send Complete*/
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Compute the prescaler value */
	uint16_t PrescalerValue;
	PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 71;//1MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 10000;//1sec
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(DMXTIM, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(DMXTIM,TIM_IT_Update);
	TIM_ITConfig(DMXTIM,TIM_IT_Update,ENABLE);

	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 250000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(DMXUSART, &USART_InitStructure);
	USART_Cmd(DMXUSART, ENABLE);
	USART_DMACmd(DMXUSART,USART_DMAReq_Tx,ENABLE);
	USART_ClearFlag(DMXUSART,USART_FLAG_TXE);
	USART_ClearITPendingBit(DMXUSART,USART_IT_TXE);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = DMX512Buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = DMXBUFFERSIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMXDMAChannel, &DMA_InitStructure);
	DMA_ITConfig(DMXDMAChannel,DMA_IT_TC,ENABLE);
}

void vChangeData(unsigned char *DMX512Buffer, unsigned int address, unsigned char data)
{
	if(address < 512) //Only addresses of 0 to 511 are possible.
	{
		DMX512Buffer[address+1] = data;
	}
}

void TIM2_IRQHandler(void)
{
	vTaskSuspendAll();
	TmrFlag = 0;
	TIM_Cmd(TIM2, DISABLE);
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	xTaskResumeAll();
}

extern DMX512 dmx =
{
	vBreak,
	vIdle,
	vSendPacket,
	vDelayUS,
	vInit,
	vChangeData
};

void DMA1_Channel2_IRQHandler(void)
{
	vTaskSuspendAll();
	dmx.ucSendComplete = 0;
	DMA_ClearFlag(DMA1_FLAG_TC2);
	DMA_ClearITPendingBit(DMA1_IT_TC2);
	xTaskResumeAll();
}
