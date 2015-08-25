/*
 * Author: BangBH
 * Date: 27.JAN.2014
 */
#ifndef __DMX512_H_
#define __DMX512_H_

/*HWConfiguration*/
#define USART1_DR_Address		((uint32_t)0x40013804)
#define USART2_DR_Address		((uint32_t)0x40004404)
#define USART3_DR_Address  		((uint32_t)0x40004804)
#define UART4_DR_Address  		((uint32_t)0x40004c04)

#define DMXPORT			GPIOB
#define DMXPINRX		GPIO_Pin_11
#define DMXPINTX		GPIO_Pin_10
#define DMXUSART		USART3
#define DMXDMAChannel	DMA1_Channel2
#define DMXSELPORT		GPIOA
#define DMXSELPIN		GPIO_Pin_11
#define DMXTIM			TIM2
#define DMXM2MChannel	DMA2_Channel4

#define DMXBUFFERSIZE	513
#define DMXADDRESSRANGE	512
#define NOFSTATE		2

void vBreak();
void vIdle();
void vSendPacket();
void vDelayUS(unsigned int);
void vInit(unsigned char *);
void vChangeData(unsigned char *, unsigned int , unsigned char );

void TIM2_IRQHandler(void);

typedef struct
{
	const void (*vBreak)(void);
	const void (*vIdle)(void);
	const void (*vSendPacket)(void);
	const void (*vDelayUS)(unsigned int);
	const void (*vInit)(unsigned char*);
	const void (*vChangeData)(unsigned char*,unsigned int, unsigned char);
	unsigned char ucSendComplete;
}DMX512;

#endif
