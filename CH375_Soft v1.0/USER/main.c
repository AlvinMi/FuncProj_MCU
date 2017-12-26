/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "stm32f10x_rcc.h"
#include "delay.h"
#include "Uart.h"
//#include "epson_m150ii.h"


#include "CH375HFM.H"
#include "ch375.h"
void RCC_Configuration(void)
{   
	ErrorStatus HSEStartUpStatus;

	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1); 

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */ 
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | 
		RCC_APB2Periph_GPIOA |
		RCC_APB2Periph_GPIOB |
		RCC_APB2Periph_ADC1 |
		RCC_APB2Periph_USART1 |
		RCC_APB2Periph_AFIO,		
		ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 |
		RCC_APB1Periph_TIM4, 
		ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, 
		ENABLE);

	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//	SEGGER_RTT_printf(0, "sysclock is %d", SystemCoreClock );
}

/******************************************************************************
函数原型:	void Nvic_Init(void)
功    能:	NVIC 初始化
*******************************************************************************/ 
void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// NVIC_PriorityGroup 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		// 位于同一组别, 

	// USART ( 串口)
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		// USART
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		// USART
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级为 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  // 从优先级为 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/* 检查操作状态,如果错误则显示错误代码并停机 */
void mStopIfError( UINT8 iError )
{
	if ( iError == ERR_SUCCESS ) return;                        /* 操作成功 */
	printf( "Error: %02X\n", (UINT16)iError );                  /* 显示错误 */
	while ( 1 ) {

	}
}



int main(void)
{
	u8 i;
	
	RCC_Configuration();	
	Uart1_Init(115200);	// 串口初始化： 波特率 115200, 8 数据位, 1 位停止位, 禁用奇偶校验
	delay_init();		// 延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	Nvic_Init();
	
	printf("010101010101010101\n");

	CH375_Init();
	i = CH375LibInit( );                                      /* 初始化CH375程序库和CH375芯片,操作成功返回0 */
	mStopIfError( i );	

  	while (1)
  	{
		printf("wait udisk connect\n");
		while ( CH375DiskStatus < DISK_CONNECT ) {            /* 查询CH375中断并更新中断状态,等待U盘插入 */
			if ( CH375DiskConnect( ) == ERR_SUCCESS ) break;  /* 有设备连接则返回成功,CH375DiskConnect同时会更新全局变量CH375DiskStatus */
			delay_ms( 100 );
		}
  	}
}

