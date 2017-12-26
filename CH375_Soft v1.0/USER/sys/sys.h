#ifndef __SYS_H
#define __SYS_H	
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
#ifndef BYTE
	#define BYTE u8
#endif

#ifndef WORD
	#define WORD unsigned short
#endif

#ifndef DWORD
	#define DWORD unsigned int
#endif

#ifndef BOOL
	#define BOOL unsigned int
#endif

#ifndef TRUE
	#define TRUE (1==1)
#endif

#ifndef FALSE
	#define FALSE (1==0)
#endif

#ifndef NULL
	#define NULL  0
#endif
	

//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	 

//0,��֧��ucos
//1,֧��ucos
#define SYSTEM_SUPPORT_OS		0		//����ϵͳ�ļ����Ƿ�֧��UCOS
																	    
	 
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����


/******************* PRINTER M-150II IO ************/
//#define PTCOM_PORT							GPIOC
//#define PTCOM_PIN								GPIO_Pin_5
//#define PTCOM_ON()							GPIO_SetBits(PTCOM_PORT, PTCOM_PIN)
//#define PTCOM_OFF()							GPIO_ResetBits(PTCOM_PORT, PTCOM_PIN)	// no use, no modify

#define PTA_PORT								GPIOC	//GPIOB
#define PTA_PIN									GPIO_Pin_3	//GPIO_Pin_3
#define PTA_OFF()    						GPIO_ResetBits(PTA_PORT, PTA_PIN)
#define PTA_ON()    						GPIO_SetBits(PTA_PORT, PTA_PIN)				// GPIOB_O

#define PTB_PORT								GPIOC	//GPIOB
#define PTB_PIN									GPIO_Pin_2	//GPIO_Pin_2
#define PTB_OFF()     					GPIO_ResetBits(PTB_PORT, PTB_PIN)
#define PTB_ON()    						GPIO_SetBits(PTB_PORT, PTB_PIN)				// GPIOB_1

#define PTC_PORT								GPIOC	//GPIOB
#define PTC_PIN									GPIO_Pin_1	//GPIO_Pin_1
#define PTC_OFF()     					GPIO_ResetBits(PTC_PORT, PTC_PIN)
#define PTC_ON()    						GPIO_SetBits(PTC_PORT, PTC_PIN)				// GPIOB_2

#define PTD_PORT								GPIOC	//GPIOB
#define PTD_PIN									GPIO_Pin_0	//GPIO_Pin_5
#define PTD_OFF()     					GPIO_ResetBits(PTD_PORT, PTD_PIN)
#define PTD_ON()    						GPIO_SetBits(PTD_PORT, PTD_PIN)				// GPIOB_3

#define MOTER_PORT								GPIOC	//GPIOB
#define MOTER_PIN								GPIO_Pin_4	//GPIO_Pin_7
#define MOTER_OFF()     				GPIO_ResetBits(MOTER_PORT, MOTER_PIN)
#define MOTER_ON()    					GPIO_SetBits(MOTER_PORT, MOTER_PIN)			// GPIOB_4

#define PT_TIMER_PORT							GPIOB	//GPIOB
#define PT_TIMER_PIN								GPIO_Pin_9	//GPIO_Pin_6
#define PT_GET_TIMER()     			GPIO_ReadInputDataBit(PT_TIMER_PORT, PT_TIMER_PIN)	// GPIOB_5

#define PT_RESET_DETCT_PORT					GPIOB	//GPIOB
#define PT_RESET_DETCT_PIN						GPIO_Pin_8	//GPIO_Pin_12
#define PT_GET_RESET()     			GPIO_ReadInputDataBit(PT_RESET_DETCT_PORT, PT_RESET_DETCT_PIN)	// GPIOB_6

////////////////////////////////////////////////////////////////////////////////

#define MAX_PATH				128
#define MAX_OBD_RESPONSE	MAX_PATH
#define TIM2_CLK_PERIOD		0xFA0	// 2ms @ 2M clk
///////////////////////////////////////////////////////////////////////////////////////////

#define SPI_MEMS_CS_GPIO	GPIOC
#define SPI_MEMS_CS				GPIO_Pin_6
//////////////////////////////////OBD///////////////////////////////////////////////////////////

#define POWER_ON	1
#define POWER_OFF	0

#define PORT_ELM_RSTN	   GPIOA
#define PIN_ELM_RSTN		 GPIO_Pin_1

#define PORT_ELM_RX			GPIOD
#define PIN_ELM_RX			GPIO_Pin_2

#define PORT_ELM_TX			GPIOC
#define PIN_ELM_TX			GPIO_Pin_12

#define DEFAULT_TX_BUFF_LEN	128
#define DEFAULT_RX_BUFF_LEN	512
//////////////////////////PEMSENSOR/////////////////////////
#define PORT_PWMSENSOR	GPIOA
#define PIN_PWMSENSOR		GPIO_Pin_8
#define PLUSE_PR				8
#define R_KM						1000
enum __CCState{
	WAIT_RISING = 1,
	WAIT_FALLING = 2
};

void  TIM2_Setting(void);
void  TIM4_Setting(void);
//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ

#endif
