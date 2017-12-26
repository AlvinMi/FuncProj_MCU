/*********************************************************************************
* File:		ch375.c
* Author:	alvinmi
* function:	U �̶�д����
* 
**********************************************************************************/
#include <string.h>
#include <stdio.h>
#include "ch375.h"
#include "sys.h"
//�⼸��������Ҫ��375��ͷ�ļ�֮ǰ
//#define LIB_CFG_FILE_IO	1	/* �ļ���д�����ݵĸ��Ʒ�ʽ, 0 Ϊ'�ⲿ�ӳ���', 1 Ϊ'�ڲ�����' */ 
#define LIB_CFG_INT_EN	1	/* CH375 �� INT# �������ӷ�ʽ, 0Ϊ'��ѯ��ʽ', 1Ϊ'�жϷ�ʽ' */ 
#define CH375HF_NO_CODE 1 	// ��ֹ������Դ���߲�������, ��ֹ��ͷ�ļ������ظ���Ŀ�����.
#define NO_DEFAULT_CH375_INT	1	// ���� NO_DEFAULT_CH375_INT ��ֹĬ�ϵ��жϴ�����, ʹ�����б�д���жϳ������.
//#define CH375_INT_WIRE	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)

#if (  CH375_PORT_MODE==1 || CH375_PORT_MODE==2 )	// ������ģʽ��Ҫʹ�� INT# ��
#define CH375_INT_WIRE   PCin(9)  	// ���� CH375 �� GPIO.
#endif

#include "CH375HFM.h"		// �ӳ����

//#define MAX_PATH_LEN			32		/* �������·������, ������б�ָܷ�����С���������Լ�·�������� 00H, CH375 ģ��֧�ֵ����ֵ�� 64, ��Сֵ�� 32 */

/*  ��·���ӷ�ʽ
	MCU  = GPIO			 ģ��
	TXD	 = PA2		---	 RD		
	RXD	 = PA3		---	 WD	
	GPIO = PB15		---	 INT#	
*/

/* �ٶ��ļ����ݻ�����: ExtRAM: 0000H-7FFFH */
//u8 xdata DATA_BUF[512 * 64] _at_0x0000	// �ⲿ RAM ���ļ����ݻ�����, �Ӹõ�Ԫ��ʼ�Ļ��������Ȳ�С��һ�ζ�д�����ݳ���, ����Ϊ 512 �ֽ�

//u8 xdata *buffer	// ���ݻ�����ָ��, ���ڶ�д���ݿ�

//CMD_PARAM	mCmdParam;	// Ĭ������¸ýṹ��ռ�� 64 �ֽڵ� RAM, �����޸� MAX_PATH_LEN ����, ���޸�Ϊ 32 ʱ, ֻռ�� 32 �ֽڵ� RAM.


// �����Ķ���
//UINT8V	CH375IntStatus;		/* CH375�������ж�״̬ */
//UINT8V	CH375DiskStatus;	/* ���̼��ļ�״̬ */
//UINT8	CH375LibConfig;		/* CH375���������,����˵�� */

//PUINT8	pDISK_BASE_BUF;		/* ָ���ⲿ RAM �Ĵ������ݻ�����,���������Ȳ�С��CH375vSectorSize,��Ӧ�ó����ʼ�� */
//-----------------------------------------------------------------------------------------------------------



/* ����ģʽ */
#if ( CH375_PORT_MODE==1 )
//CH375д�����
void xWriteCH375Cmd( UINT8 mCmd )	/* �ⲿ����ı�CH375�������õ��ӳ���,��CH375д����,��С����Ϊ4uS,����֮ǰ֮�����ʱ2uS */
{
	USART_SendData(USART2, (uint16_t)mCmd|0x0100);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	delay_us(2);	
}

//CH375д���ݺ���
void xWriteCH375Data( UINT8 mData )	/* �ⲿ����ı�CH375�������õ��ӳ���,��CH375д����,��С����Ϊ1.5uS,����֮����ʱ1.5uS */
{
	USART_SendData(USART2, (uint16_t)mData );
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	delay_us(1);	
}

//CH375�����ݺ���
UINT8 xReadCH375Data( void )
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)==RESET);
	return( (UINT8)USART_ReceiveData(USART2) );	
}

// ���� PA2 = TXD  	PA3 = RXD ����
// PB15 == INT#   //������Ĭ������Ϊ9600
void CH375_Init(void)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure; 	// �жϽṹ���ʼ��
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		// INT# 	��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
 	USART_DeInit(USART2);  //��λ����2
	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
	
	//USART2_TX   PA2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
	  
    //USART2_RX	  PA3 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure); //��ʼ������
 	
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15); // ѡ�� GPIOB_15
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	// �ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // �½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 
}
#endif

//================================================================================================================//

/*********************************************************************************
* ����ԭ��: UINT8 CH375DiskConnect(void)
* ��	��: �������Ƿ����� 
* ��	��: None
* �� �� ֵ: UINT8
**********************************************************************************/
//UINT8 CH375DiskConnect(void)
//{
//	
//}

/*********************************************************************************
* ����ԭ��: UINT8 CH375FileModify(void)
* ��	��: ��ѯ�����޸ĵ�ǰ�ļ�����Ϣ
* ��	��: None
* �� �� ֵ: UINT8
**********************************************************************************/
//UINT8 CH375FileModify(void)
//{
//	return 0;
//}

/*********************************************************************************
* ����ԭ��: UINT8 CH375FileOpen(void)
* ��	��: ���ļ�����ö���ļ�
* ��	��: None
* �� �� ֵ: UINT8
**********************************************************************************/
//UINT8 CH375FileOpen(void)
//{

//	return 0;
//}

/*********************************************************************************
* ����ԭ��: UINT8 CH375GetVer(void)
* ��	��: ��ȡ��ǰ�ӳ����İ汾��
* ��	��: None
* �� �� ֵ: UINT8
**********************************************************************************/
//UINT8 CH375GetVer(void)
//{
//	
//	return 0;
//}

/*********************************************************************************
* ����ԭ��: UINT8 xReadCH375Data (void)
* ��	��: �ⲿ����ı�CH375�������õ��ӳ���,��CH375������,
*			��С����Ϊ1.5uS,����֮ǰ��ʱ1.5uS.
* ��	��: None
* �� �� ֵ: UINT8
**********************************************************************************/
//UINT8 xReadCH375Data (void)
//{
//	
//	return 0;
//}

/*********************************************************************************
* ����ԭ��: UINT8 xReadCH375Cmd(void)
* ��	��: �ⲿ����ı�CH375�������õ��ӳ���,��CH375������,
*			��С����Ϊ1.5uS,����֮ǰ��ʱ1.5uS.
* ��	��: None
* �� �� ֵ: UINT8
**********************************************************************************/
UINT8 xReadCH375Cmd(void)
{
//	UINT8	mData;
//	delay_us(1);	// ȷ����д���ڴ��� 0.6uS
	
	return 0;
}

/*********************************************************************************
* ����ԭ��: void xWriteCH375Cmd(UINT8 mCmd)
* ��	��: �ⲿ����ı�CH375�������õ��ӳ���,��CH375д����,
*			��С����Ϊ4uS,����֮ǰ֮�����ʱ2uS
* ��	��: UINT8 mCmd
* �� �� ֵ: void
**********************************************************************************/
//void xWriteCH375Cmd(UINT8 mCmd)
//{
////	CH375_Wr(0x100|mCmd); 
//}

/*********************************************************************************
* ����ԭ��: void xWriteCH375Data(UINT8 mData)
* ��	��: �ⲿ����ı�CH375�������õ��ӳ���,��CH375д����,
*			��С����Ϊ1.5uS,����֮����ʱ1.5uS.
* ��	��: UINT8 mData
* �� �� ֵ: void
**********************************************************************************/
//void xWriteCH375Data(UINT8 mData)
//{

//}

///* ����һ���ֽ����ݸ� CH375 */
//void mSendByte(u8 data)
//{
//	
//}

/*********************************************************************************
* ����ԭ��: void EXTI15_10_IRQHandler(void)
* ��	��: �жϴ�����: ���� INT# �ⲿ�ж�
* ��	��: void
* �� �� ֵ: void
**********************************************************************************/
void EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line15)) { 
		
		
	EXTI_ClearITPendingBit(EXTI_Line15); 
//	user_kk++; 
	} 
	
	xWriteCH375Cmd( CMD_GET_STATUS );  /* ��ȡ�ж�״̬��ȡ���ж����� */
	CH375IntStatus = xReadCH375Data( );  /* ��ȡ�ж�״̬ */
	if ( CH375IntStatus == USB_INT_DISCONNECT ) CH375DiskStatus = DISK_DISCONNECT;  /* ��⵽USB�豸�Ͽ��¼� */
	else if ( CH375IntStatus == USB_INT_CONNECT ) CH375DiskStatus = DISK_CONNECT;  /* ��⵽USB�豸�����¼� */
#ifdef CLEAR_INT_MARK
	CLEAR_INT_MARK( );  /* ĳЩ��Ƭ����Ҫ���������жϱ�־ */
#endif
}



