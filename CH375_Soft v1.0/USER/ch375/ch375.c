/*********************************************************************************
* File:		ch375.c
* Author:	alvinmi
* function:	U 盘读写功能
* 
**********************************************************************************/
#include <string.h>
#include <stdio.h>
#include "ch375.h"
#include "sys.h"
//这几个定义需要在375的头文件之前
//#define LIB_CFG_FILE_IO	1	/* 文件读写的数据的复制方式, 0 为'外部子程序', 1 为'内部复制' */ 
#define LIB_CFG_INT_EN	1	/* CH375 的 INT# 引脚连接方式, 0为'查询方式', 1为'中断方式' */ 
#define CH375HF_NO_CODE 1 	// 禁止分配资源或者产生代码, 禁止该头文件产生重复的目标代码.
#define NO_DEFAULT_CH375_INT	1	// 定义 NO_DEFAULT_CH375_INT 禁止默认的中断处理函数, 使用自行编写的中断程序代替.
//#define CH375_INT_WIRE	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)

#if (  CH375_PORT_MODE==1 || CH375_PORT_MODE==2 )	// 这两种模式需要使用 INT# 脚
#define CH375_INT_WIRE   PCin(9)  	// 定义 CH375 的 GPIO.
#endif

#include "CH375HFM.h"		// 子程序库

//#define MAX_PATH_LEN			32		/* 定义最大路径长度, 含所有斜杠分隔符和小数点间隔符以及路径结束符 00H, CH375 模块支持的最大值是 64, 最小值是 32 */

/*  电路连接方式
	MCU  = GPIO			 模块
	TXD	 = PA2		---	 RD		
	RXD	 = PA3		---	 WD	
	GPIO = PB15		---	 INT#	
*/

/* 假定文件数据缓冲区: ExtRAM: 0000H-7FFFH */
//u8 xdata DATA_BUF[512 * 64] _at_0x0000	// 外部 RAM 的文件数据缓冲区, 从该单元开始的缓冲区长度不小于一次读写的数据长度, 最少为 512 字节

//u8 xdata *buffer	// 数据缓冲区指针, 用于读写数据块

//CMD_PARAM	mCmdParam;	// 默认情况下该结构将占用 64 字节的 RAM, 可以修改 MAX_PATH_LEN 常量, 当修改为 32 时, 只占用 32 字节的 RAM.


// 变量的定义
//UINT8V	CH375IntStatus;		/* CH375操作的中断状态 */
//UINT8V	CH375DiskStatus;	/* 磁盘及文件状态 */
//UINT8	CH375LibConfig;		/* CH375程序库配置,下行说明 */

//PUINT8	pDISK_BASE_BUF;		/* 指向外部 RAM 的磁盘数据缓冲区,缓冲区长度不小于CH375vSectorSize,由应用程序初始化 */
//-----------------------------------------------------------------------------------------------------------



/* 串口模式 */
#if ( CH375_PORT_MODE==1 )
//CH375写命令函数
void xWriteCH375Cmd( UINT8 mCmd )	/* 外部定义的被CH375程序库调用的子程序,向CH375写命令,最小周期为4uS,否则之前之后各延时2uS */
{
	USART_SendData(USART2, (uint16_t)mCmd|0x0100);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	delay_us(2);	
}

//CH375写数据函数
void xWriteCH375Data( UINT8 mData )	/* 外部定义的被CH375程序库调用的子程序,向CH375写数据,最小周期为1.5uS,否则之后延时1.5uS */
{
	USART_SendData(USART2, (uint16_t)mData );
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	delay_us(1);	
}

//CH375读数据函数
UINT8 xReadCH375Data( void )
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)==RESET);
	return( (UINT8)USART_ReceiveData(USART2) );	
}

// 串口 PA2 = TXD  	PA3 = RXD 复用
// PB15 == INT#   //波特率默认配置为9600
void CH375_Init(void)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure; 	// 中断结构体初始化
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		// INT# 	上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
 	USART_DeInit(USART2);  //复位串口2
	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
	
	//USART2_TX   PA2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
	  
    //USART2_RX	  PA3 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口
 	
    USART_Cmd(USART2, ENABLE);                    //使能串口
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15); // 选择 GPIOB_15
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	// 中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 
}
#endif

//================================================================================================================//

/*********************************************************************************
* 函数原型: UINT8 CH375DiskConnect(void)
* 功	能: 检查磁盘是否连接 
* 参	数: None
* 返 回 值: UINT8
**********************************************************************************/
//UINT8 CH375DiskConnect(void)
//{
//	
//}

/*********************************************************************************
* 函数原型: UINT8 CH375FileModify(void)
* 功	能: 查询或者修改当前文件的信息
* 参	数: None
* 返 回 值: UINT8
**********************************************************************************/
//UINT8 CH375FileModify(void)
//{
//	return 0;
//}

/*********************************************************************************
* 函数原型: UINT8 CH375FileOpen(void)
* 功	能: 打开文件或者枚举文件
* 参	数: None
* 返 回 值: UINT8
**********************************************************************************/
//UINT8 CH375FileOpen(void)
//{

//	return 0;
//}

/*********************************************************************************
* 函数原型: UINT8 CH375GetVer(void)
* 功	能: 获取当前子程序库的版本号
* 参	数: None
* 返 回 值: UINT8
**********************************************************************************/
//UINT8 CH375GetVer(void)
//{
//	
//	return 0;
//}

/*********************************************************************************
* 函数原型: UINT8 xReadCH375Data (void)
* 功	能: 外部定义的被CH375程序库调用的子程序,从CH375读数据,
*			最小周期为1.5uS,否则之前延时1.5uS.
* 参	数: None
* 返 回 值: UINT8
**********************************************************************************/
//UINT8 xReadCH375Data (void)
//{
//	
//	return 0;
//}

/*********************************************************************************
* 函数原型: UINT8 xReadCH375Cmd(void)
* 功	能: 外部定义的被CH375程序库调用的子程序,从CH375读数据,
*			最小周期为1.5uS,否则之前延时1.5uS.
* 参	数: None
* 返 回 值: UINT8
**********************************************************************************/
UINT8 xReadCH375Cmd(void)
{
//	UINT8	mData;
//	delay_us(1);	// 确保读写周期大于 0.6uS
	
	return 0;
}

/*********************************************************************************
* 函数原型: void xWriteCH375Cmd(UINT8 mCmd)
* 功	能: 外部定义的被CH375程序库调用的子程序,向CH375写命令,
*			最小周期为4uS,否则之前之后各延时2uS
* 参	数: UINT8 mCmd
* 返 回 值: void
**********************************************************************************/
//void xWriteCH375Cmd(UINT8 mCmd)
//{
////	CH375_Wr(0x100|mCmd); 
//}

/*********************************************************************************
* 函数原型: void xWriteCH375Data(UINT8 mData)
* 功	能: 外部定义的被CH375程序库调用的子程序,向CH375写数据,
*			最小周期为1.5uS,否则之后延时1.5uS.
* 参	数: UINT8 mData
* 返 回 值: void
**********************************************************************************/
//void xWriteCH375Data(UINT8 mData)
//{

//}

///* 发送一个字节数据给 CH375 */
//void mSendByte(u8 data)
//{
//	
//}

/*********************************************************************************
* 函数原型: void EXTI15_10_IRQHandler(void)
* 功	能: 中断处理函数: 处理 INT# 外部中断
* 参	数: void
* 返 回 值: void
**********************************************************************************/
void EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line15)) { 
		
		
	EXTI_ClearITPendingBit(EXTI_Line15); 
//	user_kk++; 
	} 
	
	xWriteCH375Cmd( CMD_GET_STATUS );  /* 获取中断状态并取消中断请求 */
	CH375IntStatus = xReadCH375Data( );  /* 获取中断状态 */
	if ( CH375IntStatus == USB_INT_DISCONNECT ) CH375DiskStatus = DISK_DISCONNECT;  /* 检测到USB设备断开事件 */
	else if ( CH375IntStatus == USB_INT_CONNECT ) CH375DiskStatus = DISK_CONNECT;  /* 检测到USB设备连接事件 */
#ifdef CLEAR_INT_MARK
	CLEAR_INT_MARK( );  /* 某些单片机需要由软件清除中断标志 */
#endif
}



