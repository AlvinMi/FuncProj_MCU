#ifndef	__CH375_H__
#define	__CH375_H__

#include "sys.h"
#include "delay.h"

// 并口模式速度比串口高, 由于我们的是串口模式.
#define CH375_PORT_MODE   1            //串口查询接收，使用INT脚
//#define CH375_PORT_MODE   2            //并口，使用INT脚
//#define CH375_PORT_MODE   3            //并口，不使用INT脚
void  CH375_Init( void );

#if (CH375_PORT_MODE == 3)
extern uint8_t xReadCH375Cmd( void );
#endif
#endif


