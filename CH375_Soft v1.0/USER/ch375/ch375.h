#ifndef	__CH375_H__
#define	__CH375_H__

#include "sys.h"
#include "delay.h"

// ����ģʽ�ٶȱȴ��ڸ�, �������ǵ��Ǵ���ģʽ.
#define CH375_PORT_MODE   1            //���ڲ�ѯ���գ�ʹ��INT��
//#define CH375_PORT_MODE   2            //���ڣ�ʹ��INT��
//#define CH375_PORT_MODE   3            //���ڣ���ʹ��INT��
void  CH375_Init( void );

#if (CH375_PORT_MODE == 3)
extern uint8_t xReadCH375Cmd( void );
#endif
#endif


