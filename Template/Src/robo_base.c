//�ļ�����:		robo_base.c
//��Ӧͷ�ļ�:	robo_base.h
//��Ҫ����:
//		����motor_system��װ���ٶȻ�/λ�û�/���Ź�����, ʵ�ֵ��̿���
//
//ʱ��:
//		2021/5/8
//
//�汾:	3.0V
//
//---------ͷ�ļ����ò���---------//
#include "robo_base.h"
//--------------------------------//

//---------������������-----------//
ROBO_BASE Robo_Base;
//--------------------------------//


//--------------------------------------------------------------------------------------------------//
//��������:
//		���̲�����ʼ��
//
//��������:
//		��ʼ���������е���Ϣ
//
//��������:
//		ROBO_BASE ָ��, ���̽ṹ���ָ��
//
//��ֲ����:
//		��ʲô״̬, ���, ���״̬���Ȱ����ݷ�װ��ROBO_BASE�ṹ����, Ȼ��ֱ�ӳ�ʼ���ͺ���
//
//--------------------------------------------------------------------------------------------------//
#if UART_COMMUNICATE_USER_ROOT
	weak
#endif
void BASE_Init(void)
{
  Motor_System* P_Motor=NULL;
  P_Motor=&Robo_Base.All_Motor;	Motor_System_Init(P_Motor,0,&Can_TxMessageList[0]);
	PID_Init(&P_Motor->Pos_PID,			0.3,	0,	0,	5000,	0,	0,	7000);
	PID_Init(&P_Motor->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000); 

  Pos_System* P_Pos=NULL;
  P_Pos=&Robo_Base.Pos_Motor; Pos_System_Init(P_Pos,0,&Can_TxMessageList[0]);
	PID_Init(&P_Pos->Pos_PID,				0.3,	0,	0,	5000,	0,	0,	7000);
  PID_Init(&P_Pos->Speed_PID,				5,	0,	0,	5000,	0,	0,	7000);

  Speed_System* P_Speed=NULL; 
  P_Speed=&Robo_Base.Speed_Motor; Speed_System_Init(P_Speed,0,&Can_TxMessageList[0]);
	PID_Init(&P_Speed->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000);
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		PID���ͺ���
//
//��������:
//		���͵��PID
//
//��������:
//		ROBO_BASE* ���̽ṹ��ָ��
//
//��ֲ����:
//		����Ҫɶ���Ŀ��ƾ���ָ��ָ�����ϵͳ, Ȼ����ö�Ӧ��PID���㺯�����д���
//
//--------------------------------------------------------------------------------------------------//
#if UART_COMMUNICATE_USER_ROOT
	weak
#endif
void Send_RoboBasePID(void)
{
  Pos_System* P_Pos=NULL;
  P_Pos=&Robo_Base.Pos_Motor; PID_Pos_Cal(P_Pos);
	Add_TxMessage(P_Pos->Speed_PID.output,P_Pos->TxMessage->Data);

  Speed_System* P_Speed=NULL;
  P_Speed=&Robo_Base.Speed_Motor; PID_Speed_Cal(P_Speed);
	Add_TxMessage(P_Speed->Speed_PID.output,P_Speed->TxMessage->Data);
	
	Motor_System* P_Motor=NULL;
  P_Motor=&Robo_Base.All_Motor; PID_General_Cal(&P_Motor->Speed_PID,P_Motor->Info.Speed,P_Motor->Tar_Speed);
	Add_TxMessage(P_Motor->Speed_PID.output,P_Motor->TxMessage->Data);
	
	CAN_Send(&Can_TxMessageList[0]);
}

#if SYSTEM_ENABLE
		//--------------------------------------------------------------------------------------------------//
	//��������:
	//		�ٶȻ�������ݷ����Ľӿں���
	//
	//��������:
	//		��ȡRobo_Base��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ�����ӵ���Ϣ, Ȼ�󴢴�������.
	//
	//��������:
	//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
	//		uint32_t �������
	//
	//--------------------------------------------------------------------------------------------------//
	#if UART_COMMUNICATE_USER_ROOT
		weak
	#endif
	void Motor_All_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
	{
		Motor_System* P_Motor=NULL;
		switch(Motor_Num)
		{
			case 0x201:P_Motor=&Robo_Base.All_Motor;break;
		default:break;
		}if(!P_Motor) return ;
		Motor_System_Analysis(&P_Motor->Info,RX_Data);
		#if WATCHDOG_ENABLE
			Feed_WatchDog(&P_Motor->Protect);
		#endif
	}
#endif

	
#if POS_SYSTEM_ENABLE
	//--------------------------------------------------------------------------------------------------//
	//��������:
	//		λ�û�������ݷ����Ľӿں���
	//
	//��������:
	//		��ȡRobo_Base��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ�����ӵ���Ϣ, Ȼ�󴢴�������.
	//
	//��������:
	//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
	//		uint32_t �������
	//
	//--------------------------------------------------------------------------------------------------//
	#if UART_COMMUNICATE_USER_ROOT
		weak
	#endif
	void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
	{
		Pos_System* P_Pos=NULL;
		switch(Motor_Num)
		{
			case 0x201:P_Pos=&Robo_Base.Pos_Motor;break;
		default:break;
		}if(!P_Pos) return ;
		Pos_System_Analysis(&P_Pos->Info,RX_Data);
		#if WATCHDOG_ENABLE
			Feed_WatchDog(&P_Pos->Protect);
		#endif
	}
#endif

#if SPEED_SYSTEM_ENABLE
	//--------------------------------------------------------------------------------------------------//
	//��������:
	//		�ٶȻ�������ݷ����Ľӿں���
	//
	//��������:
	//		��ȡRobo_Base��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ�����ӵ���Ϣ, Ȼ�󴢴�������.
	//
	//��������:
	//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
	//		uint32_t �������
	//
	//--------------------------------------------------------------------------------------------------//
	#if UART_COMMUNICATE_USER_ROOT
		weak
	#endif
	void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
	{
		Speed_System* P_Speed=NULL;
		switch(Motor_Num)
		{
			case 0x201:P_Speed=&Robo_Base.Speed_Motor;break;
		default:break;
		}if(!P_Speed) return ;
		Speed_System_Analysis(&P_Speed->Info,RX_Data);
		#if WATCHDOG_ENABLE
			Feed_WatchDog(&P_Speed->Protect);
		#endif
	}
#endif


