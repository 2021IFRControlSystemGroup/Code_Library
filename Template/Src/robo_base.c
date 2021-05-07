//文件名称:		robo_base.c
//对应头文件:	robo_base.h
//主要功能:
//		调用motor_system封装的速度环/位置环/看门狗函数, 实现底盘控制
//
//时间:
//		2021/5/8
//
//版本:	3.0V
//
//---------头文件引用部分---------//
#include "robo_base.h"
//--------------------------------//

//---------变量声明部分-----------//
ROBO_BASE Robo_Base;
//--------------------------------//


//--------------------------------------------------------------------------------------------------//
//函数名称:
//		底盘参数初始化
//
//函数功能:
//		初始化底盘所有的信息
//
//参数类型:
//		ROBO_BASE 指针, 底盘结构体的指针
//
//移植建议:
//		有什么状态, 电机, 电机状态都先把数据封装进ROBO_BASE结构体里, 然后直接初始化就好了
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
//函数名称:
//		PID发送函数
//
//函数功能:
//		发送电机PID
//
//参数类型:
//		ROBO_BASE* 底盘结构体指针
//
//移植建议:
//		有需要啥环的控制就让指针指向这个系统, 然后调用对应的PID计算函数进行处理
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
	//函数名称:
	//		速度环电机数据分析的接口函数
	//
	//函数功能:
	//		读取Robo_Base对应的CAN口储存的数据, 根据电机号码来分辨是哪一个轮子的信息, 然后储存电机数据.
	//
	//参数类型:
	//		uint8_t* 电机信息的数组, 推荐使用Rx_CAN变量, 这样可以不需要自己去声明.
	//		uint32_t 电机号码
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
	//函数名称:
	//		位置环电机数据分析的接口函数
	//
	//函数功能:
	//		读取Robo_Base对应的CAN口储存的数据, 根据电机号码来分辨是哪一个轮子的信息, 然后储存电机数据.
	//
	//参数类型:
	//		uint8_t* 电机信息的数组, 推荐使用Rx_CAN变量, 这样可以不需要自己去声明.
	//		uint32_t 电机号码
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
	//函数名称:
	//		速度环电机数据分析的接口函数
	//
	//函数功能:
	//		读取Robo_Base对应的CAN口储存的数据, 根据电机号码来分辨是哪一个轮子的信息, 然后储存电机数据.
	//
	//参数类型:
	//		uint8_t* 电机信息的数组, 推荐使用Rx_CAN变量, 这样可以不需要自己去声明.
	//		uint32_t 电机号码
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


