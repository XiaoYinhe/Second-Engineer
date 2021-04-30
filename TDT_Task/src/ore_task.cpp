/************
@brief 矿石标志位函数
@describe 
包含电机有：
气泵：翻转2个2006，在平移架上爬2个2006，2个气泵（一个摩擦轮模块）
肚子：转和夹3508和2006
摩擦轮模块
@work 分别写成标志位控制的形式，然后动作任务直接用标志位控制电机。
	  函数要写到头文件"ore_task.h"里,气缸直接translation=1即可
@function 
void Pumpturning_ctrl();
void ClimbPosition_ctrl();



*************/



#include "ore_task.h"
#include "moneyTaking_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h"					//FreeRTOS使用	 
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "motor.h"
#include "can.h"
#include "dbus.h"
#include "vision.h"


extern _ResetCmd ResetCmd;
_flagCmd flagCmd;
_count Count;


float Air_Cylinder[4];
u8 translation;//气缸平移
int16_t handleTranFeed = 0;
int16_t tranfeed=0;
enum trans_pos_name{middle,Far_right,Far_left};
double tran_position[3]={0,-165000,165000};

Motor Elevator[2] =
{
   Motor (M3508,CAN2,0x204),
   Motor (M3508,CAN2,0x205)

};


Motor Climbing[2] =
{
   Motor (M2006,CAN1,0x201),
   Motor (M2006,CAN1,0x202),
};
Motor Turning[2] =
{
   Motor (M2006,CAN1,0x203),
   Motor (M2006,CAN1,0x204),
};

Motor TurnTable(M3508,CAN2,0x201),Clip(M2006,CAN2,0x202),Transition(M3508,CAN2,0x203);

PidParam in_climb[2],out_climb[2];
PidParam in_turn[2],out_turn[2];
PidParam in_table[2],out_table[2];
PidParam in_clip[2],out_clip[2];
PidParam in_trans[2],out_trans[2];
PidParam in_elevator[2],out_elevator[2];

void TDT_Uplift_Three()
{
	/*抬升的两个电机0x201上升时顺时针，下降时逆时针，顺时针为正，0x202相反*/
		if(Count.Uplift_times == 1 && Count.Uplift_times_last == 0)   //上升到1
		{
			flagCmd.QuadraticCmd = 1;
		}
		else if((Count.Uplift_times == 1 && Count.Uplift_times_last == 2) || (Count.Uplift_times == 1 && Count.Uplift_times_last == 3))
		{
			flagCmd.QuadraticCmd = 0;																		//下降到1
		}
		else if((Count.Uplift_times == 2 && Count.Uplift_times_last == 1) || (Count.Uplift_times == 2 && Count.Uplift_times_last == 0))//上升到2
		{
			flagCmd.QuadraticCmd = 2;
		}
		else if(Count.Uplift_times == 2 && Count.Uplift_times_last == 3) //下降到2
		{
			flagCmd.QuadraticCmd = 3;
		}
		
		
	switch(Count.Uplift_times)																						
	{
		case 0:			
			out_elevator[0].resultMax = Elevator[0].getMotorSpeedLimit()/7;		
			out_elevator[1].resultMax = Elevator[0].getMotorSpeedLimit()/7;
			Elevator[0].ctrlPosition(0);
			Elevator[1].ctrlPosition(0);			
			break;
		case 1:																			
			if( flagCmd.QuadraticCmd == 1)					//上升到500
			{
				out_elevator[0].resultMax = Elevator[0].getMotorSpeedLimit()/5;
				out_elevator[1].resultMax = Elevator[0].getMotorSpeedLimit()/5;
				Elevator[0].ctrlPosition(Exchange_position);
				Elevator[1].ctrlPosition(-Exchange_position);			
			}
			if(flagCmd.QuadraticCmd == 0)					//下降到500
			{
				out_elevator[0].resultMax = Elevator[0].getMotorSpeedLimit()/7;
				out_elevator[1].resultMax = Elevator[0].getMotorSpeedLimit()/7;
				Elevator[0].ctrlPosition(Exchange_position);			
				Elevator[1].ctrlPosition(-Exchange_position);
			}
			break;
		case 2:
			if(flagCmd.QuadraticCmd == 2)					//上升到600
			{
				out_elevator[0].resultMax = Elevator[0] .getMotorSpeedLimit()/5;
				out_elevator[1].resultMax = Elevator[0] .getMotorSpeedLimit()/5;
				Elevator[0].ctrlPosition(Silver_position);
				Elevator[1].ctrlPosition(-Silver_position);
			}
			if(flagCmd.QuadraticCmd == 3)					//下降
			{
				out_elevator[0].resultMax = Elevator[0].getMotorSpeedLimit()/7;
				out_elevator[1].resultMax = Elevator[0].getMotorSpeedLimit()/7;
				Elevator[0].ctrlPosition(Silver_position);			
				Elevator[1].ctrlPosition(-Silver_position);		
			}
			break;
		case 3://上升到700
			
			out_elevator[0].resultMax = Elevator[0].getMotorSpeedLimit()/5;
			out_elevator[1].resultMax = Elevator[0].getMotorSpeedLimit()/5;
			Elevator[0].ctrlPosition(caught_falling_position);
			Elevator[1].ctrlPosition(-caught_falling_position);
			
			break;
		default:
			break;				
	}

			Count.Uplift_times_last = Count.Uplift_times;
		
			vTaskDelay(pdMS_TO_TICKS(5));
}



/*气泵吸盘转的两个2006.避免抽搐的函数，封为标志位,记得调参*/
void Pumpturning_ctrl()
{
	switch(flagCmd.pumpTurn)
	{
		case 0:
			Turning[0].ctrlPosition(UpPos + ResetCmd.LTurnNowPos);
			Turning[1].ctrlPosition(-UpPos + ResetCmd.RTurnNowPos);
			break;
		case 1:
			Turning[0].ctrlPosition(MidPos + ResetCmd.LTurnNowPos);		
		  Turning[1].ctrlPosition(-MidPos + ResetCmd.RTurnNowPos);	
			break;
		case 2:
			Turning[0].ctrlPosition(DownPos + ResetCmd.LTurnNowPos);
		  Turning[1].ctrlPosition(-DownPos + ResetCmd.RTurnNowPos);
			break;
		
		
	}


}
/*气泵吸盘爬的两个2006*/
void ClimbPosition_ctrl()
{
	switch(flagCmd.climb)
	{
		case 0:
			Climbing[0].ctrlPosition(0 + ResetCmd.LClimbNowPos);
			Climbing[1].ctrlPosition(0 + ResetCmd.RClimbNowPos);
			break;
		case 1:
			Climbing[0].ctrlPosition(Max_extension + ResetCmd.LClimbNowPos);
		  Climbing[1].ctrlPosition(-Max_extension + ResetCmd.RClimbNowPos);
			break;
	
	}
}


void Airsend()
{
	Air_Cylinder[0]=(int16_t)(translation<<4);
	Air_Cylinder[1]=1;
	Air_Cylinder[2]=1;
	Air_Cylinder[3]=1;

	canTx(Air_Cylinder,CAN1,0x113);	
}


void Transition_position_ctrl()
{
	if(flagCmd.firstliving == 0)	//firstliving由主控发过来，上电时为0，按键控制复位检测啥的，置1
	{
		switch(flagCmd.trans)
		{
			case 0:
				Transition.ctrlPosition(tran_position[middle] + tranfeed + handleTranFeed);
				break;
			case 1:
				Transition.ctrlPosition(tran_position[Far_right] + tranfeed + handleTranFeed);
				break;
			case 2:
				Transition.ctrlPosition(tran_position[Far_left] + tranfeed + handleTranFeed);
				break;
			case 3:		//调试用，及时停的
				Transition.ctrlSpeed(0,0);
				break;
		}

		
	}
	
//	if(flagCmd.firstliving != 0)//复位用，调节tranfeed值并把handleTranFeed=0
//	{
//		
//	}
	





}





void Ore_Task(void *pvParameters)
{
	//气泵吸盘爬配置
	for(int i = 0;i<2;i++)
	{
	
		Climbing[i].pidInner.setPlanNum(2);
		Climbing[i].pidOuter.setPlanNum(2);
	
		in_climb[i].kp = 3;
		in_climb[i].ki = 0;
		in_climb[i].kd = 0;
		in_climb[i].resultMax = Climbing[i].getMotorCurrentLimit();
		
		out_climb[i].kp = 0.5;
		out_climb[i].ki = 0;
		out_climb[i].kd = 0;	
		out_climb[i].resultMax = Climbing[i].getMotorSpeedLimit()/2;
		
		Climbing[i].pidInner.paramPtr = &in_climb[i];
		Climbing[i].pidOuter.paramPtr = &out_climb[i];
		
		Climbing[i].pidInner.fbValuePtr[0] = &Climbing[i].canInfo.speed;
		Climbing[i].pidOuter.fbValuePtr[0] = &Climbing[i].canInfo.totalEncoder;
		
		
	}
	
		for(int i = 0;i<2;i++)
	{
	
		Elevator[i].pidInner.setPlanNum(2);
		Elevator[i].pidOuter.setPlanNum(2);
	
		in_elevator[i].kp = 0;
		in_elevator[i].ki = 0;
		in_elevator[i].kd = 0;
		in_elevator[i].resultMax = Elevator[i].getMotorCurrentLimit();
		
		out_elevator[i].kp = 0;
		out_elevator[i].ki = 0;
		out_elevator[i].kd = 0;	
		
		
		Elevator[i].pidInner.paramPtr = &in_elevator[i];
		Elevator[i].pidOuter.paramPtr = &out_elevator[i];
		
		Elevator[i].pidInner.fbValuePtr[0] = &Elevator[i].canInfo.speed;
		Elevator[i].pidOuter.fbValuePtr[0] = &Elevator[i].canInfo.totalEncoder;
		
		
	}
	
	//气泵吸盘转配置
	for(int i = 0;i<2;i++)
	{
	
		Turning[i].pidInner.setPlanNum(2);
		Turning[i].pidOuter.setPlanNum(2);
	
		in_turn[i].kp = 3;
		in_turn[i].ki = 0;
		in_turn[i].kd = 0;
		in_turn[i].resultMax = Turning[i].getMotorCurrentLimit();
		
		out_turn[i].kp = 0.5;
		out_turn[i].ki = 0.01;
		out_turn[i].kd = 0;	
		out_turn[i].resultMax = Turning[i].getMotorSpeedLimit()/4;
		
		Turning[i].pidInner.paramPtr = &in_turn[i];
		Turning[i].pidOuter.paramPtr = &out_turn[i];
		
		Turning[i].pidInner.fbValuePtr[0] = &Turning[i].canInfo.speed;
		Turning[i].pidOuter.fbValuePtr[0] = &Turning[i].canInfo.totalEncoder;
		
		
	}
	
	
	
	/*肚子转盘TurnTable配置*/
	{
	TurnTable.pidInner.setPlanNum(2);
	TurnTable.pidOuter.setPlanNum(2);
	
	in_table[0].kp = 0;
	in_table[0].ki = 0;
	in_table[0].kd = 0;
	in_table[0].resultMax = TurnTable.getMotorCurrentLimit();
	
	out_table[0].kp = 0;
	out_table[0].ki = 0;
	out_table[0].kd = 0;	
	out_table[0].resultMax = TurnTable.getMotorSpeedLimit();
	
	TurnTable.pidInner.paramPtr = &in_table[0];
	TurnTable.pidOuter.paramPtr = &out_table[0];
	
	TurnTable.pidInner.fbValuePtr[0] = &TurnTable.canInfo.speed;
	TurnTable.pidOuter.fbValuePtr[0] = &TurnTable.canInfo.totalEncoder;
	}
	/*肚子夹子Clip配置*/
	{
	Clip.pidInner.setPlanNum(2);
	Clip.pidOuter.setPlanNum(2);
	
	in_clip[0].kp = 0;
	in_clip[0].ki = 0;
	in_clip[0].kd = 0;
	in_clip[0].resultMax = Clip.getMotorCurrentLimit();
	
	out_clip[0].kp = 0;
	out_clip[0].ki = 0;
	out_clip[0].kd = 0;	
	out_clip[0].resultMax = Clip.getMotorSpeedLimit();
	
	Clip.pidInner.paramPtr = &in_clip[0];
	Clip.pidOuter.paramPtr = &out_clip[0];
	
	Clip.pidInner.fbValuePtr[0] = &Clip.canInfo.speed;
	Clip.pidOuter.fbValuePtr[0] = &Clip.canInfo.totalEncoder;
	
	}

	//平移架配置
	{
	Transition.pidInner.setPlanNum(2);
	Transition.pidOuter.setPlanNum(2);
	
	in_trans[0].kp = 0;//9;
	in_trans[0].ki = 0;//30;
	in_trans[0].kd = 0;
	in_trans[0].resultMax = 13500;//Transition.getMotorCurrentLimit();
	
	out_trans[0].kp = 0;//0.27;
	out_trans[0].ki = 0;
	out_trans[0].kd = 0;	
	out_trans[0].resultMax = 2000;//Transition.getMotorSpeedLimit()/4;
	
	Transition.pidInner.paramPtr = &in_trans[0];
	Transition.pidOuter.paramPtr = &out_trans[0];
	
	Transition.pidInner.fbValuePtr[0] = &Transition.canInfo.speed;
	Transition.pidOuter.fbValuePtr[0] = &Transition.canInfo.totalEncoder;
	
	
	}
	
	
	
	while(1)	
	{
//		TDT_Uplift_Three();
//		ClimbPosition_ctrl();
//		Pumpturning_ctrl();
//		Transition_position_ctrl();
		Transition.ctrlSpeed(0,0);
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	


}
