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
enum pumpTurn_sta{upper,horizontal,below};
enum climb_sta{zero_point,fully_extend};
float Air_Cylinder[4];
u8 translation;
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

Motor TurnTable(M3508,CAN2,0x201),Clip(M2006,CAN2,0x202);

PidParam in_climb[2],out_climb[2];
PidParam in_turn[2],out_turn[2];
PidParam in_table[2],out_table[2];
PidParam in_clip[2],out_clip[2];

/*气泵吸盘转的两个2006.避免抽搐的函数，封为标志位,记得调参*/
void Pumpturning_ctrl()
{
	switch(flagCmd.pumpTurn)
	{
		case upper:
			Turning[0].ctrlPosition(UpPos + ResetCmd.LTurnNowPos);
			Turning[1].ctrlPosition(-UpPos + ResetCmd.RTurnNowPos);
			break;
		case horizontal:
			Turning[0].ctrlPosition(MidPos + ResetCmd.LTurnNowPos);		
		  Turning[1].ctrlPosition(-MidPos + ResetCmd.RTurnNowPos);	
			break;
		case below:
			Turning[0].ctrlPosition(DownPos + ResetCmd.LTurnNowPos);
		  Turning[1].ctrlPosition(-DownPos + ResetCmd.RTurnNowPos);
			break;
		
		break;
	}


}
/*气泵吸盘爬的两个2006*/
void ClimbPosition_ctrl()
{
	switch(flagCmd.climb)
	{
		case zero_point:
			Climbing[0].ctrlPosition(0 + ResetCmd.LClimbNowPos);
			Climbing[1].ctrlPosition(0 + ResetCmd.RClimbNowPos);
			break;
		case fully_extend:
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

void Ore_Task(void *pvParameters)
{
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

	/*肚子夹子Clip配置*/
	
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
	
	

	while(1)	
	{
	  ClimbPosition_ctrl();
    Pumpturning_ctrl();
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	


}
