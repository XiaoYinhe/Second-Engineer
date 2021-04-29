#ifndef __ORE_TASK_H__
#define __ORE_TASK_H__

#include "board.h"

#define UpPos 0
#define MidPos 71000 
#define DownPos	142000
#define Max_extension 565000
#define Ban 10




typedef struct{
	
	u8 pumpTurn = Ban;
	u8 climb = Ban;
	u8 pump = Ban;
	u8 clip = Ban;
	u8 turnTable = Ban;



}_flagCmd;



typedef struct{

	u8 climb_STA;
	u8 pumpTurn_STA;
	u8 pump_STA;


}_STA;

/*º¯ÊýÉùÃ÷*/
void Pumpturning_ctrl();
void ClimbPosition_ctrl();
void Airsend();

extern _flagCmd flagCmd;
extern _STA STA;



#endif