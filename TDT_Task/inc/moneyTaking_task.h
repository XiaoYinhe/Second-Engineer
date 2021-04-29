#ifndef __MONEYTAKING_TASK_H__
#define __MONEYTAKING_TASK_H__

#include "board.h"

typedef struct{
	
	u8 ExStart;
	u8 ExStep;
	u8 OrePos;
  u8 OreStep;
	int TimeDelay;
		
}_ExCmd;

typedef struct{
	
	u8 AllDone;
	u8 ClimbDone;
	long long int LClimbNowPos;
	long long int RClimbNowPos;
	u8 TurnDone;
	long long int LTurnNowPos;
	long long int RTurnNowPos;
	int TimeDelay;

}_ResetCmd;

/*矿石兑换经济*/
void Exchange(void);

/*吸盘复位*/
void SuctionRest(void);




#endif