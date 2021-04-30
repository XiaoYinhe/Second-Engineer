#ifndef __ORE_TASK_H__
#define __ORE_TASK_H__

#include "board.h"

#define UpPos 0
#define MidPos 71000 
#define DownPos	142000
#define Max_extension 565000

#define Ban 10

#define Exchange_position  240000
#define Silver_position  300000
#define caught_falling_position 350000


typedef struct{
	
	u8 pumpTurn = Ban;
	u8 climb = Ban;
	u8 pump = Ban;
	u8 clip = Ban;
	u8 turnTable = Ban;
	u8 firstliving = 0;
	u8 trans = Ban;
	
	u8 Elevator_position = 0;
	u8 QuadraticCmd = 0;
	

}_flagCmd;


typedef struct{
	int Uplift_times;
	int Uplift_times_last;

}_count;


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

void TDT_Uplift_Three(void);




#endif