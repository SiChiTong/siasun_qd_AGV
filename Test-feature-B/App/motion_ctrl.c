//
// Created by maxiao on 8/26/19.
//

#include <stdio.h>
#include "stdlib.h"
#include <stdint-gcc.h>
#include <common.h>
#include <main.h>
#include "motion_ctrl.h"
#include "Movement.h"
#include<agv.h>
#include <PGV150/PGV150.h>

uint16_t ethCommBreakFlag = 0x00;
uint16_t Motionstyle      = MOTIONSTATE_ONSTOPING;

extern int virtual_agv_coor_init_flag;
extern struct Coordinate_Class Virtual_AGV_Current_Coor_InWorld;
//struct Interpolation_Parameter_t Interpolation_Parameter_m;
extern Coordinate_Class_t Destination_Coor_InWorld;     //目标位置
Velocity_Class_t Virtual_AGV_Current_Velocity_InAGV;
extern Velocity_Class_t Target_Velocity_InAGV;     //目标速度
extern Coordinate_Class_t Target_Coor_InWorld;
struct Interpolation_Parameter_t Interpolation_Parameter;    //插补参数、参数暂存
extern struct Coordinate_Class AGV_Current_Coor_InWorld;

extern int Add_Command_Line, Add_Command_Rotate;

extern Coordinate_Class_t AGV_Target_Coor_InWorld;               //AGV在世界坐标系下的期望坐标
Velocity_Class_t AGV_Target_Velocity_InAGV;               //AGV在AGV坐标系下的期望速度
extern Coordinate_Class_t Error_Coor_InAGV;                      //偏差坐标

extern float VL_1200;    //发送给PLC左轮速度
extern float VR_1200;    //发送给PLC右轮速度

float wControl;   //旋转角速度

extern float PID_result;
extern int judge_axis;
extern float target_velocity;

extern float acc_distance;    //加速段距离(mm)
extern float const_distance;  //匀速段距离(mm)
extern float dec_distance;    //减速段距离(mm)
extern float slowly_distance; //慢速段距离(mm)

int stop = 0;
 int stop_flage = 0;
//enum Interpolation_State_Enum State;

void Run_Movement_Class( )
{
	static enum Interpolation_State_Enum Interpolation_State;

     //printf("插补状态=%d\n",State);
    switch (State)
    {

    case No_Interpolation:  //未插补
    	if (Movement_Init() == 1)
    	{
    		State = IS_Interpolating;    //IS_Interpolating
    	}
    	else if (Movement_Init() == -1)
        {
            State = No_Interpolation;
        }
    	break;


    case IS_Interpolating:   //插补中
    	if (!Movement_Cal_Velocity( Interpolation_Parameter))    //插补完成
    	{
    		State = IS_Interpolated;
            Target_Velocity_InAGV.velocity_x = target_velocity;
            Virtual_AGV_Current_Velocity_InAGV.velocity_x=Target_Velocity_InAGV.velocity_x;
    	}
    	else   //还在插补，获取虚拟的坐标和速度
    	{
             Target_Velocity_InAGV.velocity_x = target_velocity;
             Virtual_AGV_Current_Velocity_InAGV.velocity_x=Target_Velocity_InAGV.velocity_x;
    	}
    	break;


    case IS_Interpolated:  //插补结束
    	if(stop == 0)
    	{
    		//printf("插补结束\n");
    		State = IS_Interpolated;
    		if (data_Ok==1)
    			Target_Velocity_InAGV.velocity_x = 55.0* Distance_Symbols;
    		else
    			Target_Velocity_InAGV.velocity_x = 65.0* Distance_Symbols;
    	}

    	else if(stop == 1)
    	{
    		current_distance = 0.0;
    		acc_distance = const_distance = dec_distance = slowly_distance = 0.0;
    		virtual_agv_coor_init_flag = 0;
    		State = No_Interpolation;
    	}
    	break;

    default:
   	break;
    }
}


void Movement_Control()
{
	float V_err;
    float vx;

    vx = Target_Velocity_InAGV.velocity_x;

    	switch(judge_axis)
    	{
    		case 1: //X方向
    			if ((abs(PGV150_coor.x_coor-Destination_Coor_InWorld.x_coor) < 1.5) )  //停车区间
    			{
    				//printf("**stop_X**\n");
    				Target_Velocity_InAGV.velocity_x = 0.0;
    			     VL_1200 = 0.0;
    			     VR_1200 = 0.0;
    			     stop = 1;
    			     stop_flage++;
    			}
    	    	else
    	    	{
    	    		stop = 0;
    	           	VL_1200 = vx + PID_result;
    	        	 VR_1200 = vx - PID_result;
    	        }
    			break;

    		case 2: //Y方向
    			if ((abs(PGV150_coor.y_coor-Destination_Coor_InWorld.y_coor) < 1.5) )  //停车区间
    			{
    				//printf("**stop_Y**\n");
    				Target_Velocity_InAGV.velocity_x = 0.0;
    				VL_1200 = 0.0;
    				VR_1200 = 0.0;
         		     stop = 1;
         		   stop_flage++;
    			}
    	    	else
    	    	{
    	    		stop =0;
    	    		VL_1200 = vx + PID_result;
    	    		VR_1200 = vx - PID_result;
    	        }
    			break;
    	}
}
