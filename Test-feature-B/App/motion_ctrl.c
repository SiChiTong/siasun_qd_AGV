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
	//struct Coordinate_Class Coor = virtual_agv_coor_init_flag ? Virtual_AGV_Current_Coor_InWorld : AGV_Current_Coor_InWorld;

if (Add_Command_Line == 1)
 {
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
    	if (!Movement_Cal_Velocity(AGV_Current_Coor_InWorld, Interpolation_Parameter))    //插补完成
    	{
    		State = IS_Interpolated;
    		// State = No_Interpolation;

    		 //virtual_agv_coor_init_flag = 0;

    		//Virtual_AGV_Current_Coor_InWorld = Destination_Coor_InWorld;
            Virtual_AGV_Current_Velocity_InAGV = Target_Velocity_InAGV;
            printf ("Interpolat OK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

    	}
    ///*
    	else   //还在插补，获取虚拟的坐标和速度
    	{
    	//	Virtual_AGV_Current_Coor_InWorld = Target_Coor_InWorld;
            Virtual_AGV_Current_Velocity_InAGV = Target_Velocity_InAGV;
    	}
    //	*/
    	break;

    case IS_Interpolated:  //插补结束
    	//printf("插补结束\n");
    	if(stop == 0){
    		printf("插补结束\n");
    		State = IS_Interpolated;
    		if (data_Ok==1)
    		Virtual_AGV_Current_Velocity_InAGV.velocity_x = 56.0* Distance_Symbols;
    		else
    			Virtual_AGV_Current_Velocity_InAGV.velocity_x = 70.0* Distance_Symbols;
    	}

    	else if(stop == 1){
    		current_distance = 0.0;
    		acc_distance = const_distance = dec_distance = slowly_distance = 0.0;
    		virtual_agv_coor_init_flag = 0;
    		//Virtual_AGV_Current_Velocity_InAGV.velocity_x = 0.0;
    		State = No_Interpolation;
    	}

    	break;

    //default:
   	//break;

    }
 }
  //  return 0;

}

/*
void Process_Movement_Command()
{
    static enum Interpolation_State_Enum Interpolation_State;

    //Run_Movement_Class(AGV_Current_Coor_InWorld, Interpolation_Parameter);   //执行运动指令并返回结果
   static int Is_Parsing_Movement = 0;     //指示当前是否在执行运动指令
    if (!Is_Parsing_Movement)
    {
        Is_Parsing_Movement = !Run_Movement_Class(AGV_Current_Coor_InWorld, Interpolation_Parameter);   //执行运动指令并返回结果

    }
    else
    {
        Is_Parsing_Movement = !Run_Movement_Class(AGV_Current_Coor_InWorld, Interpolation_Parameter);    //执行运动指令并返回结果
    }

}
*/

void Movement_Control()
{
	float V_err;
    float vx;

    vx = Virtual_AGV_Current_Velocity_InAGV.velocity_x;

  //  printf("vx = %f\n", vx);

    //如果直行
    if (Add_Command_Line == 1)
    {
    	//printf("Error_X = %f,Error_Y = %f\n",Error_Coor_InAGV.x_coor,Error_Coor_InAGV.y_coor);
    	switch(judge_axis)
    	{
    		case 1: //X方向
    			if ((abs(PGV150_coor.x_coor-Destination_Coor_InWorld.x_coor) < 1.5) )
    			{
    				printf("**stop_X**\n");
    				Virtual_AGV_Current_Velocity_InAGV.velocity_x = 0.0;
    			     VL_1200 = 0.0;
    			     VR_1200 = 0.0;
    			     stop = 1;
    			     ++stop_flage;
    			}
    	    	else
    	    	{
    	    		stop = 0;
    	    		/*
    	        	if (abs(vx - PID_result) - abs(vx + PID_result)>=1 ) //0/1/2
    	        		{
    	        			//printf("偏外\n");
/*
    	        		if (abs(vx) < 900)
    	        			V_err =(abs(vx - PID_result) - abs(vx + PID_result))*Z;
    	        		else

    	        			//V_err =(abs(vx - PID_result) - abs(vx + PID_result))*(5.15*abs(vx)*0.001-4.725)*2;
    	        			V_err =(abs(vx - PID_result) - abs(vx + PID_result))*Z;

    	        			VL_1200 = vx + PID_result;
    	        			  VR_1200 = vx - PID_result;
    	        			//VR_1200 = vx - PID_result+V_err*Distance_Symbols;
    	        		}*/

    	        			//printf("偏里\n");
    	        	        VL_1200 = vx + PID_result;
    	        	        VR_1200 = vx - PID_result;

    	        }
    			break;

    		case 2: //Y方向
    			if ((abs(PGV150_coor.y_coor-Destination_Coor_InWorld.y_coor) < 1.5) )
    			{
    				printf("**stop_Y**\n");
    				Virtual_AGV_Current_Velocity_InAGV.velocity_x = 0.0;
    				VL_1200 = 0.0;
    				VR_1200 = 0.0;
         		     stop = 1;
         		   ++stop_flage;
    			}
    	    	else
    	    	{
    	    		stop =0;
    	        /*	if (abs(vx - PID_result) - abs(vx + PID_result)>=1 ) //0/1/2
    	        		{
    	        			//printf("偏外\n");
    	        			V_err =(abs(vx - PID_result) - abs(vx + PID_result))*(5.15*abs(vx)*0.001-4.725)*2;

    	        	    	//printf ("速度插补= %f\n",V_err);

    	        			VL_1200 = vx + PID_result;
    	        			  VR_1200 = vx - PID_result+V_err*Distance_Symbols;
    	        			//VR_1200 = vx - PID_result+V_err*Distance_Symbols;

    	        		}
    	        	else
    	        	     {
    	        		//	printf("偏里\n");
    	        	        VL_1200 = vx + PID_result;
    	        	        VR_1200 = vx - PID_result;
    	        	        	}*/

    	    		  VL_1200 = vx + PID_result;
    	    		   VR_1200 = vx - PID_result;
    	        }
    			break;
    	}
    }

/*
        //如果旋转
    else if (Add_Command_Rotate == 1)
    {
        if ((Error_Coor_InAGV.angle_coor <= 10.0) && (Error_Coor_InAGV.angle_coor > -10.0))
        {
            VL_1200 = 0.0;
            VR_1200 = 0.0;
            stop = 1;
        }
        else
        {
            wControl = 0.45;
            VL_1200 =  wControl * wheel_distance / 2.0;
            VR_1200 = -wControl * wheel_distance / 2.0;
            //VL_1200 =  130.905;
            //VR_1200 = -130.905;
            stop = 0;
        }
    }
*/
}
