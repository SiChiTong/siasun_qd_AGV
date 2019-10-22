/*
 * main.c
 *
 *  Created on: Jun 24, 2019
 *      Author: maxiao
 */


#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>
#include <sys/types.h>
#include <can.h>
#include <MPI204A.h>
#include <time.h>
#include <signal.h>
#include <Timer.h>
#include <PGV150.h>
#include <common.h>
#include <Movement.h>
#include <1200.h>
#include <TCP/TCP.h>
#include <MPI204A.h>
#include <zconf.h>
#include <App/motion_ctrl.h>
#include "AGVControl.h"
#include "AGV/agv.h"
#include "motion_ctrl.h"



Coordinate_Class_t Destination_Coor;    //目标坐标





extern Interpolation_State_Enum_m Interpolation_State;

extern int command_x;
extern int command_y;
extern int command_angle;



char buffer[8];    //发送PLC数组

float VL_1200;    //发送给PLC左轮速度
float VR_1200;    //发送给PLC右轮速度


Coordinate_Class_t Destination_Coor_InWorld;     //目标位置



int Add_Command_Line, Add_Command_Rotate;





//由触摸屏得到目标坐标
void Gcode_G0()
{
/*
	static int x;
	static int y;

	 if(abs(AGV_Current_Coor_InWorld.y_coor-900)<7 && abs(AGV_Current_Coor_InWorld.x_coor-700)<3){
	   x = 700;//700/0
	   y = 2000;//3300/500
	}

	else if(abs(AGV_Current_Coor_InWorld.y_coor-2000)<3 && abs(AGV_Current_Coor_InWorld.x_coor-700)<7){
		x = 0;
		y = 2000;
	}

	else if(abs(AGV_Current_Coor_InWorld.x_coor) < 3 && abs(AGV_Current_Coor_InWorld.y_coor - 2000) < 7){
		x = 0;
		y = 900;
	}
	else if(abs(AGV_Current_Coor_InWorld.x_coor ) < 7 && abs(AGV_Current_Coor_InWorld.y_coor - 900) <3){
		x = 700;
		y = 900;
	}
/*
	else if(abs(AGV_Current_Coor_InWorld.x_coor -700) < 7 && abs(AGV_Current_Coor_InWorld.y_coor - 1200) < 2){
		x = 700;
		y=3300;
	}

*/

	Destination_Coor_InWorld.x_coor = command_x * 1.0;
	//Destination_Coor_InWorld.x_coor = x;
	Destination_Coor_InWorld.y_coor = command_y * 1.0;
	//Destination_Coor_InWorld.y_coor = y;
   // Destination_Coor.angle_coor = Destination_Coor_InWorld.angle_coor; //屏幕获取目标角度
    printf("目标点位： %f, %f,  %f\n", Destination_Coor_InWorld.x_coor, Destination_Coor_InWorld.y_coor, Destination_Coor_InWorld.angle_coor);
    //printf("C_X = %d, C_Y = %d, C_A = %d\n", command_x, command_y, command_angle);

    // printf("目标点位：%d,%d\n",x,y);
    //Coordinate_Class_t Destination_Coor_temp;
    if ((Add_Command_Line == 1) || (Add_Command_Rotate == 1))
    {
        //Destination_Coor_temp = Destination_Coor;
       // Destination_Coor_InWorld = Destination_Coor;
        Interpolation_State = No_Interpolation;
    }

}





//系统初始化

void init_System()
{
    paramter_define();  //参数定义

    MPI204A_init();      //IMU初始化

    PGV_init();          //相机初始化
    get_PLC_init();      //1200数据接收初始化


    get_pthread();  //开TCP线程
    get_PID();      //开PID线程
}




int main ()
{


	typedef union{
		int fdata;
		char ldata[4];
	}VL;
	typedef union{
		int fdata;
		char ldata[4];
	}VR;

	VR Vr;
	VL Vl;

	VR_1200 = 0.0;
	VL_1200 = 0.0;


    init_System();  //初始化



	while(1)
	{
	    printf("VL_1200=%d, VR_1200=%d\n", Vl.fdata, Vr.fdata);
        Vl.fdata = (int)(VL_1200 * 1000);
        Vr.fdata = (int)(VR_1200 * 1000);
        buffer[0] = Vl.ldata[0];
        buffer[1] = Vl.ldata[1];
        buffer[2] = Vl.ldata[2];
        buffer[3] = Vl.ldata[3];
        buffer[4] = Vr.ldata[0];
        buffer[5] = Vr.ldata[1];
        buffer[6] = Vr.ldata[2];
        buffer[7] = Vr.ldata[3];



		get_PLC_Data();     //获取车体信息
		Location_AGV();         //AGV定位
        Gcode_G0();  //由触摸屏得到目标坐标

		printf("command_line = %d, command_rotate = %d\n", Add_Command_Line, Add_Command_Rotate);

		Prase_Sensor_Data();    //传感器处理
        DirectionDetermination();  //判断车体的运行方向

		AGV_RUN();



	}

}
