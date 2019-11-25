/*
 * Movement.c
 *
 *  Created on: Jun 28, 2019
 *      Author: maxiao
 */


#include <Movement.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

extern Coordinate_Class_t Destination_Coor_InWorld;    //终点坐标
extern float target_velocity;
extern struct Interpolation_Parameter_t Interpolation_Parameter;    //插补参数、参数暂存

float input_distance_abs; //待插补距离
Velocity_Class_t Target_Velocity_InAGV;     //目标速度
Coordinate_Class_t Target_Coor_InWorld;



//************************************
// FullName:  Movement_Init
// Returns:   int
// Parameter: struct Coordinate_Class Origin_Coor 起点坐标
// Parameter: struct Interpolation_Parameter_t Input_Para  插补参数
// Description: 计算加减速时间及距离
//************************************
int Movement_Init()
{
	struct Coordinate_Class Destination_Coor_InOrigin;    //起点坐标系下的终点坐标
	int temp_result;

	Destination_Coor_InOrigin.x_coor = Destination_Coor_InWorld.x_coor - Virtual_AGV_Current_Coor_InWorld.x_coor;    //计算目标点在起点坐标系下的相对坐标
	Destination_Coor_InOrigin.y_coor = Destination_Coor_InWorld.y_coor - Virtual_AGV_Current_Coor_InWorld.y_coor;

	if (abs(Destination_Coor_InWorld.x_coor-AGV_Current_Coor_InWorld.x_coor) > 90 || abs(Destination_Coor_InWorld.y_coor-AGV_Current_Coor_InWorld.y_coor) > 90 )
    {
         input_distance_abs = sqrtf(Destination_Coor_InOrigin.x_coor*Destination_Coor_InOrigin.x_coor+Destination_Coor_InOrigin.y_coor*Destination_Coor_InOrigin.y_coor);
        temp_result = Interpolation_Init(input_distance_abs, Interpolation_Parameter);
    }

	else
	    temp_result = -1;

	return temp_result;
}
//************************************
// FullName:  Movement_Cal_Velocity
// Returns:   int
// Parameter: Coordinate_Class_t Current_Coor_InWorld
// Description: 插补速度
//************************************
int Movement_Cal_Velocity( struct Interpolation_Parameter_t Interpolation_Parameter)
{
	static float diatance;//避免摄像头数据突然增大

	Coordinate_Class_t Current_Coor_InOrigin;
	Current_Coor_InOrigin.x_coor = AGV_Current_Coor_InWorld.x_coor - Virtual_AGV_Current_Coor_InWorld.x_coor;    //获取当前坐标在起点坐标系下的坐标
	Current_Coor_InOrigin.y_coor = AGV_Current_Coor_InWorld.y_coor - Virtual_AGV_Current_Coor_InWorld.y_coor;
	//Current_Coor_InOrigin.angle_coor = AGV_Current_Coor_InWorld.angle_coor - Virtual_AGV_Current_Coor_InWorld.angle_coor;
	//Current_Coor_InOrigin.angle_rad = AGV_Current_Coor_InWorld.angle_rad - Virtual_AGV_Current_Coor_InWorld.angle_rad;

	current_distance = sqrtf( ( Current_Coor_InOrigin.x_coor) * (Current_Coor_InOrigin.x_coor) + ( Current_Coor_InOrigin.y_coor) * ( Current_Coor_InOrigin.y_coor) );
	if((current_distance -diatance) > 30)//
		current_distance=diatance;//

	 diatance = current_distance;//
	int Inter_result = Interpolation_Cal_Velocity(current_distance, Interpolation_Parameter);     //计算插补结果

	Velocity_Class_t Target_Velocity_InOrigin;    //期望坐标坐标系下的速度

	 Target_Velocity_InAGV.velocity_x = target_velocity;
		//printf("Inter_result=%d\n",Inter_result);
	return Inter_result;

}





