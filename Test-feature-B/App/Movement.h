/*
 * Movement.h
 *
 *  Created on: Jun 28, 2019
 *      Author: siasunhebo
 */

#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_


#include <Interpolation.h>
#include <common.h>

//Coordinate_Class_t Origin_Coor_InWorld;         //起点坐标

//struct Coordinate_Class Destination_Coor_InOrigin;    //起点坐标系下的终点坐标

struct Coordinate_Class Virtual_AGV_Current_Coor_InWorld; //虚拟的AGV坐标用于路径跟随
struct Coordinate_Class AGV_Current_Coor_InWorld;


float current_distance ;
float x_temp_InOrigin;    //终点坐标在起点坐标系下的x轴偏移
float y_temp_InOrigin;    //终点坐标在起点坐标系下的y轴偏移
float angle_equivalent_temp_InOrigin;    //终点坐标在起点坐标系下的角度轴偏移(等效为线偏移)
float distance_InOrigin_ABS;             //终点坐标离起点坐标的距离绝对值

float X_H_mul_X;      //用于计算投影矩阵的值


float Cal_Destination_Displacement(Coordinate_Class_t Destination_Coor_InOrigin);
int Movement_Init();
Coordinate_Class_t Cal_Projection_Coor(Coordinate_Class_t Current_Coor_InOrigin);
int Movement_Cal_Velocity( struct Interpolation_Parameter_t Interpolation_Parameter);

#endif /* MOVEMENT_H_ */
