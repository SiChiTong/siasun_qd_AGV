/*
 * main.h
 *
 *  Created on: Jun 24, 2019
 *      Author: siasunhebo
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <can.h>
#include <Odom_Calib.h>
#include <Interpolation.h>

//struct Coordinate_Class Coor_delta;


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


#endif /* MAIN_H_ */
