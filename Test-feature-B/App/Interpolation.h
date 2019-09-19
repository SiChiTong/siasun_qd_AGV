/*
 * Interpolation.h
 *
 *  Created on: Jun 28, 2019
 *      Author: siasunhebo
 */

#ifndef _INTERPOLATION_H_
#define _INTERPOLATION_H_


int Distance_Symbols ;  //指示待插补距离的符号

struct Interpolation_Parameter_t   //插补参数、参数暂存
{
	float max_velocity_abs;    //最大速度mm/s
	float min_velocity_abs;    //最小速递mm/s
	float accleration_abs;     //加速度
	float slow_time_abs;       //最小速度移动时间
};   //插补参数结构体定义

typedef enum Interpolation_State_Enum
{
	No_Interpolation,         //未插补
	IS_Interpolating,         //插补中
	IS_Interpolated           //插补完成
}Interpolation_State_Enum_m;                   //插补状态



int Interpolation_Init(float distance, struct Interpolation_Parameter_t Interpolation_Parameter);
struct Interpolation_Parameter_t Update_Interpolation_Parameter(struct Interpolation_Parameter_t Input_Para);
int Interpolation_Cal_Velocity(float current_distance, struct Interpolation_Parameter_t Interpolation_Parameter);


#endif /* INTERPOLATION_H_ */
