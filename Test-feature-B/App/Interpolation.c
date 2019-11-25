/*
 * Interpolation.c
 *
 *  Created on: Jun 28, 2019
 *      Author: siasunhebo
 */


#include <Interpolation.h>
#include <math.h>
#include <stdlib.h>
#include<stdio.h>
#include<motion_ctrl.h>
//#include<main.h>

//extern struct Interpolation_Parameter_t Interpolation_Parameter;    //插补参数、参数暂存

//int Distance_Symbols = 1;  //指示待插补距离的符号
float Interpolation_Distance = 0.0;


float target_velocity = 0.0;
float target_distance = 0.0;

float acc_distance = 0.0;    //加速段距离(mm)
float const_distance = 0.0;  //匀速段距离(mm)
float dec_distance = 0.0;    //减速段距离(mm)
float slowly_distance = 0.0; //慢速段距离(mm)


float max_velocity_abs;
Interpolation_State_Enum_m Interpolation_State;

///************************************
// FullName:  Update_Interpolation_Parameter
// Returns:   NULL
// Parameter: struct Interpolation_Parameter_t Input_Para
// Description: 更新待插补参数
//************************************
struct Interpolation_Parameter_t Update_Interpolation_Parameter(struct Interpolation_Parameter_t Input_Para)
{
    struct Interpolation_Parameter_t Interpolation_Parameter;
	Interpolation_Parameter = Input_Para;
}



//************************************
// FullName:  Interpolation_Init
// Returns:   int
// Parameter: float distance
// Description: 插补加减速时间及距离
//************************************
int Interpolation_Init(float distance, struct Interpolation_Parameter_t Interpolation_Parameter)
{

	float slowly_distance_temp = Interpolation_Parameter.slow_time_abs * Interpolation_Parameter.min_velocity_abs;   //计算最低速移动距离
    //加减速段移动距离，用来判断是否存在匀速段：2S
	float distance_acc_dec_temp = (Interpolation_Parameter.max_velocity_abs * Interpolation_Parameter.max_velocity_abs - Interpolation_Parameter.min_velocity_abs * Interpolation_Parameter.min_velocity_abs) /(Interpolation_Parameter.accleration_abs);
    float input_distance_abs  = distance;  //待插补距离

    //复位目标距离和速度
    target_velocity = target_distance = 0.0;

    input_distance_abs =input_distance_abs- slowly_distance_temp;    //先减去慢速段距离

   if (input_distance_abs < distance_acc_dec_temp)    //不存在匀速段
    {
	    max_velocity_abs = sqrtf(input_distance_abs * Interpolation_Parameter.accleration_abs + Interpolation_Parameter.min_velocity_abs * Interpolation_Parameter.min_velocity_abs);  //最高速计算
    	acc_distance = dec_distance = input_distance_abs*0.5;    //加减速距离
    	const_distance = 0;  //匀速段距离
    }
    else     //存在匀速段
    {
    	 acc_distance = dec_distance = distance_acc_dec_temp*0.5;    //加减速距离
    	 const_distance =input_distance_abs - distance_acc_dec_temp;    //匀速距离
    	 max_velocity_abs = Interpolation_Parameter.max_velocity_abs;
    }

    slowly_distance =slowly_distance_temp;//低速距离

    return 1;

}


//************************************
// FullName:  Interpolation_Cal_Velocity
// Returns:   int
// Parameter: float current_distance
// Description: 根据距离插补速度
//************************************
int Interpolation_Cal_Velocity(float current_distance, struct Interpolation_Parameter_t Interpolation_Parameter)
{
	//printf("current_distance=%f\n",current_distance);
	//printf("acc_distance=%f,const_distance=%f,dec_distance=%f\n,slowly_distance=%f\n",acc_distance,const_distance,dec_distance,slowly_distance);
	//获取插补速度


/*	if (current_distance < 0.0)
	{
		printf("1 !!!!!!\n");
		target_velocity = Interpolation_Parameter.min_velocity_abs * Distance_Symbols;
	}
	*/

	 if (current_distance < acc_distance)    //在加速区
	{
		target_velocity = sqrtf(2 * current_distance * Interpolation_Parameter.accleration_abs + Interpolation_Parameter.min_velocity_abs * Interpolation_Parameter.min_velocity_abs) * Distance_Symbols;
	}
	else if (current_distance < (acc_distance + const_distance))    //在匀速区
	{
		target_velocity = max_velocity_abs * Distance_Symbols;
	}
	else if (current_distance < (acc_distance + const_distance + dec_distance))    //在减速区
	{
		target_velocity = sqrtf(max_velocity_abs * max_velocity_abs - 2 * (current_distance - acc_distance - const_distance) * Interpolation_Parameter.accleration_abs) * Distance_Symbols;
	}
	else if (current_distance < (acc_distance + const_distance + dec_distance + slowly_distance)-8) //在慢速区
	{
		target_velocity = Interpolation_Parameter.min_velocity_abs * Distance_Symbols;
	}

	else       //插补结束
	{
		 target_velocity = Interpolation_Parameter.min_velocity_abs * Distance_Symbols;

		State = IS_Interpolated;
		return 0;
	}

	return (State != IS_Interpolated);    //返回插补结果，插补成功返回0
}






