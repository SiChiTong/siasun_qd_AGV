//
// Created by siasunhebo on 9/12/19.
//
#include <stdio.h>
#include <common.h>
#include <main.h>
#include <PGV150/PGV150.h>
#include <zconf.h>
#include <MPI204A/MPI204A.h>
#include "agv.h"
#include <math.h>
#include <Odom_Calib.h>
#include "motion_ctrl.h"
#include <stdint-gcc.h>
#include "stdlib.h"
#include <pthread.h>
//#include <Thread_Pool.h>

extern uint16_t Motionstyle;
wheel_paramter_t wheelParamter;
extern struct Interpolation_Parameter_t Interpolation_Parameter;    //插补参数、参数暂存

struct Coordinate_Class Virtual_AGV_Current_Coor_InWorld; //虚拟的AGV坐标用于路径跟随
extern struct Coordinate_Class PGV150_coor;   //PGV相机读到的坐标

extern int len_r;
int PGV_rx_flag;
int virtual_agv_coor_init_flag = 0;   //1为地标更新

extern canBus_t *CAN0;
MPI204A_t IMU;

struct Velocity_Class AGV_Current_Velocity_InAGV;         //AGV当前速度
//struct Velocity_Class AGV_Current_Velocity_By_Encoder;    //PLC上传由编码器得到速度

struct Coordinate_Class AGV_Current_Coor_InWorld;
Coordinate_Class_t AGV_Target_Coor_InWorld;               //AGV在世界坐标系下的期望坐标
Coordinate_Class_t Error_Coor_InAGV;                      //偏差坐标

extern float angle;
extern int odom_flag;
//extern Velocity_Class_t AGV_Target_Velocity_InAGV;
extern Velocity_Class_t Virtual_AGV_Current_Velocity_InAGV;
//extern int data_ok;
float angle_error;    //角度误差
float PID_error,PID_error_xy, error_last, error_next;      //PID误差输入
float PID_result = 0.0;

int can_succeed = 1;
int pgv_succeed = 1;
int pid_succeed = 1;




void paramter_define()
{

    wheelParamter.line_slowest_time = 0.35; //0.1
    wheelParamter.motor_max_rotationl_velocity_soft = 2800;
    wheelParamter.motor_min_rotationl_velocity_soft = 150;
    wheelParamter.WHEEL_DIAMETER = 150.0;
    wheelParamter.REDUCATION_RATIO = 15.0;
    wheelParamter.wheel_acceleration_time = 0.5454;//慢速2m/0.3636，快速3m/0.5454

    wheelParamter.wheel_max_line_velocity = wheelParamter.motor_max_rotationl_velocity_soft * wheelParamter.WHEEL_DIAMETER * M_PI / 60.0 / wheelParamter.REDUCATION_RATIO;
    wheelParamter.wheel_min_line_velocity = wheelParamter.motor_min_rotationl_velocity_soft * wheelParamter.WHEEL_DIAMETER * M_PI / 60.0 / wheelParamter.REDUCATION_RATIO;
    wheelParamter.wheel_acceleration_line_velocity = (wheelParamter.wheel_max_line_velocity - wheelParamter.wheel_min_line_velocity) / wheelParamter.wheel_acceleration_time;

    Interpolation_Parameter.max_velocity_abs = wheelParamter.wheel_max_line_velocity;
    Interpolation_Parameter.min_velocity_abs = wheelParamter.wheel_min_line_velocity;
    Interpolation_Parameter.accleration_abs = wheelParamter.wheel_acceleration_line_velocity;
    Interpolation_Parameter.slow_time_abs = wheelParamter.line_slowest_time;
}

void *myprocess2(void *t) //Prase_Sensor_Data.485pgv
{
	while (1)
	{
	//pgv_succeed = 0;
		//usleep(1000);

    PGV_Send_data();           //串口发送
    usleep(60);
    PGV_Rev();            //串口接受
    data_Ok = 0;

    if (len_r ==21){    //接收数据正常
    	  if (PGV_AnalyzeData() == 1)
    	        {
    	            PGV_Cal_Coor();
    	            data_Ok = 1;
    	        }
    	  if (!virtual_agv_coor_init_flag)
    	      {
    	    	Virtual_AGV_Current_Coor_InWorld = PGV150_coor;
    	    	  virtual_agv_coor_init_flag = 1;
    	      }
    	}
    //pgv_succeed = 1;
  //  return NULL;
	}
   }


void *myprocess3(void *t)  //can TMU
{
	while(1)
	{
		//usleep(1000);
	//can_succeed = 0;
    CanSendThread(CAN0);//初始化can0通道 (发送)
    CanRecvThread(CAN0);  //初始化can0通道(接收)
    IMU = MPI204A_Analyze_Data();  //解析imu角度及角速度

   //can_succeed = 1;
    //usleep(5000);
    //return NULL;
	}

}

/*
void can_fun()
{
		CanRecvThread(CAN0);  //初始化can0通道(接收)
		 IMU = MPI204A_Analyze_Data();  //解析imu角度及角速度
}
*/
/*****************************定位程序**********************************/
void Location_AGV()//Location_AGV
{
	//while(1)
	//{
		//usleep(100);

	static float x_last;
	static float y_last;

    Error_Coor_InAGV.x_coor = Destination_Coor_InWorld.x_coor - AGV_Current_Coor_InWorld.x_coor;
    Error_Coor_InAGV.y_coor = Destination_Coor_InWorld.y_coor - AGV_Current_Coor_InWorld.y_coor;
    if (AGV_Current_Coor_InWorld.angle_coor >= 0.0)
    {
        Error_Coor_InAGV.angle_coor = Destination_Coor_InWorld.angle_coor - AGV_Current_Coor_InWorld.angle_coor;
    }
    else if (AGV_Current_Coor_InWorld.angle_coor < 0.0 && AGV_Current_Coor_InWorld.angle_coor >= -100.0)
    {
        Error_Coor_InAGV.angle_coor = Destination_Coor_InWorld.angle_coor - AGV_Current_Coor_InWorld.angle_coor;
    }
    else if (AGV_Current_Coor_InWorld.angle_coor < -100.0 && AGV_Current_Coor_InWorld.angle_coor >= -180)
    {
        Error_Coor_InAGV.angle_coor = Destination_Coor_InWorld.angle_coor - abs(AGV_Current_Coor_InWorld.angle_coor);
    }
    Error_Coor_InAGV.angle_coor = abs(Error_Coor_InAGV.angle_coor);

	//AGV_Current_Velocity_InAGV.angular_velocity_angle = 0.84 * IMU.angleRate + 0.16 * AGV_Current_Velocity_By_Encoder.angular_velocity_angle;  //角速度加权融合
    AGV_Current_Velocity_InAGV.angular_velocity_angle = IMU.angleRate;
	//AGV_Current_Velocity_InAGV.velocity_x = AGV_Current_Velocity_By_Encoder.velocity_x;  //编码器返回速度有误
	//AGV_Current_Velocity_InAGV.velocity_y = 0;

  //  printf("data_ok=%d\n",data_Ok);
   // printf("odom_flag=%d\n",odom_flag);

	if (data_Ok == 1)     //相机读取到数据
	{
        //data_Ok = 0;
        angle = 0.0;

        Coor_delta.y_coor = 0;
        Coor_delta.x_coor = 0;
        Coor_delta.angle_coor = 0;
        Coor_delta.angle_rad = 0 ;

		AGV_Current_Coor_InWorld = PGV150_coor;   //更新全局坐标
	}

	else if ( data_Ok==0 )
	{
		//odom_flag = 0;
		AGV_Current_Coor_InWorld = Odom_Calib(Virtual_AGV_Current_Velocity_InAGV.velocity_x, AGV_Current_Velocity_InAGV.angular_velocity_angle);    //无二维码部分进行里程推算
	}
	/*
	if ( abs(AGV_Current_Coor_InWorld.x_coor - x_last ) > 100 || abs( y_last - AGV_Current_Coor_InWorld.y_coor) > 100){
		AGV_Current_Coor_InWorld.x_coor = x_last;
		AGV_Current_Coor_InWorld.y_coor = y_last;
	}

	 x_last = AGV_Current_Coor_InWorld.x_coor;
	 y_last = AGV_Current_Coor_InWorld.y_coor;
	 */
	//return NULL;
	//}
}


/*****************************PID调节*********************************/
void *myprocess1(void *t) //PID_fun
{
	while(1)
	{
	//pid_succeed = 0;
		usleep(3);
            angle_error = Destination_Coor_InWorld.angle_coor - AGV_Current_Coor_InWorld.angle_coor;
            //printf("angle_error=%f\n",angle_error);

if( (abs(angle_error) < 0.47 && (abs(Error_Coor_InAGV.x_coor) < 2 ||abs(Error_Coor_InAGV.y_coor) < 2))||( abs(Virtual_AGV_Current_Velocity_InAGV.velocity_x)<80 ) ){
	 PID_error = 0;
	 PID_error_xy = 0;
	// printf("安全区间\n");
    }

else
{
        //float HT = T*abs(AGV_Target_Velocity_InAGV.velocity_x)*0.001;
       // float HT = T;
	if ( Motionstyle == ACTION_MODE_GOAHEAD )
		{
        if (Destination_Coor_InWorld.angle_coor == 0.0){
        	//if( abs(angle_error) <= 1.5 && abs(Error_Coor_InAGV.y_coor) >4.5 ){//0.5 3 3
        	if( 0&&  abs(Error_Coor_InAGV.y_coor) >5.0  ){
        		 PID_error = Error_Coor_InAGV.y_coor*T;
        		// printf("危险区间\n");
        	}
        	else
        		PID_error = Error_Coor_InAGV.y_coor * 0.45 + angle_error * 0.55;
        		PID_error_xy =  Error_Coor_InAGV.y_coor;
        }

        else if (Destination_Coor_InWorld.angle_coor == 90.0){
        	//if(   abs(angle_error) <= 1.5 && abs(Error_Coor_InAGV.x_coor) >4.5 ){
        	if(  0&&  abs(Error_Coor_InAGV.x_coor) > 5.0 ){
        		 PID_error = -Error_Coor_InAGV.x_coor*T;
        		 //printf("危险区间 \n");
        	}
        	else
        		PID_error = -Error_Coor_InAGV.x_coor * 0.45 + angle_error * 0.55;
        		PID_error_xy = -Error_Coor_InAGV.x_coor;
        }
     }


 if ( Motionstyle == ACTION_MODE_GOBACK)
 {
       if (Destination_Coor_InWorld.angle_coor == 0.0){
    	  // if(    abs(angle_error) <= 1.5 && abs(Error_Coor_InAGV.y_coor) >4.5 ){
    	   if(  0&&    abs(Error_Coor_InAGV.y_coor) > 5.0  ){
    		   PID_error = -Error_Coor_InAGV.y_coor*T;
    		   //printf("危险区间 \n");
    	   }
    	   else
    		   PID_error = -Error_Coor_InAGV.y_coor * 0.45 + angle_error * 0.55;
    	   	   PID_error_xy = -Error_Coor_InAGV.y_coor;
    	   	   //printf("PID_error_xy=%f\n",PID_error_xy);
    	   	   //printf("PID_error=%f\n",PID_error);
       }

       else if (Destination_Coor_InWorld.angle_coor == 90.0){
    	   //if(    abs(angle_error) <= 1.5 && abs(Error_Coor_InAGV.x_coor) >4.5 ){
    	   if(  0&&  abs(Error_Coor_InAGV.x_coor) > 5.0  ){
    		   PID_error = Error_Coor_InAGV.x_coor*T;
    		   //printf("危险区间 \n");
    	   }
    	   else
    		   PID_error = Error_Coor_InAGV.x_coor * 0.45 + angle_error * 0.55;
    	   	   PID_error_xy = Error_Coor_InAGV.x_coor;
       }
    }
}

	if(abs(PID_error_xy) > abs(error_last))   //产生误差
	{
		printf("in_put\n");
 	// if (abs(Virtual_AGV_Current_Velocity_InAGV.velocity_x)> 1200)
        PID_result = HI * (PID_error_xy+angle_error) + HP * PID_error + HD * (PID_error - 2 * error_next + error_last);
 	// else
 		//PID_result = KI * (PID_error - error_next) + KP * PID_error + KD * (PID_error - 2 * error_next + error_last);
	}

	if(abs(PID_error_xy) < abs(error_last))  //回调
	{
		printf("bake\n");
		 PID_result = -HI * PID_error_xy + HP * PID_error + HD * (PID_error - 2 * error_next + error_last);
	}
	/*if(abs(PID_error_xy) == abs(error_last))  //回调
	{
		//printf("safe\n");
		 PID_result =  HP * PID_error + HD * (PID_error - 2 * error_next + error_last);
	}*/

        if (PID_result < -60)
        {
            PID_result = -60.0;
        }
        else if (PID_result > 60)
        {
            PID_result = 60.0;
        }
        static int i;
        i++;
        if(i>50){
        	i = 0;
        error_last = error_next;
        }
        error_next = PID_error_xy;
        //usleep(3);
       // printf("PID_result = %f\n", PID_result);
       // pid_succeed = 1;
       // usleep(5000);
       // return NULL;
	}
}




void get_can()
{
    int rc;
    long t;
    pthread_t id_can;
    rc = pthread_create(&id_can, NULL, myprocess3, (void *)t);
    if (rc)
    {
        printf("Can pthread create failed!\n");
        return;
    }
}

void get_pid()
{
    int rc;
    long t;
    pthread_t id_pid;
    rc = pthread_create(&id_pid, NULL, myprocess1, (void *)t);
    if (rc)
    {
        printf("PID pthread create failed!\n");
        return;
    }
}

void get_pgv()
{
    int rc;
    long t;
    pthread_t id_pgv;
    rc = pthread_create(&id_pgv, NULL, myprocess2, (void *)t);
    if (rc)
    {
        printf("Pgv pthread create failed!\n");
        return;
    }
}
/*
void get_location()
{
    int rc;
    long t;
    pthread_t id_location;
    rc = pthread_create(&id_location, NULL, myprocess4, (void *)t);
    if (rc)
    {
        printf("Pgv pthread create failed!\n");
        return;
    }
}
*/

