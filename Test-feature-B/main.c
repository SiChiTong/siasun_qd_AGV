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
#include <Thread_Pool.h>
#include <sys/time.h>



Coordinate_Class_t Destination_Coor;    //目标坐标





extern Interpolation_State_Enum_m Interpolation_State;

extern int command_x;
extern int command_y;
extern int command_angle;
extern int stop;
extern int stop_flage;

char buffer[8];    //发送PLC数组

float VL_1200;    //发送给PLC左轮速度
float VR_1200;    //发送给PLC右轮速度


Coordinate_Class_t Destination_Coor_InWorld;     //目标位置



int Add_Command_Line, Add_Command_Rotate;
int odom_flag;

//***临时printf****//
extern Velocity_Class_t Virtual_AGV_Current_Velocity_InAGV;
extern float angle_error;
extern Coordinate_Class_t Error_Coor_InAGV;                      //偏差坐标
int i =0;
extern float acc_distance;    //加速段距离(mm)
extern float const_distance ;  //匀速段距离(mm)
extern float dec_distance;    //减速段距离(mm)
extern float slowly_distance ; //慢速段距离(mm)

extern canBus_t *CAN0; //can接收
//由触摸屏得到目标坐标
void Gcode_G0()
{


	static int x;
	static int y;

	static int flage =0;
	if(flage != 1){
		flage =1;
		stop_flage =800;
	}


	 if(abs(AGV_Current_Coor_InWorld.y_coor-900)<7 && abs(AGV_Current_Coor_InWorld.x_coor-700)<7 && stop_flage>10){
	   x = 700;//700/0
	   y = 2000;//3300/500
	   stop_flage=0;
	}

	else if(abs(AGV_Current_Coor_InWorld.y_coor-2000)<7 && abs(AGV_Current_Coor_InWorld.x_coor-700)<7 &&  stop_flage>10 ){
		x = 0;
		y = 2000;
		stop_flage=0;
	}

	else if(abs(AGV_Current_Coor_InWorld.x_coor) < 7 && abs(AGV_Current_Coor_InWorld.y_coor - 2000) < 7&&  stop_flage>10 ){
		x = 0;
		y = 900;
		stop_flage=0;
	}
	else if(abs(AGV_Current_Coor_InWorld.x_coor ) < 7 && abs(AGV_Current_Coor_InWorld.y_coor - 900) <7 &&  stop_flage>10 ){
		x = 700;
		y = 900;
		stop_flage=0;
	}
/*
		 if(abs(AGV_Current_Coor_InWorld.x_coor) < 7 && abs(AGV_Current_Coor_InWorld.y_coor - 3300) < 7&&  stop_flage>15 ){
		x = 0;
		y = 700;
		stop_flage=0;
	}
	else if(abs(AGV_Current_Coor_InWorld.x_coor ) < 7 && abs(AGV_Current_Coor_InWorld.y_coor - 700) <7 &&  stop_flage>15 ){
		x = 0;
		y = 3300;
		stop_flage=0;
	}
*/




	//Destination_Coor_InWorld.x_coor = command_x * 1.0;
	Destination_Coor_InWorld.x_coor = x;
    //Destination_Coor_InWorld.y_coor = command_y * 1.0;
	Destination_Coor_InWorld.y_coor = y;
   // Destination_Coor.angle_coor = Destination_Coor_InWorld.angle_coor; //屏幕获取目标角度
    //printf("Destination： %f, %f,  %f\n", Destination_Coor_InWorld.x_coor, Destination_Coor_InWorld.y_coor, Destination_Coor_InWorld.angle_coor);//目标点
   // printf("C_X = %d, C_Y = %d, C_A = %d\n", command_x, command_y, command_angle);

    // printf("目标点位：%d,%d\n",x,y);
    //Coordinate_Class_t Destination_Coor_temp;
    /*
    if ((Add_Command_Line == 1) || (Add_Command_Rotate == 1))
    {
        //Destination_Coor_temp = Destination_Coor;
       // Destination_Coor_InWorld = Destination_Coor;
        Interpolation_State = No_Interpolation;
    }
*/
}

/***************定时器**********************/
void test_func()
{
     odom_flag = 1;
}

void init_sigaction()
{
    struct sigaction act;

    act.sa_handler = test_func; //设置处理信号的函数
    act.sa_flags  = 0;

    sigemptyset(&act.sa_mask);
    sigaction(SIGPROF, &act, NULL);//时间到发送SIGROF信号
}

void init_time()
{
    struct itimerval val;

    val.it_value.tv_sec = 0; //10u秒后启用定时器
    val.it_value.tv_usec = 5;

    val.it_interval = val.it_value; //定时器间隔为10us

    setitimer(ITIMER_PROF, &val, NULL);
}


//系统初始化

void init_System()
{
    paramter_define();  //参数定义
    MPI204A_init();      //IMU初始化 CAN
    PGV_init();          //相机初始化 485
    get_PLC_init();      //1200数据接收初始化
  //  init_sigaction();//定时器
   // init_time();			//定时器
   // pool_init (4);  //线程池初始化，其中共4个活动线程

    //get_can();
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
	   // printf("VL_1200=%d, VR_1200=%d\n", Vl.fdata, Vr.fdata);
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
        Gcode_G0();  //由触摸屏得到目标坐标

        myprocess0(); //tcp
        myprocess1(); //pid
        myprocess2();  //485
        myprocess3(); //can
       // Pthread_Analy();//线程处理函数
        can_fun();
        Location_AGV();         //AGV定位
        DirectionDetermination();  //判断车体的运行方向
        AGV_RUN();

        /******************临时打印*************************/
        i++;
        if (i>1){
        	i=0;

            printf("data_ok=%d\n",data_Ok);
            printf("odom_flag=%d\n",odom_flag);
		if(data_Ok==1)
		printf("CURRENT_X =%f,CURRENT_Y=%f,CURRENT_A=%f\n",AGV_Current_Coor_InWorld.x_coor,AGV_Current_Coor_InWorld.y_coor,AGV_Current_Coor_InWorld.angle_coor);
		if(data_Ok==0)
			printf("currentr_x =%f,current_y=%f,current_a=%f\n",AGV_Current_Coor_InWorld.x_coor,AGV_Current_Coor_InWorld.y_coor,AGV_Current_Coor_InWorld.angle_coor);

		printf("stop=%d\n",stop);
		printf("Destination： %f, %f,  %f\n", Destination_Coor_InWorld.x_coor, Destination_Coor_InWorld.y_coor, Destination_Coor_InWorld.angle_coor);//目标点

		printf("State=%d\n",State);
		printf("current_distance=%f\n",current_distance);
		printf("acc_distance=%f,const_distance=%f,dec_distance=%f\n,slowly_distance=%f\n",acc_distance,const_distance,dec_distance,slowly_distance);

		if( (abs(angle_error) < 0.45 && (abs(Error_Coor_InAGV.x_coor) < 1.5 ||abs(Error_Coor_InAGV.y_coor) < 1.5))||( abs(Virtual_AGV_Current_Velocity_InAGV.velocity_x)<85 ) ){
		 printf("安全区间\n");
		    }
		else
		{
			if (Distance_Symbols == 1){
		        if (Destination_Coor_InWorld.angle_coor == 0.0){
		        	if( abs(angle_error) <= 1.5 && abs(Error_Coor_InAGV.y_coor) > 5){//0.5 3 3
		        		printf("危险区间\n");
		        	} }
		        else if (Destination_Coor_InWorld.angle_coor == 90.0){
		        	if( abs(Error_Coor_InAGV.x_coor) > 5){
		        		printf("危险区间 \n");
		        	} }
		     }

			if (Distance_Symbols == -1) {
		       if (Destination_Coor_InWorld.angle_coor == 0.0){
		    	   if(  abs(Error_Coor_InAGV.y_coor) > 5){
		    	  printf("危险区间 \n");
		    	   } }
		       else if (Destination_Coor_InWorld.angle_coor == 90.0){
		    	   if( abs(Error_Coor_InAGV.x_coor) >5){
		    		   printf("危险区间 \n");
		    	   } }
		     }
		}
       }

	}
}
