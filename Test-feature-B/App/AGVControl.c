//
// Created by maxaio on 8/26/19.
//

#include<stdio.h>
#include <stdint-gcc.h>
#include <include/user_function.h>
#include <common.h>
#include <main.h>
#include "AGV/agv.h"
#include "motion_ctrl.h"
#include "Tools/Tools.h"
#include "AGVControl.h"
#include <PGV150/PGV150.h>
#include <stdlib.h>
#include<Movement.h>
#include<agv.h>
#include<math.h>
#include<time.h>




extern Velocity_Class_t Virtual_AGV_Current_Velocity_InAGV;
extern Velocity_Class_t Target_Velocity_InAGV;     //目标速度
extern struct Coordinate_Class AGV_Current_Coor_InWorld;
extern Interpolation_State_Enum_m Interpolation_State;

extern Coordinate_Class_t Error_Coor_InAGV;                      //偏差坐标

static uint16_t nEvent = NO_EVENT;    //当前所响应的事件


static uint16_t OutOfTrack = 0x00;    //脱轨，默认为不脱轨
static uint16_t GSMissing  = 0x00;    //导航信息丢失

uint16_t twoDMissingCnt;
extern float wControl;   //旋转角速度

uint16_t nAgvWorkMode = AGV_MODE_STANDBY;

extern int Add_Command_Rotate;

extern float VL_1200;    //发送给PLC左轮速度
extern float VR_1200;    //发送给PLC右轮速度
int judge_axis=0;
int run_flage = 600;

/*************************直线判断********************************/
void goStraight()
{
	    int a=1;
		float direction;
		static float direction_x;
		static float direction_y;

		if( abs( Error_Coor_InAGV.x_coor)>30 ||abs(Error_Coor_InAGV.y_coor)>30 ) {
			 direction_x = Destination_Coor_InWorld.x_coor - AGV_Current_Coor_InWorld.x_coor;
			 direction_y= Destination_Coor_InWorld.y_coor - AGV_Current_Coor_InWorld.y_coor;
			}

		a = (( PGV150_coor.angle_coor > (-45) && PGV150_coor.angle_coor < 135) ? 1:-1);
		direction = ( abs( direction_x) - abs( direction_y ) > 0 ? direction_x : direction_y );
		Distance_Symbols = (direction > 0 ? 1 : -1) * a ;

		if (Distance_Symbols == 1)
		    Motionstyle = ACTION_MODE_GOAHEAD;

		if (Distance_Symbols == -1)
			Motionstyle = ACTION_MODE_GOBACK;
	}


/****************************转向判断*********************************/
 swerve()
{
//	printf("angle_deviation=%f\n",angle_deviation);
      if (  Destination_Coor_InWorld.angle_coor == 0 )			//如果目标角度为0°
      {
          if ( angle_deviation > 0 && angle_deviation < 180)
          Motionstyle = MOTIONSTATE_TRUNLEFT;				//左转
          else
          Motionstyle = MOTIONSTATE_TRUNRIGHT;					//右转
      }
      if ( Destination_Coor_InWorld.angle_coor == 90 )			//如果目标角度为90°
      {
          if ( angle_deviation > 90 && angle_deviation < 270)
              Motionstyle = MOTIONSTATE_TRUNRIGHT;			//右转
          else
              Motionstyle = MOTIONSTATE_TRUNLEFT;				//左转
      }
/*
      if ( Destination_Coor_InWorld.angle_coor == 180 )			//如果目标角度为180°
      {
          if ( angle_deviation > 0 && angle_deviation < 180 )
          	  Motionstyle = MOTIONSTATE_TRUNRIGHT;			//右转
          else
          	Motionstyle = MOTIONSTATE_TRUNLEFT;				//左转
      }
      if ( Destination_Coor_InWorld.angle_coor == -90 )			//如果目标角度为270°
      {
          if ( angle_deviation > 90 && angle_deviation < 270 )
              Motionstyle = MOTIONSTATE_TRUNLEFT;				//左转
          else
              Motionstyle = MOTIONSTATE_TRUNRIGHT;			//右转
      }
*/
  }

 /*************************事件复位程序********************/
void track_resrt() //复位
{
	switch (nEvent)
	{

	 case OBSTACLE_STOP_EVENT:  //避障解除后地标丢失
	 {
		   VL_1200 =  300.0*Distance_Symbols;
		   VR_1200 =  300.0*Distance_Symbols;
		   break;
	 }

	 case OUT_OF_TRACK_EVENT:   //脱轨复位
	 {
		 if (judge_axis == 1)
		{
			 if (AGV_Current_Coor_InWorld.y_coor<0) {
			 	 VL_1200 =  -130.0*Distance_Symbols;
			 	 VR_1200 =  -140.0*Distance_Symbols;
			 }
			 else {
				 VL_1200 =  -140.0*Distance_Symbols;
				 VR_1200 =  -130.0*Distance_Symbols;
			 }
		}
		 else if (judge_axis == 2)
		 {
			if (AGV_Current_Coor_InWorld.x_coor<0) {
				VL_1200 =  -130.0*Distance_Symbols;
				 VR_1200 =  -140.0*Distance_Symbols;
		   	}
		   else {
			   VL_1200 =  -140.0*Distance_Symbols;
			   VR_1200 =  -130.0*Distance_Symbols;
		   }
		 }
		 break;
	 }
	}

}


/***************************判断车体的运行方向************************************/
void DirectionDetermination()
{

	static float direction_x = 0;
	static float direction_y = 0;


	if (abs( Error_Coor_InAGV.x_coor)>80 ||abs(Error_Coor_InAGV.y_coor)>80){
	 direction_x = Destination_Coor_InWorld.x_coor - AGV_Current_Coor_InWorld.x_coor;
	 direction_y= Destination_Coor_InWorld.y_coor - AGV_Current_Coor_InWorld.y_coor;
	}


		judge_axis = ( abs( direction_x) - abs( direction_y ) > 0 ? 1:2 );

		if (judge_axis == 1)  //X方向
		{
			Destination_Coor_InWorld.angle_coor = 0.0;

			if (abs(Error_Coor_InAGV.angle_coor) <= 10.0 && run_flage >10)
			goStraight();

			else if  (abs(Error_Coor_InAGV.angle_coor) >10 && abs(direction_x)>10) {
			run_flage = 0;
			swerve();
			}
		}

		else if (judge_axis == 2)  //Y方向
		{
			Destination_Coor_InWorld.angle_coor = 90.0;

			if (abs(Error_Coor_InAGV.angle_coor) <=10.0 && run_flage >10 )
			goStraight();

			else if  (abs(Error_Coor_InAGV.angle_coor) >10 && abs(direction_y)>10) {
				run_flage = 0;
				swerve();
			}
		}

}



void AGV_RUN()
{
	   if (0)  //避障停车标志位
	   {
	    	nEvent = OBSTACLE_STOP_EVENT; //避障停车事件
	    	nAgvWorkMode = AGV_MODE_SUSPENDED;   //进入挂起
	    }
	   else if(Add_Command_Rotate == 1 ) //避障减速标志位
	   {
		   nEvent =  OBSTACLE_LOW_EVENT; //避障减速事件
		   nAgvWorkMode = AGV_MODE_SUSPENDED;   //进入挂起
	   }
	//printf("nAgvWorkMode=%d\n",nAgvWorkMode);
    switch (nAgvWorkMode)
    {
        case AGV_MODE_STANDBY:
            AGV_StandBy();
            break;
        case AGV_MODE_RUNNING:
            AGV_Running();
            break;
        case AGV_MODE_SUSPENDED:
            AGV_Suspended();
            break;
        case AGV_MODE_OP:
            AGV_Op();
            break;
        default:
            break;
    }

}

//待机操作函数
void AGV_StandBy()
{
    nAgvWorkMode = AGV_MODE_RUNNING;

    //ToDo: 各状态及变量清零
    //系统初始化
}

//手动操作函数
void AGV_Op()
{

}


/**********************挂起函数******************************/
void AGV_Suspended()
{
    //ToDo: 关掉驱动器使能，速度清零
    printf("nEvent=%d\n",nEvent);
    switch (nEvent)
    {
    case OBSTACLE_STOP_EVENT:    //如果触发避障停车
    {
    	 if (Add_Command_Rotate == 1)  //急停标志位
    	 {
    		 VL_1200 = 0.0;
    		 VR_1200 = 0.0;
    	 }
    	 else if (Add_Command_Rotate == 0)  //急停解除
    	 {
             if (data_Ok ==1 ) {
             	nEvent = 0;
               Virtual_AGV_Current_Coor_InWorld = PGV150_coor;
               State = No_Interpolation;
               nAgvWorkMode = AGV_MODE_RUNNING;
               }
             track_resrt(); //脱轨复位
    	 }
/*
        if (GSMissing == 0x00)
        {
            nAgvWorkMode = AGV_MODE_RUNNING;
            //ToDo: 速度等参数清零
        }
        */
        break;
    }

    	case OBSTACLE_LOW_EVENT:    //避障减速
    	{
    		Virtual_AGV_Current_Velocity_InAGV.velocity_x = 280.0* Distance_Symbols; //减速
    		usleep(30);

    		if (data_Ok ==1 )  {
            	VL_1200 = 0.0;
            	VR_1200 = 0.0;
            	if (Add_Command_Rotate == 0)
            	{
                 	nEvent = 0;
                   Virtual_AGV_Current_Coor_InWorld = PGV150_coor;
                   State = No_Interpolation;
                   nAgvWorkMode = AGV_MODE_RUNNING;
            	}
    		}
    		break;
    	}

        case OUT_OF_TRACK_EVENT:      //如果脱轨，检查是否回归坐标点
        {
        	printf("Derailment\n");
        	//ToDo: 速度等参数清零
        	VL_1200 = 0.0;
        	VR_1200 = 0.0;
        	//track_resrt(); //脱轨复位

     //     printf("data_ok=%d\n",data_ok);
            if (data_Ok ==1 ) {
            	nEvent = 0;
             twoDMissingCnt =0;
              OutOfTrack = 0x00;
              Virtual_AGV_Current_Coor_InWorld = PGV150_coor;
              State = No_Interpolation;
              nAgvWorkMode = AGV_MODE_RUNNING;
              }
            break;
        }


        case CONTROL_RESUMABLE_STOP_EVENT:     //如果网络中断
        {
            if (ethCommBreakFlag == 0x00)
            {
                nAgvWorkMode = AGV_MODE_RUNNING;
                //ToDo: 速度等参数清零
            }
            break;
        }

        //驱动器
        case SERVO_INVALID_EVENT:            //如果伺服失效
        {
            //ToDo: 判断伺服是否恢复 速度等参数清零
            break;
        }
        default:
            break;
    }
}


/****************************************执行函数**********************************/
void AGV_Running()
{

    if (NO_EVENT == nEvent)
    {
        if (ethCommBreakFlag == 0x01)    //通讯中断
        {
            nEvent = CONTROL_RESUMABLE_STOP_EVENT;     //可恢复停车标志位
        }
        else if (OutOfTrack == 0x01)     //检查导航脱轨事件
        {
            nEvent = OUT_OF_TRACK_EVENT;
        }
        else if (GSMissing == 0x01)      //检查地标点是否丢失//急停
        {
            nEvent = GS_COMM_BREAK_EVENT;
        }

        else
        {
   //   	printf("AGV_Runing\n");
        	//intf("Motionstyle=%d\n",Motionstyle);
/*********************************************前进****************************************************/
            if ( Motionstyle == ACTION_MODE_GOAHEAD )
            {
            	//printf("Go_head\n");
            	Movement_Control();
            	Run_Movement_Class();

                if (Target_Velocity_InAGV.velocity_x > 300)
                {
             ++ twoDMissingCnt;
    //              printf("twoDMissingCnt=%d\n",twoDMissingCnt);

                    if (data_Ok ==1 ) {
                    	twoDMissingCnt =0;
                    	OutOfTrack = 0x00;
                    }

                    if (twoDMissingCnt > 750)
                    {
                        OutOfTrack = 0x01;
                        nEvent = OUT_OF_TRACK_EVENT;
                        nAgvWorkMode = AGV_MODE_SUSPENDED ;
                    }

                }
            }
 /*********************************************后退****************************************************/
            if ( Motionstyle == ACTION_MODE_GOBACK)
            {
            //	printf("Go_back\n");
            	Movement_Control();
            	Run_Movement_Class();

                if (abs(Target_Velocity_InAGV.velocity_x) > 300)
                {
                	++ twoDMissingCnt;
   //           	  printf("twoDMissingCnt=%d\n",twoDMissingCnt);
                      if (data_Ok ==1 ) {
                      	twoDMissingCnt =0;
                      	OutOfTrack = 0x00;
                      }

                    if (twoDMissingCnt > 750)    //检查导航脱轨事件
                    {
                        OutOfTrack = 0x01;
                        nEvent = OUT_OF_TRACK_EVENT;
                        nAgvWorkMode = AGV_MODE_SUSPENDED ;
                     }
                }
            }
/********************************************左转******************************************************/
            else if (Motionstyle == ACTION_MODE_TRUNLEFT)
            {
            	static float vx =0.0 ;
            	//printf("Turn_left\n");
              if (abs(Error_Coor_InAGV.angle_coor) <= 2.0 )
              {
            	  VL_1200 = 0.0;
            	  VR_1200 = 0.0;
            	  vx =0;
            	  ++run_flage;
              }
              else if( abs(Destination_Coor_InWorld.angle_coor - AGV_Current_Coor_InWorld.angle_coor) >65  )
                {
            	  vx=vx+100;
            	  if(vx>300)
            	        vx=300;
                  }
              else if ( abs(Destination_Coor_InWorld.angle_coor - AGV_Current_Coor_InWorld.angle_coor) < 65)
              {
            	  vx = vx -180;
            	  if (vx<53)
            	        vx=53;
              }

        	  VL_1200 = vx;
        	  VR_1200 = -vx;

      /*          if (wControl > 0.4)
                {
               //    ++ twoDMissingCnt;
                    if (twoDMissingCnt > 100)
                    {
                        OutOfTrack = 0x01;
                        nEvent = 2;
                        nAgvWorkMode = AGV_MODE_SUSPENDED ;
                    }
                    //ToDo: 旋转速度插补 Rotate_Process_Movement_Command()
                    //ToDo: 旋转运动控制 Rotate_Movemnet_control_left()
                }
*/

            }
/********************************************右转******************************************************/
            else if (Motionstyle == ACTION_MODE_TRUNRIGHT)
            {
            	static float vx =0.0 ;
            	//printf("Turn_right\n");
                if (abs(Error_Coor_InAGV.angle_coor) <= 2.0 )
                {
              	  VL_1200 = 0.0;
              	  VR_1200 = 0.0;
              	  vx =0;
              	  ++run_flage;
                }
                else if( abs(Destination_Coor_InWorld.angle_coor - AGV_Current_Coor_InWorld.angle_coor) >65  )
                  {
              	  vx=vx+100;
              	  if(vx>300)
              	        vx=300;
                    }
                else if ( abs(Destination_Coor_InWorld.angle_coor - AGV_Current_Coor_InWorld.angle_coor) < 65)
                {
              	  vx=vx-180;
              	  if (vx<53)
              	        vx=53;
                }

          	  VL_1200 = -vx;
          	  VR_1200 = vx;

 /*               if (wControl > 0.4)
                {
              /     ++ twoDMissingCnt;
                    if (twoDMissingCnt > 100)
                    {
                        OutOfTrack = 0x01;
                        nEvent = 2;
                        nAgvWorkMode = AGV_MODE_SUSPENDED ;
                    }
                    //ToDo: 旋转速度插补 Rotate_Process_Movement_Command()
                    //ToDo: 旋转运动控制 Rotate_Movemnet_control_Right()
                }
*/
            }
        }
      //  nAgvWorkMode = AGV_MODE_SUSPENDED ;		//插补结束，进入挂起模式
    }
    else if (nEvent != NO_EVENT)
    {
        if ((nEvent == GS_COMM_BREAK_EVENT) && ((Motionstyle == ACTION_MODE_TRUNRIGHT) || (Motionstyle == ACTION_MODE_TRUNLEFT)))
        {

            Motionstyle = ACTION_MODE_STOP;
        }
        //恢复直行
        if ((Motionstyle == ACTION_MODE_GOAHEAD) || (Motionstyle == ACTION_MODE_GOBACK))
        {
            UpdateAgvHeaderDirToNow();
            Run_Movement_Class();
            if (Target_Velocity_InAGV.velocity_x > 1.0)
            {
                ++ twoDMissingCnt;
                if (twoDMissingCnt > 100)
                {
                    OutOfTrack = 0x01;

                }
              Run_Movement_Class();
                //ToDo: 前进后退速度相反
                //ToDo: 运动控制 Movement_control()
            }
        }

    }



}
