/*
 * MPI204A.c
 *
 *  Created on: Jun 24, 2019
 *      Author: siasunhebo
 */


#include <MPI204A.h>
#include <stdio.h>

canBus_t *CAN0;
//float MPI204A_Angle = 0.0;        //angle output
//float MPI204A_AngleRate = 0.0;    //angle's rate output

void IO_PowerOn () //io口使能
{
	CAN0->sendBuffer.can_id = 0x607;
	CAN0->sendBuffer.can_dlc = 8;
	CAN0->sendBuffer.data[0] = 0x23;
	CAN0->sendBuffer.data[1] = 0x08;
	CAN0->sendBuffer.data[2] = 0x62;
	CAN0->sendBuffer.data[3] = 0x00;
	CAN0->sendBuffer.data[4] = 0x00;
	CAN0->sendBuffer.data[5] = 0xFF;
	CAN0->sendBuffer.data[6] = 0XFF;
	CAN0->sendBuffer.data[7] = 0X00;

	CAN0->sendBuffer.can_id = 0x608;
	CAN0->sendBuffer.can_dlc = 8;
	CAN0->sendBuffer.data[0] = 0x23;
	CAN0->sendBuffer.data[1] = 0x08;
	CAN0->sendBuffer.data[2] = 0x62;
	CAN0->sendBuffer.data[3] = 0x00;
	CAN0->sendBuffer.data[4] = 0x00;
	CAN0->sendBuffer.data[5] = 0xFF;
	CAN0->sendBuffer.data[6] = 0XFF;
	CAN0->sendBuffer.data[7] = 0X00;


}


void MPI204A_init()
{
	CAN0 = CanBusInit("can0");       //init can
     IO_PowerOn();   //io口使能
	//pthread_t canRecvCan0;              //can0接收线程句柄
	//pthread_create(&canRecvCan0, NULL, (void *)CanRecvThread, CAN0);
}

MPI204A_t MPI204A_Analyze_Data()
{
    float angle, angleRate;
    MPI204A_t MPI204a;
	angle     = (float)((short)(CAN0->recvBuffer.data[2] << 8) | CAN0->recvBuffer.data[3]) / 10.0;
	angleRate = (float)((short)(CAN0->recvBuffer.data[4] << 8) | CAN0->recvBuffer.data[5]) / 10.0;
	angleRate = -angleRate;
	//printf("MPI204A_Angle=%f, MPI204A_AngleRate=%f\n", MPI204A_Angle, MPI204A_AngleRate);
    MPI204a.angle = angle;
    MPI204a.angleRate = angleRate;
	//printf("MPI204A_Angle=%f\n", angle);
	//printf("MPI204A_AngleRate=%f\n", angleRate);
    return MPI204a;
}
