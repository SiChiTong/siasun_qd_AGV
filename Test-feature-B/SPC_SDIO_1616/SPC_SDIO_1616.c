/*
 * SPC_SDIO_1616.c
 *
 *  Created on: 2019-8-26
 *      Author: siasunggm
 */

#include <SPC_SDIO_1616.h>
#include <stdio.h>
#include <stdint.h>

extern canBus_t *CAN0;
 SPC_SDIO_1616_Data SPC_SDIO_1616a;
 SPC_SDIO_1616_Data SPC_SDIO_1616c;
 AGV_SDIO_Data SPC_SDIO_1616_Outa;
 AGV_SDIO_Data SPC_SDIO_1616_Outc;
 AGV_SDIO_Data_Out AGV_SDIO_Data_Outa;
 AGV_SDIO_Data_Out AGV_SDIO_Data_Outc;
 Sound_Data Sound_Control_Data;
 Lamplight_Data Lamplight_Control_Data;

void SPC_SDIO_1616a_Analyze_Data()  //接收的数据
{
    int  import0, import1,import2,import3;
	import0 = (int)((short)(CAN0->recvBuffer.data[0]));
	import1 = (int)((short)(CAN0->recvBuffer.data[1]));
	import2 = (int)((short)(CAN0->recvBuffer.data[2]));
	import3 = (int)((short)(CAN0->recvBuffer.data[3]));

    SPC_SDIO_1616a.point[0] = import0&0x01;//启动按钮x00
    SPC_SDIO_1616a.point[1] = import0&0x02;//ALLOK检测x01
    SPC_SDIO_1616a.point[2] = import0&0x04;//自动x02
    SPC_SDIO_1616a.point[3] = import0&0x08;//手动x03
    SPC_SDIO_1616a.point[4] = import0&0x10;//机器人操作权输出x04
    SPC_SDIO_1616a.point[5] = import0&0x20;//错误发生中x05
    SPC_SDIO_1616a.point[6] = import0&0x40;//伺服on x06
    SPC_SDIO_1616a.point[7] = import0&0x80;//运行中 x07

    SPC_SDIO_1616a.point[12] = import1&0x10;//安全继电器检测  拍下急停时置1 控制器进入待机状态

    SPC_SDIO_1616a.point[15] = import1&0x80;//地标传感器S级信号 暂用

    SPC_SDIO_1616a.point[16] = import2&0x01;//地标传感器N级信号

    SPC_SDIO_1616a.point[24] = import1&0x01;///机器人急停
    SPC_SDIO_1616a.point[25] = import3&0x02;//机器人故障
    SPC_SDIO_1616a.point[26] = import3&0x04;//机器人运行
    SPC_SDIO_1616a.point[27] = import3&0x08;//机器人停止
    SPC_SDIO_1616a.point[28] = import3&0x10;//自定义接口
    SPC_SDIO_1616a.point[29] = import3&0x20;//自定义接口
    SPC_SDIO_1616a.point[30] = import3&0x40;//自定义接口

    printf("SPC_SDIO_1616a_import0=%d\n", import0);
    printf("SPC_SDIO_1616a_import1=%d\n", import1);
    printf("SPC_SDIO_1616a_import2=%d\n", import2);
    printf("SPC_SDIO_1616a_import3=%d\n", import3);
}
void SPC_SDIO_1616c_Analyze_Data()  //接收的数据
{
    int import0, import1,import2,import3;
    import0 = (int)((short)(CAN0->recvBuffer.data[0]));
    import1 = (int)((short)(CAN0->recvBuffer.data[1]));
    import2 = (int)((short)(CAN0->recvBuffer.data[2]));
    import3 = (int)((short)(CAN0->recvBuffer.data[3]));

    SPC_SDIO_1616c.point[2] = import0&0x04;//前PLS中距离
    SPC_SDIO_1616c.point[3] = import0&0x08;//前PLS远距离
    SPC_SDIO_1616c.point[4] = import0&0x10;//前PLS故障
    SPC_SDIO_1616c.point[6] = import0&0x40;//前PLS近距离
    SPC_SDIO_1616c.point[7] = import0&0x80;//前PLS屏蔽信号

    SPC_SDIO_1616c.point[15] = import1&0x80;//后PLS近距离
    SPC_SDIO_1616c.point[16] = import2&0x01;//后PLS屏蔽信号

    SPC_SDIO_1616c.point[24] = import3&0x01;//后PLS中距离
    SPC_SDIO_1616c.point[25] = import3&0x02;//后PLS远距离
    SPC_SDIO_1616c.point[26] = import3&0x04;//后PLS报警
    printf("SPC_SDIO_1616c_import0=%d\n", import0);
    printf("SPC_SDIO_1616c_import1=%d\n", import1);
    printf("SPC_SDIO_1616c_import2=%d\n", import2);
    printf("SPC_SDIO_1616c_import3=%d\n", import3);
}
void SDIO_Out_Data()   //发送
{
	AGV_SDIO_Data_Outa.Q_Out[8];//自定义接口
	AGV_SDIO_Data_Outa.Q_Out[9];//自定义接口
	AGV_SDIO_Data_Outa.Q_Out[10]=1;//自定义接口
	AGV_SDIO_Data_Outa.Q_Out[11];//自定义接口
	AGV_SDIO_Data_Outa.Q_Out[13];// 安全继电器自动复位 急停旋开后复位
	AGV_SDIO_Data_Outa.Q_Out[14] ;//ALLOK检测   所有器件运行正常时触发
	AGV_SDIO_Data_Outa.Q_Out[17] = Sound_Control_Data.Sound_Control[0];//声音控制1
	AGV_SDIO_Data_Outa.Q_Out[18] = Sound_Control_Data.Sound_Control[1];//声音控制2
	AGV_SDIO_Data_Outa.Q_Out[19] = Sound_Control_Data.Sound_Control[2];//声音控制3
	AGV_SDIO_Data_Outa.Q_Out[20] ; //Sound_Control_Data.Sound_Control[3];//声音控制4
	AGV_SDIO_Data_Outa.Q_Out[21] = Lamplight_Control_Data.Blue_Light;//指示灯蓝色
	AGV_SDIO_Data_Outa.Q_Out[22] = Lamplight_Control_Data.Green_Light;//指示灯绿色
	AGV_SDIO_Data_Outa.Q_Out[23] = Lamplight_Control_Data.Red_Light;//指示灯红色

	SPC_SDIO_1616_Outa.SDIO_Out[1] =
			 AGV_SDIO_Data_Outa.Q_Out[8]|
			(AGV_SDIO_Data_Outa.Q_Out[9] <<1) |
			(AGV_SDIO_Data_Outa.Q_Out[10] << 2)|
			(AGV_SDIO_Data_Outa.Q_Out[11] << 3)|
			(AGV_SDIO_Data_Outa.Q_Out[13] << 5 )|
			(AGV_SDIO_Data_Outa.Q_Out[14] << 6) ;

	 SPC_SDIO_1616_Outa.SDIO_Out[2] =
			(AGV_SDIO_Data_Outa.Q_Out[17] << 1) |
			(AGV_SDIO_Data_Outa.Q_Out[18] << 2 ) |
			(AGV_SDIO_Data_Outa.Q_Out[19] << 3)|
			(AGV_SDIO_Data_Outa.Q_Out[20] << 4)|
			(AGV_SDIO_Data_Outa.Q_Out[21] << 5) |
			(AGV_SDIO_Data_Outa.Q_Out[22] << 6 )|
			(AGV_SDIO_Data_Outa.Q_Out[23] << 7) ;

	AGV_SDIO_Data_Outc.Q_Out[8]=1;//自定义接口
	AGV_SDIO_Data_Outc.Q_Out[9];//自定义接口
	AGV_SDIO_Data_Outc.Q_Out[10] ;//自定义接口
	AGV_SDIO_Data_Outc.Q_Out[11] ;//自定义接口
	AGV_SDIO_Data_Outc.Q_Out[12] ;//操作权信号输出
	AGV_SDIO_Data_Outc.Q_Out[13];//前PLS屏蔽控制
	AGV_SDIO_Data_Outc.Q_Out[14];//伺服on输出
	AGV_SDIO_Data_Outc.Q_Out[17];//启动输出
	AGV_SDIO_Data_Outc.Q_Out[18] ;//机器人上电
	AGV_SDIO_Data_Outc.Q_Out[19] ;//错误复位输出
	AGV_SDIO_Data_Outc.Q_Out[20] ;//伺服off输出
	AGV_SDIO_Data_Outc.Q_Out[21] ;//停止输出
	AGV_SDIO_Data_Outc.Q_Out[22];//后PLS屏蔽控制
	AGV_SDIO_Data_Outc.Q_Out[23];//机器人手制动切换

	SPC_SDIO_1616_Outc.SDIO_Out[1] =
		     AGV_SDIO_Data_Outc.Q_Out[8]|
			(AGV_SDIO_Data_Outc.Q_Out[9] << 1)|
			(AGV_SDIO_Data_Outc.Q_Out[10] << 2)|
			(AGV_SDIO_Data_Outc.Q_Out[11] << 3)|
			(AGV_SDIO_Data_Outc.Q_Out[12] << 4)|
			(AGV_SDIO_Data_Outc.Q_Out[13] << 5 )|
			(AGV_SDIO_Data_Outc.Q_Out[14] << 6) ;

	SPC_SDIO_1616_Outc.SDIO_Out[2] =
			(AGV_SDIO_Data_Outc.Q_Out[17] << 1)|
			(AGV_SDIO_Data_Outc.Q_Out[18] << 2)|
			(AGV_SDIO_Data_Outc.Q_Out[19] << 3)|
			(AGV_SDIO_Data_Outc.Q_Out[20] << 4)|
			(AGV_SDIO_Data_Outc.Q_Out[21] << 5)|
			(AGV_SDIO_Data_Outc.Q_Out[22] << 6 )|
			(AGV_SDIO_Data_Outc.Q_Out[23] << 7) ;

}



