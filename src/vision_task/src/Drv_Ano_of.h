#ifndef __DRV_ANO_OF_H
#define __DRV_ANO_OF_H

#define U8 unsigned char
#define S16 signed short
#define S8 signed char
#define U32 unsigned int
//==引用
//#include "include.h" //
//#include "parameter.h"
//#include "stm32f4xx.h" //修改


//==定义/声明

typedef struct
{
	//
	U8 of_update_cnt;//光流数据更新计数。
	U8 alt_update_cnt;//高度数据更新计数。
	//
	U8 link_sta;//连接状态：0，未连接。1，已连接。
	U8 work_sta;//工作状态：0，异常。1，正常
	//
	U8 of_quality;
	//
	U8 of0_sta;
	S8 of0_dx;
	S8 of0_dy;
	//
	U8 of1_sta;
	S16 of1_dx;
	S16 of1_dy;
	//
	U8 of2_sta;
	S16 of2_dx;
	S16 of2_dy;	
	S16 of2_dx_fix;
	S16 of2_dy_fix;
	S16 intergral_x;
	S16 intergral_y;
	//
	U32 of_alt_cm;
	//
	float quaternion[4];
	//
	S16 acc_data_x;
	S16 acc_data_y;
	S16 acc_data_z;
	S16 gyr_data_x;
	S16 gyr_data_y;
	S16 gyr_data_z;

}_ano_of_st;

//飞控状态


//==数据声明
extern _ano_of_st ano_of;
//==函数声明
//static
static void AnoOF_DataAnl(U8 *data_buf,U8 num);

//public
U8 AnoOF_GetOneByte(U8 data);
void AnoOF_Check_State(float dT_s);
#endif

