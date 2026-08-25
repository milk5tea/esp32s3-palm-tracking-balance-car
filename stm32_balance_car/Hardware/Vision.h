#ifndef __VISION_H
#define __VISION_H

#include "stdint.h"

#define VISION_MAX_TARGETS 5		//最大目标数量，需大于等于ESP32侧最大目标数（当前为4）

typedef struct
{
	int16_t id;			//目标ID
	int16_t x;			//目标质心X坐标，范围0~320
	int16_t y;			//目标质心Y坐标，范围0~240
	uint32_t area;		//目标面积（像素^2），可用于估算距离
	uint8_t miss;		//目标连续丢失帧数
} MotionTarget;

void Vision_Init(void);				//视觉串口初始化（USART3，115200）
void Vision_Tick(void);				//视觉模块时钟函数，需在1ms定时中断里调用
int16_t Vision_GetTargetX(void);	//获取锁定目标的X坐标，-1表示当前无目标
uint32_t Vision_GetTargetArea(void);//获取锁定目标的面积，0表示当前无目标

#endif
