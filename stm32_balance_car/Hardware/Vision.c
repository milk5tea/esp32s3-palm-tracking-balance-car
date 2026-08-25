#include "stm32f10x.h"                  // Device header
#include "Vision.h"
#include <string.h>
#include <stdlib.h>

/*全局变量，用于存储视觉模块数据*/
static MotionTarget Targets[VISION_MAX_TARGETS];	//目标数组
static uint8_t TargetCount;							//当前帧目标数量
static int16_t LockedID = -1;						//锁定目标ID，-1表示未锁定
static uint32_t RxTickCount;						//距上次收到视觉数据的毫秒计数，用于超时判空

/*串口接收缓冲，一行MOTION数据最长约90字节，128足够*/
static char RxBuf[128];
static uint8_t RxIdx;

/**
  * 函    数：视觉串口初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：使用USART1，PA9=TX，PA10=RX，波特率115200
  *           与调试打印共用USART1（Serial.c已配置），此处重复配置以保证独立可用
  *           与ESP32-CAM连接时，ESP32的TX接板上RX口（PA10），共地
  */
void Vision_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟

	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA9引脚初始化为复用推挽输出

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA10引脚初始化为上拉输入

	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;					//定义结构体变量
	USART_InitStructure.USART_BaudRate = 115200;			//波特率，与ESP32侧保持一致
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择
	USART_InitStructure.USART_Parity = USART_Parity_No;	//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART1, &USART_InitStructure);				//将结构体变量交给USART_Init，配置USART1

	/*中断输出配置*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断

	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2

	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设

	/*USART使能*/
	USART_Cmd(USART1, ENABLE);								//使能USART1，串口开始运行
}

/**
  * 函    数：解析一行MOTION数据
  * 参    数：Line 以'\0'结尾的一行数据，格式 "MOTION:n;id,x,y,area,miss;..."
  * 返 回 值：无
  * 注意事项：此函数在USART1中断里被调用，需保持快速
  */
static void Vision_ParseLine(char *Line)
{
	char *p;					//解析指针
	int i;						//循环变量
	int Count;					//目标数量
	int Id, X, Y, Miss;		//目标各字段的临时变量
	int32_t Area;			//面积可能达到76800，超出uint16范围，需用32位

	if (strncmp(Line, "MOTION:", 7) != 0) return;		//帧头不对，丢弃此行

	p = Line + 7;
	Count = atoi(p);									//解析目标数量
	if (Count > VISION_MAX_TARGETS) Count = VISION_MAX_TARGETS;	//目标数量限幅
	TargetCount = 0;

	while (*p && *p != ';') p++;						//跳过数量字段

	for (i = 0; i < Count; i ++)						//依次解析每个目标
	{
		if (*p != ';') break;							//字段不足，提前结束
		p ++;

		Id = atoi(p);	while (*p && *p != ',') p++;	p ++;
		X = atoi(p);	while (*p && *p != ',') p++;	p ++;
		Y = atoi(p);	while (*p && *p != ',') p++;	p ++;
		Area = atoi(p);	while (*p && *p != ',') p++;	p ++;
		Miss = atoi(p);	while (*p && *p != ';') p++;

		Targets[TargetCount].id = (int16_t)Id;
		Targets[TargetCount].x = (int16_t)X;
		Targets[TargetCount].y = (int16_t)Y;
		Targets[TargetCount].area = (uint32_t)Area;
		Targets[TargetCount].miss = (uint8_t)Miss;
		TargetCount ++;
	}
}

/**
  * 函    数：串口接收一个字节的状态机
  * 参    数：Byte 串口接收到的一个字节
  * 返 回 值：无
  * 注意事项：以'\n'为一行结束标志，收到完整一行后立即解析
  */
static void Vision_RxByte(uint8_t Byte)
{
	if (Byte == '\r') return;		//忽略回车符，防止其混入数据
	if (Byte == '\n')				//收到换行，一行结束
	{
		RxBuf[RxIdx] = '\0';		//添加字符串结束标志
		RxIdx = 0;					//接收位置归零
		Vision_ParseLine(RxBuf);	//立即解析
	}
	else if (RxIdx < sizeof(RxBuf) - 1)
	{
		RxBuf[RxIdx ++] = (char)Byte;	//存入缓冲
	}
	else
	{
		RxIdx = 0;					//行超长，丢弃整行，重新同步
	}
}

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)	//判断是否是USART1的接收事件触发的中断
	{
		uint8_t RxData = USART_ReceiveData(USART1);			//读取数据寄存器
		RxTickCount = 0;									//收到数据，超时计数清零
		Vision_RxByte(RxData);								//将字节交给接收状态机
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);		//清除USART1的RXNE标志位
	}
}

/**
  * 函    数：视觉模块时钟函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数必须在主程序中每隔1ms自动执行一次
  */
void Vision_Tick(void)
{
	if (RxTickCount < 300)		//300ms内收到过数据
	{
		RxTickCount ++;			//超时计数自增
	}
	else						//超过300ms没有收到数据，认为视觉模块掉线
	{
		TargetCount = 0;		//清空目标
		LockedID = -1;			//取消锁定
	}
}

/**
  * 函    数：选择锁定目标
  * 参    数：无
  * 返 回 值：锁定目标的ID，-1表示无目标
  * 注意事项：锁定目标短时丢失（miss<5）时继续维持锁定，不轻易换目标；
  *           锁定目标丢失后，选择面积最大且连续丢失少于3帧的目标重新锁定
  */
static int16_t Vision_SelectTarget(void)
{
	int i;					//循环变量
	int BestIdx;			//面积最大目标的索引
	uint32_t MaxArea;		//最大面积

	/*1. 锁定目标还活着吗？*/
	if (LockedID != -1)
	{
		for (i = 0; i < TargetCount; i ++)
		{
			if (Targets[i].id == LockedID && Targets[i].miss < 5)
			{
				return LockedID;	//锁定目标仍存活，维持锁定
			}
		}
		LockedID = -1;				//锁定目标丢失
	}

	/*2. 选择面积最大的目标重新锁定*/
	BestIdx = -1;
	MaxArea = 0;
	for (i = 0; i < TargetCount; i ++)
	{
		if (Targets[i].miss < 3 && Targets[i].area > MaxArea)
		{
			MaxArea = Targets[i].area;
			BestIdx = i;
		}
	}
	if (BestIdx != -1)
	{
		LockedID = Targets[BestIdx].id;
		return LockedID;
	}
	return -1;
}

/**
  * 函    数：获取锁定目标的X坐标
  * 参    数：无
  * 返 回 值：锁定目标的X坐标，范围0~320，-1表示当前无目标
  */
int16_t Vision_GetTargetX(void)
{
	int16_t Id;		//锁定目标ID
	int i;			//循环变量

	Id = Vision_SelectTarget();
	if (Id == -1) return -1;

	for (i = 0; i < TargetCount; i ++)
	{
		if (Targets[i].id == Id)
		{
			return Targets[i].x;
		}
	}
	return -1;
}

/**
  * 函    数：获取锁定目标的面积
  * 参    数：无
  * 返 回 值：锁定目标的面积，单位像素^2，0表示当前无目标
  */
uint32_t Vision_GetTargetArea(void)
{
	int16_t Id;		//锁定目标ID
	int i;			//循环变量

	Id = Vision_SelectTarget();
	if (Id == -1) return 0;

	for (i = 0; i < TargetCount; i ++)
	{
		if (Targets[i].id == Id)
		{
			return Targets[i].area;
		}
	}
	return 0;
}
