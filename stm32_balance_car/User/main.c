#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "Vision.h"
#include "PID.h"
#include "NRF24L01.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/*视觉追踪参数：追踪人的手掌，面积阈值需按实测的VA值修改*/
/*测试方法：串口接板上TX口(115200)，观察VX/VA打印，记录手掌在30cm、40cm、60cm、1m时晃动的VA值*/
#define VISION_AREA_NEAR	10000	//手掌到达30~40cm时实测的面积，大于等于此值停车对正
#define VISION_AREA_FAR		3000	//中远距离分界，小于此值全速接近

/*定义全局变量*/
int16_t AX, AY, AZ, GX, GY, GZ;		//读取MPU6050的原始数据
uint8_t TimerErrorFlag;	//定时器错误标志位，如果定时中断函数执行时间超过了定时时间，则此标志位置1
uint16_t TimerCount;	//定时器计数值，此值可用于计算定时中断函数具体的执行时间

float AngleAcc;			//由加速度计得到的角度值
float AngleGyro;		//由陀螺仪得到的角度值，执行互补滤波后，此值基本与Angle相等
float Angle;			//互补滤波后的角度值，准确且无漂移

uint8_t KeyNum, RunFlag;			//按键键码，运行标志位
uint8_t VisionMode;					//视觉追踪模式标志位，K2按键切换

int16_t LeftPWM, RightPWM;			//左PWM，右PWM
int16_t AvePWM, DifPWM;				//平均PWM，差分PWM

float LeftSpeed, RightSpeed;		//左速度，右速度
float AveSpeed, DifSpeed;			//平均速度，差分速度

/*定义PID结构体变量*/
PID_t AnglePID = {					//角度环PID结构体变量，定义的时候同时给部分成员赋初值
	.Kp = 5,						//比例项权重
	.Ki = 0.1,						//积分项权重
	.Kd = 5,						//微分项权重
	
	.OutMax = 100,					//输出限幅的最大值
	.OutMin = -100,					//输出限幅的最小值
	
	.OutOffset = 3,					//输出偏移
	
	.ErrorIntMax = 600,				//误差积分的最大值
	.ErrorIntMin = -600,			//误差积分的最小值
};

PID_t SpeedPID = {					//角度环PID结构体变量，定义的时候同时给部分成员赋初值
	.Kp = 2,						//比例项权重
	.Ki = 0.05,						//积分项权重
	.Kd = 0,						//微分项权重
	
	.OutMax = 20,					//输出限幅的最大值
	.OutMin = -20,					//输出限幅的最小值
	
	.ErrorIntMax = 150,				//误差积分的最大值
	.ErrorIntMin = -150,			//误差积分的最小值
};

PID_t TurnPID = {					//转向环PID结构体变量，定义的时候同时给部分成员赋初值
	.Kp = 4,						//比例项权重
	.Ki = 3,						//积分项权重
	.Kd = 0,						//微分项权重
	
	.OutMax = 50,					//输出限幅的最大值
	.OutMin = -50,					//输出限幅的最小值
	
	.ErrorIntMax = 20,				//误差积分的最大值
	.ErrorIntMin = -20,				//误差积分的最小值
};

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	MPU6050_Init();		//MPU6050初始化
	BlueSerial_Init();	//蓝牙串口初始化
	LED_Init();			//LED初始化
	Key_Init();			//按键初始化
	Motor_Init();		//电机初始化
	Encoder_Init();		//编码器初始化
	Serial_Init();		//串口初始化
	Vision_Init();		//视觉串口初始化
	NRF24L01_Init();	//NRF24L01初始化
	
	Timer_Init();		//定时器初始化，定时中断时间1ms
	
	while (1)
	{
		/*LED指示程序运行状态*/
		if (RunFlag) {LED_ON();} else {LED_OFF();}		//RunFlag非0时LED点亮
		
		/*按键控制*/
		KeyNum = Key_GetNum();		//获取键码
		if (KeyNum == 1)			//如果K1按下
		{
			/*RunFlag变量取非*/
			if (RunFlag == 0)		//如果RunFlag为0
			{
				PID_Init(&AnglePID);//启动程序时给PID初始化，清除之前可能遗留的参数
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID);
				RunFlag = 1;		//则RunFlag置1
			}
			else					//否则，RunFlag非0
			{
				RunFlag = 0;		//则RunFlag置0
			}
		}

		if (KeyNum == 2)			//如果K2按下
		{
			VisionMode = !VisionMode;	//视觉追踪模式标志位取非
		}
		
		/*OLED显示*/
		OLED_Clear();
		OLED_Printf(0, 0, OLED_6X8, "  Angle");						//OLED左侧显示角度环参数
		OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);		//角度环Kp
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);		//角度环Ki
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);		//角度环Kd
		OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", AnglePID.Target);	//角度环目标值
		OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);			//角度环实际值
		OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", AnglePID.Out);	//角度环输出值
		OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);				//显示GY，便于校准陀螺仪零漂
		OLED_Printf(56, 56, OLED_6X8, "Offset:%02.0f", AnglePID.OutOffset);		//显示输出偏移值
		OLED_Printf(50, 0, OLED_6X8, "Speed");						//OLED中间显示速度环参数
		OLED_Printf(50, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);		//速度环Kp
		OLED_Printf(50, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);		//速度环Ki
		OLED_Printf(50, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);		//速度环Kd
		OLED_Printf(50, 32, OLED_6X8, "%+05.1f", SpeedPID.Target);	//速度环目标值
		OLED_Printf(50, 40, OLED_6X8, "%+05.1f", AveSpeed);			//速度环实际值
		OLED_Printf(50, 48, OLED_6X8, "%+05.0f", SpeedPID.Out);		//速度环输出值
		OLED_Printf(88, 0, OLED_6X8, "Turn");						//OLED右侧显示转向环参数
		OLED_Printf(88, 8, OLED_6X8, "%05.2f", TurnPID.Kp);			//转向环Kp
		OLED_Printf(88, 16, OLED_6X8, "%05.2f", TurnPID.Ki);		//转向环Ki
		OLED_Printf(88, 24, OLED_6X8, "%05.2f", TurnPID.Kd);		//转向环Kd
		OLED_Printf(88, 32, OLED_6X8, "%+05.1f", TurnPID.Target);	//转向环目标值
		OLED_Printf(88, 40, OLED_6X8, "%+05.1f", DifSpeed);			//转向环实际值
		OLED_Printf(88, 48, OLED_6X8, "%+05.0f", TurnPID.Out);		//转向环输出值
		OLED_Update();				//OLED更新
		
		/*NRF24L01接收数据包处理*/
		if (NRF24L01_Receive() == 1)				//如果收到数据包
		{
			uint8_t ID = NRF24L01_RxPacket[0];		//字节0，规定为ID，用于区分不同的数据包
			
			/*如果ID是0x00，则是遥控数据包，且不要求数据回传*/
			/*如果ID是0x01，则是遥控数据包，且要求数据回传*/
			if (ID == 0x00 || ID == 0x01)
			{
				if (ID == 0x01)		//如果ID是0x01，进行数据回传
				{
					NRF24L01_TxPacket[0] = 0x02;					//回传数据包字节0，规定为ID，值为0x02
					NRF24L01_TxPacket[1] = (int8_t)LeftPWM;			//回传数据包字节1，规定为LeftPWM
					NRF24L01_TxPacket[2] = (int8_t)RightPWM;		//回传数据包字节2，规定为RightPWM
					*(float *)&NRF24L01_TxPacket[4] = Angle;		//回传数据包字节4 5 6 7，规定为Angle
					*(float *)&NRF24L01_TxPacket[8] = LeftSpeed;	//回传数据包字节8 9 10 11，规定为LeftSpeed
					*(float *)&NRF24L01_TxPacket[12] = RightSpeed;	//回传数据包字节12 13 14 15，规定为RightSpeed
					
					NRF24L01_Send();		//NRF24L01发送回传数据包
				}
				
				/*ID是0x00或0x01，都执行遥控控制*/
//				int8_t LH = NRF24L01_RxPacket[1];	//字节1，规定为左摇杆横向值
				int8_t LV = NRF24L01_RxPacket[2];	//字节2，规定为左摇杆纵向值
				int8_t RH = NRF24L01_RxPacket[3];	//字节3，规定为右摇杆横向值
//				int8_t RV = NRF24L01_RxPacket[4];	//字节4，规定为右摇杆纵向值
				uint8_t KEY = NRF24L01_RxPacket[5];	//字节4，规定为按键键码
				
				/*执行摇杆操作*/
				if (!VisionMode)	//视觉追踪模式下，摇杆不控制速度环和转向环
				{
					SpeedPID.Target = LV / 25.0;	//摇杆值LV缩放后，控制速度环目标值，前后行进控制
					TurnPID.Target = RH / 25.0;		//摇杆值RH缩放后，控制转向环目标值，左右转弯控制
				}
				
				/*执行按键操作*/
				if (KEY == 1)				//如果遥控器K1按下，则执行和平衡车K1按下一样的程序
				{
					/*RunFlag变量取非*/
					if (RunFlag == 0)		//如果RunFlag为0
					{
						PID_Init(&AnglePID);//启动程序时给PID初始化，清除之前可能遗留的参数
						PID_Init(&SpeedPID);
						PID_Init(&TurnPID);
						RunFlag = 1;		//则RunFlag置1
					}
					else					//否则，RunFlag非0
					{
						RunFlag = 0;		//则RunFlag置0
					}
				}
			}
		}
		
		/*蓝牙串口接收数据包处理*/
		/*规定的数据包格式为：[数据1,数据2,数据3,...]*/
		if (BlueSerial_RxFlag == 1)		//如果收到数据包
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");	//提取数据1，定义为标签Tag
			if (strcmp(Tag, "key") == 0)					//Tag为key，收到按键数据包
			{
				char *Name = strtok(NULL, ",");				//提取数据2，定义为按键名称
				char *Action = strtok(NULL, ",");			//提取数据3，定义为按键动作
				
				/*此处可执行按键操作，目前程序暂时没用到按键*/
			}
			else if (strcmp(Tag, "slider") == 0)			//Tag为slider，收到滑杆数据包
			{
				char *Name = strtok(NULL, ",");				//提取数据2，定义为滑杆名称
				char *Value = strtok(NULL, ",");			//提取数据3，定义为滑杆值
				
				/*执行滑杆操作*/
				if (strcmp(Name, "AngleKp") == 0)			//如果滑杆名称是AngleKp
				{
					AnglePID.Kp = atof(Value);				//则把滑杆值赋值给角度环Kp
				}
				else if (strcmp(Name, "AngleKi") == 0)		//如果滑杆名称是AngleKi
				{
					AnglePID.Ki = atof(Value);				//则把滑杆值赋值给角度环Ki
				}
				else if (strcmp(Name, "AngleKd") == 0)		//如果滑杆名称是AngleKd
				{
					AnglePID.Kd = atof(Value);				//则把滑杆值赋值给角度环Kd
				}
				else if (strcmp(Name, "SpeedKp") == 0)		//如果滑杆名称是SpeedKp
				{
					SpeedPID.Kp = atof(Value);				//则把滑杆值赋值给速度环Kp
				}
				else if (strcmp(Name, "SpeedKi") == 0)		//如果滑杆名称是SpeedKi
				{
					SpeedPID.Ki = atof(Value);				//则把滑杆值赋值给速度环Ki
				}
				else if (strcmp(Name, "SpeedKd") == 0)		//如果滑杆名称是SpeedKd
				{
					SpeedPID.Kd = atof(Value);				//则把滑杆值赋值给速度环Kd
				}
				else if (strcmp(Name, "TurnKp") == 0)		//如果滑杆名称是TurnKp
				{
					TurnPID.Kp = atof(Value);				//则把滑杆值赋值给转向环Kp
				}
				else if (strcmp(Name, "TurnKi") == 0)		//如果滑杆名称是TurnKi
				{
					TurnPID.Ki = atof(Value);				//则把滑杆值赋值给转向环Ki
				}
				else if (strcmp(Name, "TurnKd") == 0)		//如果滑杆名称是TurnKd
				{
					TurnPID.Kd = atof(Value);				//则把滑杆值赋值给转向环Kd
				}
				else if (strcmp(Name, "Offset") == 0)		//如果滑杆名称是Offset
				{
					AnglePID.OutOffset = atof(Value);		//则把滑杆值赋值给OutOffset
				}
			}
			else if (strcmp(Tag, "joystick") == 0)			//Tag为joystick，收到摇杆数据包
			{
				int8_t LH = atoi(strtok(NULL, ","));		//提取数据2，定义为摇杆值LH
				int8_t LV = atoi(strtok(NULL, ","));		//提取数据3，定义为摇杆值LV
				int8_t RH = atoi(strtok(NULL, ","));		//提取数据4，定义为摇杆值RH
				int8_t RV = atoi(strtok(NULL, ","));		//提取数据5，定义为摇杆值RV
				
				/*执行摇杆操作*/
				if (!VisionMode)	//视觉追踪模式下，摇杆不控制速度环和转向环
				{
					SpeedPID.Target = LV / 25.0;	//摇杆值LV缩放后，控制速度环目标值，前后行进控制
					TurnPID.Target = RH / 25.0;		//摇杆值RH缩放后，控制转向环目标值，左右转弯控制
				}
			}
			
			BlueSerial_RxFlag = 0;				//处理完成后，标志位置0，允许接收下一个数据包
		}

		/*视觉追踪模式控制*/
		if (VisionMode)					//视觉追踪模式下
		{
			int16_t VisionX = Vision_GetTargetX();			//获取锁定目标的X坐标
			uint32_t VisionArea = Vision_GetTargetArea();	//获取锁定目标的面积
			static uint16_t VisionPrintCount;				//串口打印分频计数
			static uint8_t VisionNear;						//近区滞回状态，1=已进入30~40cm近区

			if (VisionX >= 0)		//有目标
			{
				/*以目标面积估算距离，控制速度环目标值*/
				/*速度值参考遥控器尺度：满摇杆≈4*/
				/*到达30~40cm就停车对正，更远则接近；滞回避免临界抖动*/
				if (VisionArea >= VISION_AREA_NEAR || (VisionNear && VisionArea >= VISION_AREA_NEAR * 0.8))
				{
					VisionNear = 1;
					SpeedPID.Target = 0;		//已到达，停车仅转向对正
				}
				else
				{
					VisionNear = 0;
					if (VisionArea >= VISION_AREA_FAR)	//目标中等距离
					{
						SpeedPID.Target = 1.5;			//中速接近
					}
					else								//目标较远
					{
						SpeedPID.Target = 2.5;			//快速接近
					}
				}

				/*以目标X坐标控制转向环目标值，画面中心为160*/
				/*若实测转向方向相反，将(VisionX - 160)改为(160 - VisionX)*/
				TurnPID.Target = (VisionX - 160) / 25.0;
				if (TurnPID.Target > 4)			//转向环目标值限幅
				{
					TurnPID.Target = 4;
				}
				else if (TurnPID.Target < -4)
				{
					TurnPID.Target = -4;
				}

				/*串口调试打印：目标X坐标与面积，用于调阈值，调好后可删除此段*/
				VisionPrintCount ++;
				if (VisionPrintCount >= 100)	//主循环约100次打印一次，约1秒一次
				{
					VisionPrintCount = 0;
					Serial_Printf("VX:%d VA:%d\r\n", VisionX, VisionArea);
				}
			}
			else					//无目标
			{
				TurnPID.Target = 0;		//原地原方向静止等待，不旋转搜索
				SpeedPID.Target = 0;	//速度环目标值0，不前进
			}
		}

		/*蓝牙串口打印波形，需配合蓝牙串口小程序实现波形绘制*/
//		BlueSerial_Printf("[plot,%f,%f,%f]", AnglePID.ErrorInt, SpeedPID.ErrorInt, TurnPID.ErrorInt);
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0, Count1;
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/*定时中断函数1ms自动执行一次*/
		
		/*进入中断函数后，立刻清标志位*/
		/*如果中断函数退出前，标志位又置1了，说明中断函数执行时间超过了定时时间（1ms）*/
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		Key_Tick();				//调用按键的Tick函数
		Vision_Tick();			//调用视觉模块的Tick函数
		
		/*计次分频*/
		Count0 ++;				//计次自增
		if (Count0 >= 10)		//如果计次10次，则if成立，即if每隔10ms进一次
		{
			Count0 = 0;			//计次清零，便于下次计次
			
			/*在中断里读取MPU6050，可以保证读取间隔严格为1ms*/
			/*但要保证MPU6050_GetData执行时间不超过1ms*/
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
			
			/*校准陀螺仪Y轴零漂*/
			/*此值需实测确定，不同的设备零漂一般不同*/
			/*实测方法是，在完全静止时，观察OLED显示的GY值，即为零漂值*/
			/*然后在此处将零漂值减去，使得完全静止时，GY值为0*/
			GY -= 16;
			
			/*由加速度计计算得到角度值*/
			/*atan2计算反正切，得到角度（弧度制）， / 3.14159 * 180可将弧度制转为角度值*/
			AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;
			
			/*校准中心角度*/
			/*此值需实测确定，不同的设备中心角度一般不同*/
			/*实测方法是，使平衡车绝对竖直，观察OLED显示的角度环实际值，即为中心角度偏移值*/
			/*然后在此处将偏移值减去，使得绝对竖直时，角度环实际值为0*/
			AngleAcc += 0.5;
			
			/*由陀螺仪积分得到角度值*/
			/*互补滤波下，角度积分要在上次滤波后的Angle上进行*/
			/*公式中32768是int16_t变量的最大值，2000是陀螺仪配置的满量程2000度每秒，0.01是间隔时间10ms*/
			AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01;
			
			/*执行互补滤波*/
			float Alpha = 0.01;		//互补滤波系数，值越大，越偏向于加速度计，值越小，越偏向于陀螺仪
			Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;		//互补滤波计算，得到稳定的角度值
			
			/*平衡车倒地后自动停止PID程序*/
			if (Angle > 50 || Angle < -50)	//角度超过-50度~50度的范围，认为平衡车倒地了
			{
				RunFlag = 0;				//RunFlag置0，停止PID程序
			}
			
			/*执行PID调控程序*/
			if (RunFlag)					//RunFlag非0时，启动PID程序
			{
				/*角度环PID控制*/
				AnglePID.Actual = Angle;	//角度环实际值为Angle
				PID_Update(&AnglePID);		//调用封装好的函数，一步完成PID计算和更新
				AvePWM = -AnglePID.Out;		//角度环的输出值给到电机平均PWM，用于控制前后行进
				
				/*控制量转换*/
				LeftPWM = AvePWM + DifPWM / 2;		//由平均PWM和差分PWM计算得到左轮PWM
				RightPWM = AvePWM - DifPWM / 2;		//由平均PWM和差分PWM计算得到右轮PWM
				
				/*PWM限幅*/
				/*上式计算后，LeftPWM和RightPWM可能会超出电机允许的PWM范围，此处将PWM值范围限制在-100~100之内*/
				if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
				if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
				
				/*PWM输出给电机*/
				Motor_SetPWM(1, LeftPWM);		//LeftPWM输出给左轮电机
				Motor_SetPWM(2, RightPWM);		//RightPWM输出给右轮电机
			}
			else		//RunFlag为0时，停止PID程序
			{
				/*左右电机PWM均设置为0*/
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
			}
		}
		
		/*计次分频*/
		Count1 ++;				//计次自增
		if (Count1 >= 50)		//如果计次50次，则if成立，即if每隔50ms进一次
		{
			Count1 = 0;			//计次清零，便于下次计次
			
			/*获取编码器的计次值增量，并计算电机旋转速度*/
			/*Encoder_Get函数，可以获取两次读取编码器的计次值增量*/
			/*编码器磁铁旋转一圈，计次增量为44，编码器读取间隔是50ms（0.05s）*/
			/*因此磁铁旋转速度 = 计次增量 / 44 / 0.05，单位是转每秒*/
			/*平衡车使用的电机带有减速箱，减速比为9.27666*/
			/*因此电机输出轴旋转速度 = 磁铁旋转速度 / 9.27666，单位是转每秒*/
			LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;
			RightSpeed = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;
			
			/*信号量转换*/
			AveSpeed = (LeftSpeed + RightSpeed) / 2.0;	//由左轮速度和右轮速度计算得到平均速度
			DifSpeed = LeftSpeed - RightSpeed;			//由左轮速度和右轮速度计算得到差分速度
			
			/*执行PID调控程序*/
			if (RunFlag)					//RunFlag非0时，启动PID程序
			{
				/*速度环PID控制*/
				SpeedPID.Actual = AveSpeed;			//速度环实际值为AveSpeed
				PID_Update(&SpeedPID);				//调用封装好的函数，一步完成PID计算和更新
				AnglePID.Target = SpeedPID.Out;		//速度环的输出值给到角度环的目标值，构成串级PID
				
				/*转向环PID控制*/
				TurnPID.Actual = DifSpeed;			//转向环实际值为DifSpeed
				PID_Update(&TurnPID);				//调用封装好的函数，一步完成PID计算和更新
				DifPWM = TurnPID.Out;				//转向环的输出值给到电机差分PWM，用于控制左右转弯
			}
		}
		
		/*中断函数退出前，再次检查标志位*/
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			/*标志位又置1了，说明中断函数执行时间超过了定时时间（1ms）*/
			/*置TimerErrorFlag为1，表示定时中断错误*/
			TimerErrorFlag = 1;
			
			/*清标志位，避免中断连续触发，导致主函数完全无法执行*/
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}
		
		/*中断函数退出前，读取计数器的值，此值可用于测量中断函数的具体执行时间*/
		TimerCount = TIM_GetCounter(TIM1);
	}
}
