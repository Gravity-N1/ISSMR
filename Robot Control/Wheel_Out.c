#include "Wheel_Out.h"
#include"sys.h"
#include"usart.h"
#include"CAN.h"
#include"holder.h"
#include"Remote.h"
#include"Wheel_Control.h"
#include "PowerLimit.h"
#include "referee.h"

struct Wheels_Info Motor=
{
0,0,0,0
};

 	/********************************************
	函数名：int16_t Output_Limit(int16_t input, int16_t limit)  
  功能：限幅
	输入：作用的数据、限制范围
	返回：输出
	*********************************************/
int16_t Output_Limit(int16_t input, int16_t limit)
	{
	if(input >limit)        	return limit;
	else if(input<-limit)	    return -limit;	
	else                      return input;
	}


struct Single_PID_Para Wheels_PID=
{
5,0,0  //5 .05 
};

u16 Wheel_Current_Limit = Max_Current;
 	/********************************************
	函数名：void Wheel_Speed_Out(u8 flag)   
  功能：轮子速度环函数
	*********************************************/
void Wheel_Speed_Out(u8 flag)  
{		
 	static float Interval[4];	
  float P_part=0, I_part=0;
	float  Delta=0;
  u32 i;
		for(i=0;i<4;i++)
		{
			/*****************比例***********/	
		 Delta=Motor.Target_Speed[i]-Motor.Feedback_Speed[i];
		 P_part = Wheels_PID.P* Delta;		
			if(P_part>6000)	       P_part=6000;
			else if(P_part<-6000)	 P_part=-6000;			 		
			/*****************积分操作*************/
			Interval[i] += Delta* Wheels_PID.I;		
			I_part = Interval[i];		

			if(I_part>600)	   I_part=600;
			if(I_part<-600)	   I_part=-600;		
					
			Motor.Wheels_Out[i]=(int16_t)(P_part+I_part); 
	    Motor.Wheels_Out[i]= Output_Limit( Motor.Wheels_Out[i], Wheel_Current_Limit);
		}	
			
  		CAN_Send_Msg_To_Wheel(flag);  
		
}
	
