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
	��������int16_t Output_Limit(int16_t input, int16_t limit)  
  ���ܣ��޷�
	���룺���õ����ݡ����Ʒ�Χ
	���أ����
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
	��������void Wheel_Speed_Out(u8 flag)   
  ���ܣ������ٶȻ�����
	*********************************************/
void Wheel_Speed_Out(u8 flag)  
{		
 	static float Interval[4];	
  float P_part=0, I_part=0;
	float  Delta=0;
  u32 i;
		for(i=0;i<4;i++)
		{
			/*****************����***********/	
		 Delta=Motor.Target_Speed[i]-Motor.Feedback_Speed[i];
		 P_part = Wheels_PID.P* Delta;		
			if(P_part>6000)	       P_part=6000;
			else if(P_part<-6000)	 P_part=-6000;			 		
			/*****************���ֲ���*************/
			Interval[i] += Delta* Wheels_PID.I;		
			I_part = Interval[i];		

			if(I_part>600)	   I_part=600;
			if(I_part<-600)	   I_part=-600;		
					
			Motor.Wheels_Out[i]=(int16_t)(P_part+I_part); 
	    Motor.Wheels_Out[i]= Output_Limit( Motor.Wheels_Out[i], Wheel_Current_Limit);
		}	
			
  		CAN_Send_Msg_To_Wheel(flag);  
		
}
	
