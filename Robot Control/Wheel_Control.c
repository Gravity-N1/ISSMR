#include "Wheel_Control.h"
#include "Wheel_Out.h"
#include "Remote.h"
#include "Holder.h"
#include "math.h"
#include "PowerLimit.h"
#include"vision.h"
#ifdef TANK_1 
struct Dual_PID_Para  Chassis_Follow_PID=
{ 
	200,0,0, 
	200,0,0  
};
#endif

#ifdef TANK_2 
struct Dual_PID_Para  Chassis_Follow_PID=
{ 
	200,0,0,
	200,0,0 
};
#endif


//角度闭环参数
struct PID_PARA Chassis_Angle_para = 
{
	30,0,0,//shell 
	0,0,0//core  
};


int16_t Chasiss_Follow_Limit=5000;  //底盘跟随输出限幅
float	chassis_f0 = 0;
float chassis_r0 = 0;
float chassis_y0 = 0; 

float Chassis_Target_Angle=0; //底盘与云台的目标角度

#define Mid_Angle 25
u16 Psc_Angle =30;
#define nin_dev 3

int16_t Angle2=0;
int16_t Get_Random_Angle(void)
{
  static u32 t=0;
    t++;
	if(t%3==0) 
		return 15;
	if(t%3==1) 
		return 0;
	if(t%3==2)
		return -15;	
    
}
void Rock_Random(u8 flag)
{
	static u8  cnt=0;

	if( flag == 0 )		return;
	

	if(cnt==0)
	{
	 Chassis_Target_Angle=Mid_Angle+Angle2;
	 if(abs(Chassis_Target_Angle - Holder.Yaw_Can_Angle)<nin_dev)
	 {
		cnt=1;
		Angle2=Get_Random_Angle();
	 }
	}
	else if(cnt==1)
	{
	 Chassis_Target_Angle=Mid_Angle+Psc_Angle+Angle2;
	 if(abs(Chassis_Target_Angle - Holder.Yaw_Can_Angle)<nin_dev)
	 {
		cnt=2;
		Angle2=Get_Random_Angle();
	 }
	}		
	else if(cnt==2)
	{
	 Chassis_Target_Angle=Mid_Angle+Angle2;
	 if(abs(Chassis_Target_Angle - Holder.Yaw_Can_Angle)<nin_dev)
	 {
		cnt=3;
		Angle2=Get_Random_Angle();
	 }
	}		
	else if(cnt==3)
	{
	 Chassis_Target_Angle=Mid_Angle-Psc_Angle+Angle2;
	 if(abs(Chassis_Target_Angle - Holder.Yaw_Can_Angle)<nin_dev)
	 {
		cnt=0;
		Angle2=Get_Random_Angle();
	 }
	}		
	

}

	/*******************************************
	函数名：void Lets_Rock(uint8_t flag)
  功能：底盘摇摆控制
	输入：使能标志位
	返回：无
  注意：此函数在于计算摇摆模式下当前时刻底盘与云台的角度，存在Chassis_Target_Angle变量里
	*********************************************/
void Lets_Rock(uint8_t flag )
{
	static uint32_t cnt = 0;
	static int16_t time_ms =17;	 //修改此值调整摇摆的速度
	static float angle = 50.0f;  //摇摆角度
	static float k ;

	k = angle / time_ms;	
	if( flag == 0 )
	{
		return;
	}		
	cnt ++;
	
	if( cnt <= time_ms )
	{
		Chassis_Target_Angle = -( cnt * k );
	}
	
	else if( cnt <= (2 * time_ms) )
	{
		Chassis_Target_Angle = -( (2 * time_ms - cnt) * k );
	}
	
		
	else if( cnt <= (3 * time_ms))
	{
		Chassis_Target_Angle = ( (cnt - ( 2 * time_ms )) * k );
	}
	
	else if( cnt <= (4 * time_ms))
	{
		Chassis_Target_Angle = ( (4 * time_ms - cnt) * k );
	}
	else
	{
		cnt = 0;
		Chassis_Target_Angle = 0;
	}
	
}

 
/********************************************
	函数名：void Chassis_Control(float Chassis_Target_Angle)
  功能：底盘跟随的PID函数
	输入：Chassis_Target_Angle，云台和底盘的目标夹角
	返回：无
	注意：需要调的参数：PID和限幅
	*********************************************/
float Chassis_Follow_Control(float Chassis_Target_Angle)
{
//底盘双环跟随/实际单环也可以
	float shell_delta=0,core_delta=0;
	float shell_p_part=0;
	float core_p_part=0,shell_out=0;
	
  float Now_6050_Angle=0;	
	
  static float Can_Angle_Speed=0;
  static float Last_6050_Angle=0;	
	
	Now_6050_Angle=Holder.Yaw_6050_Angle;             //取当前编码器的角度值
	Can_Angle_Speed=Now_6050_Angle-Last_6050_Angle;   //两个角度差分得到角速度
	Last_6050_Angle=Now_6050_Angle;
		/***********PID***************/
	shell_delta = Chassis_Target_Angle - Holder.Yaw_6050_Angle;  //角度偏差
	shell_p_part=shell_delta*Chassis_Follow_PID.shell_P*0.1f;  //位置式PID的比例

	shell_out=shell_p_part;

	core_delta=shell_out+ Can_Angle_Speed;
	core_p_part=core_delta*Chassis_Follow_PID.core_P*0.1f;    //内环PID的P

	if      (core_p_part>Chasiss_Follow_Limit)   core_p_part=Chasiss_Follow_Limit;
	else if (core_p_part<-Chasiss_Follow_Limit)  core_p_part=-Chasiss_Follow_Limit;
		

return 	-core_p_part;

}




/*******************************************************************************
* Function Name  : 底盘角度闭环
* Description    : 
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
float angle_now;
float angle_last;
int16_t angle_out;
float angle_speed_out;
int16_t remote_now,remote_last,remote_temp;
float remote_interval;
float delta_angle_speed, angle_p_speed, angle_i_speed;
float delta_angle;

int8_t Angle_Amended(void)
{
	float angle_p_para, angle_i_para;
	float angle_interval = 0, angle_speed_interval = 0;
	static float angle=0,angle_last=0;

	  remote_interval += remote_info.left_LR/1000.0f;

		delta_angle = remote_interval - (-Holder.Yaw_6050_Angle * 2.0f);
		angle_p_para = delta_angle * Subsection_Chassis_p(delta_angle) * 0.1f;
		angle_interval += delta_angle;//增量式	
		angle_i_para = angle_interval * Chassis_Angle_para.shell_I;
		
		angle_out = angle_p_para + angle_i_para;	
		angle_speed_out = (float)Limiter(angle_out,1000);
		Chassis_Target_Angle = angle_speed_out;

}

/********************************************
	函数名：void Chassis_Remote_Dispack(void)  
  功能：将遥控器发过来的数据转换成最终轮子的目标速度
	输入：无
	返回：无
	注意：xyz系数分别决定了车子前后左右和转向的速度
*********************************************/
	float kp1=5.5f,kp2=6;
float temp_left_LR;
void Chassis_Remote_Dispack( uint8_t flag)  
{
	u8 i;
	int16_t x=10.0f,y=10.0f;//发送数据电流值范围 -5000~+5000, 10000/1320=8	x是速度


	float temp_right_UD,temp_right_LR;
	float temp_left_UD;
	
	temp_right_LR = remote_info.right_LR;
	temp_right_UD = remote_info.right_UD;
	temp_left_LR  = Chassis_Target_Angle;	

	Motor.Target_Speed[0]=x*(-temp_right_UD - temp_right_LR) + y* temp_left_LR;
	Motor.Target_Speed[1]=x*( temp_right_UD - temp_right_LR) + y* temp_left_LR;
	Motor.Target_Speed[2]=x*(-temp_right_UD + temp_right_LR) + y* temp_left_LR;
	Motor.Target_Speed[3]=x*( temp_right_UD + temp_right_LR) + y* temp_left_LR;

  for(i=0;i<4;i++)
	{
	 if(Motor.Target_Speed[i]>7000)  Motor.Target_Speed[i]=7000;
	 if(Motor.Target_Speed[i]<-7000) Motor.Target_Speed[i]=-7000;
	}
}

float soft_start(float input)
{
	static float  delta=0;
	static int8_t cnt=0;
  static float last_input;
  if(input!=last_input)  	
	{
		delta=input-last_input;
		last_input=input;
		cnt=10;
	}
	else
		delta=0;
	
	if(cnt>0)
		cnt--;
	
	if(cnt<1) cnt=1;
	 return last_input+delta/cnt;	
}

void Chasis_Motion_Control(u8 mode)
{	
	static float temp;
	static u32 _cnt=0;
	u8 i;
	_cnt++;
	///////////////
	if(mode==Remote)
	{
		if(_cnt%10==0)	
		{			
    Lets_Rock(0);			
		chassis_y0=Chassis_Follow_Control(0);		
		}
	  Chassis_Remote_Dispack(1); 	
	
	}
	else if((mode==Patrol)||(mode==Supply))
	{
	if(_cnt%10==0)	
   Lets_Rock(0);
	 Chassis_Remote_Dispack(1);  	
	} 
	else if(mode==Attack)
	{
//	  Rock_Random(1);		
//		if(_cnt%10==0)	
//		{
//	  Lets_Rock(0);	
//	  temp=Chassis_Follow_Control(Chassis_Target_Angle);	
//		}
//		chassis_y0= soft_start( temp);  //使摇摆平滑	
//	  Chassis_Remote_Dispack(0,  0,  chassis_y0);  	

			if(_cnt%10==0)	
		{			
    Lets_Rock(0);			
		chassis_y0=Chassis_Follow_Control(0);		
		}
	  Chassis_Remote_Dispack(1); 	
	}
	else if(mode==Escape)
	{		if(_cnt%10==0)	
		{
	  Lets_Rock(1);			
    chassis_y0=Chassis_Follow_Control(0);
		}			
	  Chassis_Remote_Dispack(1); 
	}
	else
	{
	 for(i=0;i<4;i++)
	 Motor.Target_Speed[i]=0;   //关掉输出
	}
		
	 odometry_encoder(Motor.Feedback_Speed);  //解算里程计		
	 
}

void odometry_encoder(int16_t * speed_raw_list)
{
	u8 i;
	float u_list[4];
	for( i=0;i<4;i++)
	{
		u_list[i]=*(speed_raw_list++)*0.15f*3.14159f*1.414f/60;
	}
	Send.speed_x=(-u_list[0]+u_list[1]+u_list[2]-u_list[3])/4*1.414f/2/18.89f;
  Send.speed_y =(-u_list[0]-u_list[1]+u_list[2]+u_list[3])/4*1.414f/2/18.89f;
  Send.angle_speed= -Holder.Yaw_Extern_Imu_Angle_Speed*1000/57.3f; //底盘角速度，单位
  Send.distance_x+=Send.speed_x/1000;
  Send.distance_y+=Send.speed_y/1000;
}
	
//角度闭环分段PID
float Subsection_Chassis_p(float input_delta)
{
	float Chassis_Angle_P;
	if(input_delta > 100)
		Chassis_Angle_P = 80;
	if(input_delta > 80 && input_delta < 100)
		Chassis_Angle_P = 70;
	if(input_delta > 60 && input_delta <80)
		Chassis_Angle_P = 60;
	if(input_delta > 40 && input_delta <60)
		Chassis_Angle_P = 50;
	if(input_delta< 40)
		Chassis_Angle_P = 40;
	return Chassis_Angle_P;
}

float Limiter(float input, float limit)
{
	if(input > limit) input = limit;
	if(input < -limit) input = -limit;
	return input;
}




