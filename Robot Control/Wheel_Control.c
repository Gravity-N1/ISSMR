/*******************************************************************************
* Function Name  : 小车角度闭环
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

	delta_angle = Recieve.sdim_angle_out - SDIM_6050_angle;
	angle_p_para = delta_angle * Chassis_Angle_P;
	angle_interval += delta_angle;//增量式	
	angle_i_para = angle_interval * Chassis_Angle_para.shell_I;
		
	angle_out = angle_p_para + angle_i_para;	
	angle_speed_out = (float)Limiter(angle_out,1000);//输出限幅
	Chassis_Target_Angle = -angle_speed_out;
	//由于UWB定位的问题，出现角度过0点问题，在y轴区域由180度——>-180度，本段解决角度在180是过0点问题
	if(Chassis_Target_Angle > 180)
	 {
	    Chassis_Target_Angle = Chassis_Target_Angle-360;
	 }				
	else if (Chassis_Target_Angle < -180)
	 {
	    Chassis_Target_Angle = Chassis_Target_Angle + 360;
	 }
	else
	 {
	    Chassis_Target_Angle = Chassis_Target_Angle;
	 }		

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

	float temp_right_UD,temp_right_LR;//temp_right_LR为小车横向平移叠加速度量，本课题中暂时未使用
	float temp_left_UD;
	
	temp_left_LR = ( Chassis_Target_Angle )*5;//小车左右旋转速度量，适当放大输出
	
	//障碍物判断，有障碍物时，速度为0,；没有时，跟随人运动。
	if(Recieve.sdim_stop_flag == 0)//flat=0,stop
	    speed_out = 0; 
	else
	    speed_out = dis_out;//距离环PID输出幅值给速度输出量
	temp_right_UD = speed_out;//小车前进后退速度量
	
	//速度矢量合成
	Motor.Target_Speed[0]=x*(-temp_right_UD - temp_right_LR) + y* temp_left_LR;
	Motor.Target_Speed[1]=x*( temp_right_UD - temp_right_LR) + y* temp_left_LR;
	Motor.Target_Speed[2]=x*(-temp_right_UD + temp_right_LR) + y* temp_left_LR;
	Motor.Target_Speed[3]=x*( temp_right_UD + temp_right_LR) + y* temp_left_LR;

  for(i=0;i<4;i++)//电机电流限幅，防止电机跑飞
	{
	 if(Motor.Target_Speed[i]>7000)  Motor.Target_Speed[i]=7000;
	 if(Motor.Target_Speed[i]<-7000) Motor.Target_Speed[i]=-7000;
	}
}

/*******************************************************************************
* Function Name  : 小车距离闭环
* Description    : 保持小车和目标的相对距离稳定在1000mm左右，实现跟随效果
* Input          : None 
* Output         : 距离PID输出折算成电流值
* Return         : None
****************************************************************************** 
void Diatance_Amended(void)
{
	float dis_p_para, dis_i_para;
	float dis_interval = 0;
	float delta_dis =0;
	float dis_out=0;
	float target_distance=1000.0f;//目标距离1000mm

	delta_dis = Recieve.sdim_distance - target_distance;
	dis_p_para = delta_dis * Chassis_Dis_pare.shell_P;//结构体参数P
	dis_interval += delta_dis;//增量式PID	
	dis_i_para = dis_interval * Chassis_Dis_para.shell_I;//结构体参数I
		
	dis_out = (float)Limiter((dis_p_para + dis_i_para),7000);//输出限幅

}
