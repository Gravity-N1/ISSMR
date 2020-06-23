
u16 Wheel_Current_Limit = Max_Current;//Max_Current=5500，电机的电流值
 /********************************************
函数名：void Wheel_Speed_Out(u8 flag)   
功能：轮子速度环函数，最后通过CAN通信打包发送至电机
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
	



/****************************************
函数名：void CAN_Send_Msg_To_Wheel(void)
功能：发送电流值到底盘电调，电机型号：RM3508，电调型号：C620
输入：无
返回：无
注意：底盘电调的反馈信息量较大，过多的CAN设备或发送过频繁会导致网络崩溃
*****************************************/
void CAN_Send_Msg_To_Wheel(u8 flag)
{ 
	u8 mbox;
	u32 i=0;
	CanTxMsg TxMessage;
	TxMessage.DLC=8;//数据段的长度8
	TxMessage.StdId=0x200;
	TxMessage.IDE=CAN_ID_STD;//使用标准ID
	TxMessage.RTR=CAN_RTR_Data;		  // 消息类型为数据帧，一帧8位

	if(flag)
	{
 	TxMessage.Data[0]=(Motor.Wheels_Out[0]>>8);			  //对应820R电调的协议
	TxMessage.Data[1]=(Motor.Wheels_Out[0]&0xff);	    
	TxMessage.Data[2]=(Motor.Wheels_Out[1]>>8);		  
	TxMessage.Data[3]=(Motor.Wheels_Out[1]&0xff);	
	TxMessage.Data[4]=(Motor.Wheels_Out[2]>>8);		
	TxMessage.Data[5]=(Motor.Wheels_Out[2]&0xff);	
	TxMessage.Data[6]=(Motor.Wheels_Out[3]>>8);	
	TxMessage.Data[7]=(Motor.Wheels_Out[3]&0xff);	
	}
	else
	 for(i=0;i<8;i++)
 	 TxMessage.Data[i]=0;
 	 mbox=CAN_Transmit (CANx_Chassis,&TxMessage);
 	 while((CAN_TransmitStatus(CANx_Chassis, mbox)==CAN_TxStatus_Failed)&&i<0xff) //等待发送结束，确保CAN要发送成功
 	 i++;
}

