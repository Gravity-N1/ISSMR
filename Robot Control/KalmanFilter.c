 	/********************************************
	函数名：static double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
  功能：陀螺仪的数据卡尔曼滤波
	输入：浮点型data：ResrcData
	返回：无
	注意：使用时定义ProcessNiose_Q = 0.02，MeasureNoise_R = 6，可以获得较好的波形
	*********************************************/
static double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
}
