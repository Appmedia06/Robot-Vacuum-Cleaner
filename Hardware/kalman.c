#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "kalman.h"
#include <math.h>

/* 變量定義 */
int16_t ori_accx, ori_accy, ori_accz; // 加速度的原始數據
int16_t ori_gyrox, ori_gyroy, ori_gyroz; // 陀螺儀原始數據


float temp_Accx; // X軸的加速度暫存變量
float temp_Accy; // Y軸的加速度暫存變量
float temp_Accz; // Z軸的加速度暫存變量

float temp_Gyrox; // X軸的陀螺儀暫存變量
float temp_Gyroy; // Y軸的陀螺儀暫存變量
float temp_Gyroz; // Z軸的陀螺儀暫存變量

float slope_Accx; // 加速度計算的x軸傾斜角度
float slope_Accy; // 加速度計算的y軸傾斜角度

float Pitch; // X最終傾斜角度
float Roll; // Y最終傾斜角度
float Yaw; // Z最終傾斜角度


void Calculate_MPU6050Angle(void)
{
	float Accx, Accy, Accz; // 三個方向的腳加速度值
	
	// 獲取原始數據
	MPU6050_GetAcc(&ori_accx, &ori_accy, &ori_accz); // 獲取加速度
	MPU6050_GetGyro(&ori_gyrox, &ori_gyroy, &ori_gyroz); // 獲取陀螺儀
	
	temp_Accx = ori_accx; // X軸的加速度暫存變量
	temp_Accy = ori_accy; // Y軸的加速度暫存變量
	temp_Accz = ori_accz; // Z軸的加速度暫存變量
	temp_Gyrox = ori_gyrox; // X軸的陀螺儀暫存變量
	temp_Gyroy = ori_gyroy; // Y軸的陀螺儀暫存變量
	temp_Gyroz = ori_gyroz; // Z軸的陀螺儀暫存變量
	
	
	/*
	處理角加速度
	加速度傳感器配置寄存器0X1C內寫入0x01,設置範圍為±2g。換算關係：2^16/4 = 16384LSB/g
	*/
	if(temp_Accx < 32764) 
		Accx = temp_Accx / 16384; // 計算x軸加速度
	else              
		Accx = 1 - (temp_Accx - 49152) / 16384;
	if(temp_Accy < 32764) 
		Accy = temp_Accy / 16384; // 計算y軸加速度
	else              
		Accy = 1 - (temp_Accy - 49152) / 16384;
	if(temp_Accz < 32764) 
		Accz = temp_Accz / 16384; // 計算z軸加速度
	else              
		Accz = 1 - (temp_Accz - 49152) / 16384;
	
	// 加速度反正切公式計算三個軸和水平面坐標系之間的夾角
	slope_Accx = (atan(Accy / Accz)) * 180 / 3.14;
	slope_Accy = (atan(Accx / Accz)) * 180 / 3.14;
	
	// 角度正負值
	if(temp_Accx < 32764) 
		slope_Accy = +slope_Accy;
	if(temp_Accx > 32764) 
		slope_Accy = -slope_Accy;
	if(temp_Accy < 32764) 
		slope_Accx = +slope_Accx;
	if(temp_Accy > 32764) 
		slope_Accx = -slope_Accx;
	
	
	/*
	計算角速度:
	陀螺儀配置寄存器0X1B內寫入0x18，設置範圍為±2000deg/s。換算關係：2^16/4000=16.4LSB/(°/S)
	*/
	if(temp_Gyrox < 32768) 
		temp_Gyrox = -(temp_Gyrox / 16.4);
	if(temp_Gyrox > 32768) 
		temp_Gyrox = +(65535 - temp_Gyrox) / 16.4;
	if(temp_Gyroy < 32768) 
		temp_Gyroy = -(temp_Gyroy / 16.4);
	if(temp_Gyroy > 32768) 
		temp_Gyroy = +(65535 - temp_Gyroy) / 16.4;
	if(temp_Gyroz < 32768) 
		temp_Gyroz = -(temp_Gyroz / 16.4);
	if(temp_Gyroz > 32768) 
		temp_Gyroz = +(65535-temp_Gyroz) / 16.4;
	
	
	// 卡爾曼濾波
	Kalman_Filter_X(slope_Accx, temp_Gyrox);
	Kalman_Filter_Y(slope_Accy, temp_Gyroy);
	get_Yaw(temp_Gyroz);
	
	
}



// 卡爾曼參數		
float Q_angle = 0.001;		// 角度數據置信度，角度噪聲的協方差
float Q_gyro  = 0.003;		// 角速度數據置信度，角速度噪聲的協方差  
float R_angle = 0.5;		// 加速度計測量噪聲的協方差
float dt      = 0.02;		// 濾波算法計算週期，由定時器定時20ms
char  H       = 1;			// H矩陣值
float Q_bias, Angle_err;	// Q_bias:陀螺儀的偏差  Angle_err:角度偏量 
float PCt_0, PCt_1, E;		// 計算的過程量
float K_0, K_1, t_0, t_1;	//卡爾曼增益  K_0:用於計算最優估計值  K_1:用於計算最優估計值的偏差 t_0/1:中間變量
float P[4] = {0,0,0,0};	    // 過程協方差矩陣的微分矩陣，中間變量
float PP[2][2] = { { 1, 0 },{ 0, 1 } }; // 過程協方差矩陣P


void Kalman_Filter_X(float Accel, float Gyro)
{
	/*
	Step1:先驗估計
	公式：X(k|k-1) = A*X(k-1|k-1) + B*U(k) 
	A = 1(單位矩陣), B = dt
	*/
	Pitch = Pitch + (Gyro - Q_bias) * dt; // 狀態方程,角度值等於上次最優角度加角速度減零漂後積分
	
	/*
	Step2:計算過程協方差矩陣的微分矩陣
	公式：P(k|k-1)=A*P(k-1|k-1)*A^T + Q 
	A = 1, A^T = 1(因為A事單位矩陣)
	Q(1,1) = cov(Angle,Angle)	Q(1,2) = cov(Q_bias,Angle)
	Q(2,1) = cov(Angle,Q_bias)	Q(2,2) = cov(Q_bias,Q_bias)
	*/
	P[0]= Q_angle - PP[0][1] - PP[1][0];
	P[1]= -PP[1][1]; // 先驗估計誤差協方差
	P[2]= -PP[1][1];
	P[3]= Q_gyro;
	PP[0][0] += P[0] * dt;   
	PP[0][1] += P[1] * dt;   
	PP[1][0] += P[2] * dt;
	PP[1][1] += P[3] * dt;	
	
	/*
	Step3:計算過程協方差矩陣的微分矩陣
	公式：Kg(k)= P(k|k-1)*H^T/(H*P(k|k-1)*H^T+R)
	Kg = (K_0,K_1) 對應Angle, Q_bias增益
	H = (1,0)	可由z=HX+v求出z:Accel
	R(觀測噪聲的協方差矩陣)
	*/
	PCt_0 = PP[0][0] * H;
	PCt_1 = PP[1][0] * H;
	E = R_angle + H * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	/*
	Step4:後驗估計誤差協方差
	公式  ：P(k|k)=(I-Kg(k)H)P(k|k-1)
	或寫成 :P(k|k)=P(k|k-1)-Kg(k)HP(k|k-1)
	*/
	t_0 = PCt_0;
	t_1 = H * PP[0][1];
	PP[0][0] -= K_0 * t_0;		
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	
	/*
	Step5:計算最優角速度值
	公式：X(k|k)= X(k|k-1)+Kg(k)(Z(k)-X(k|k-1))
	*/
	Angle_err = Accel - Pitch;	// Z(k)先驗估計 計算角度偏差
	Pitch += K_0 * Angle_err;	 // 後驗估計，給出最優估計值
	Q_bias += K_1 * Angle_err;	 // 後驗估計，跟新最優估計值偏差
	temp_Gyrox = Gyro - Q_bias;	 
}

void Kalman_Filter_Y(float Accel, float Gyro)
{
	/*
	Step1:先驗估計
	公式：X(k|k-1) = A*X(k-1|k-1) + B*U(k) 
	A = 1(單位矩陣), B = dt
	*/
	Roll = Roll + (Gyro - Q_bias) * dt; // 狀態方程,角度值等於上次最優角度加角速度減零漂後積分
	
	/*
	Step2:計算過程協方差矩陣的微分矩陣
	公式：P(k|k-1)=A*P(k-1|k-1)*A^T + Q 
	A = 1, A^T = 1(因為A事單位矩陣)
	Q(1,1) = cov(Angle,Angle)	Q(1,2) = cov(Q_bias,Angle)
	Q(2,1) = cov(Angle,Q_bias)	Q(2,2) = cov(Q_bias,Q_bias)
	*/
	P[0]= Q_angle - PP[0][1] - PP[1][0];
	P[1]= -PP[1][1]; // 先驗估計誤差協方差
	P[2]= -PP[1][1];
	P[3]= Q_gyro;
	PP[0][0] += P[0] * dt;   
	PP[0][1] += P[1] * dt;   
	PP[1][0] += P[2] * dt;
	PP[1][1] += P[3] * dt;	
	
	/*
	Step3:計算過程協方差矩陣的微分矩陣
	公式：Kg(k)= P(k|k-1)*H^T/(H*P(k|k-1)*H^T+R)
	Kg = (K_0,K_1) 對應Angle, Q_bias增益
	H = (1,0)	可由z=HX+v求出z:Accel
	R(觀測噪聲的協方差矩陣)
	*/
	PCt_0 = PP[0][0] * H;
	PCt_1 = PP[1][0] * H;
	E = R_angle + H * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	/*
	Step4:後驗估計誤差協方差
	公式  ：P(k|k)=(I-Kg(k)H)P(k|k-1)
	或寫成 :P(k|k)=P(k|k-1)-Kg(k)HP(k|k-1)
	*/
	t_0 = PCt_0;
	t_1 = H * PP[0][1];
	PP[0][0] -= K_0 * t_0;		
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	
	/*
	Step5:計算最優角速度值
	公式：X(k|k)= X(k|k-1)+Kg(k)(Z(k)-X(k|k-1))
	*/
	Angle_err = Accel - Roll;	// Z(k)先驗估計 計算角度偏差
	Roll += K_0 * Angle_err;	 // 後驗估計，給出最優估計值
	Q_bias += K_1 * Angle_err;	 // 後驗估計，跟新最優估計值偏差
	temp_Gyroy = Gyro - Q_bias;	 
}

void get_Yaw(float Gyro)
{
	Gyro += Gyro * dt;
	Yaw += Gyro;
}



//int freq = 50; 
//void cal_Offset(float *gyroX_offset, float *gyroY_offset, float *gyroZ_offset)
//{
//	static uint16_t g_GetZeroOffset = 0;
//	
//	// 間隔8ms一次採樣，統計125次，共1秒時間
//	if (g_GetZeroOffset++ < freq)
//	{
//		MPU6050_GetGyro(&gyroX, &gyroX, &gyroX);
//		*gyroX_offset += gyroX * dt;  // 每次8%积分，累计加权125次
//		*gyroY_offset += gyroY * dt;
//		*gyroZ_offset += gyroZ * dt;
//	}
//}




