#ifndef __KALMAN_H
#define __KALMAN_H

extern float Pitch; // X最終傾斜角度
extern float Roll; // Y最終傾斜角度
extern float Yaw; // Z最終傾斜角度

void Calculate_MPU6050Angle(void);
void Kalman_Filter_X(float Accel, float Gyro);
void Kalman_Filter_Y(float Accel, float Gyro);

void get_Yaw(float Gyro);

void cal_Offset(float *gyroX_offset, float *gyroY_offset, float *gyroZ_offset);

#endif
