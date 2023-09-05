#ifndef __PIN_H
#define __PIN_H

// 陀螺儀(MPU6050)
#define MPU6050_SCL_Port GPIOB
#define MPU6050_SCL_Pin GPIO_Pin_6
#define MPU6050_SDA_Port GPIOB
#define MPU6050_SDA_Pin GPIO_Pin_7

// 超聲波傳感器(HC-SR04)
#define HCSR04_Trig_Port GPIOA
#define HCSR04_Trig_Pin GPIO_Pin_0

#define HCSR04_Echo_1_Port GPIOA
#define HCSR04_Echo_2_Port GPIOA
#define HCSR04_Echo_3_Port GPIOA
#define HCSR04_Echo_1_Pin GPIO_Pin_1
#define HCSR04_Echo_2_Pin GPIO_Pin_2
#define HCSR04_Echo_3_Pin GPIO_Pin_3


// 輪胎1
#define MotorA_PWM_Port GPIOA
#define MotorA_AIN_Port GPIOB
#define MotorA_PWM_Pin GPIO_Pin_8
#define MotorA_AIN1_Pin GPIO_Pin_0
#define MotorA_AIN2_Pin GPIO_Pin_1
// 輪胎2
#define MotorB_PWM_Port GPIOA
#define MotorB_BIN_Port GPIOB
#define MotorB_PWM_Pin GPIO_Pin_9
#define MotorB_BIN1_Pin GPIO_Pin_13
#define MotorB_BIN2_Pin GPIO_Pin_14


// 邊刷
#define Brush_PWM_Port GPIOA
#define Brush_AIN_Port GPIOB
#define Brush_PWM_Pin GPIO_Pin_10
#define Brush_AIN1_Pin GPIO_Pin_4
#define Brush_AIN2_Pin GPIO_Pin_5

// 風機
#define fan_PWM_Port GPIOA
#define fan_PWM_Pin GPIO_Pin_11



// 藍芽串口(USART)
#define Serial_USART_Port GPIOB
#define Serial_USART_TX_Pin GPIO_Pin_10
#define Serial_USART_RX_Pin GPIO_Pin_11

// Key
#define Key_Port GPIOB
#define Key_Pin GPIO_Pin_12




#endif
