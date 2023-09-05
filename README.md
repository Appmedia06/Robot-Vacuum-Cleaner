# Robot Vacuum Cleaner
基於stm32的掃地機器人
# Material
* 主板
  * Stm32F103C8T6 * 1
* 外設
  * MPU6050(陀螺儀) * 1
  * HC-SR04(超聲波傳感器) * 3
  * 馬達 * 3
  * TB661 (電機驅動) * 1
  * L298N (電機驅動) * 1
  * OLED (顯示屏) * 1
  * XL6009 (DC升壓) * 1
  * 風扇 * 1
  * 按鈕 * 1
  * LED * 1
* 其他
  * 輪胎 * 2
  * 厚紙板
  * 灰塵箱
  * 邊刷

# 外設說明
1. TB661:   用於驅動掃地機器人之雙輪
2. L298N:   用於驅動掃地機器人之邊刷
3. XL6009:  用於將5v電壓升壓至12v，給風扇供電
4. HC-SR04: 用於給掃地機器人避障
5. MPU6050: 計算Yaw角，用於做PID控制
6. OLED:    顯示掃地機器人狀態
7. 風扇:    用於吸入灰塵
8. 按鈕:    用於開始工作即停止工作(LPM)的開關
  
# 接線圖
<img src='https://github.com/Appmedia06/Parking_System/blob/master/image_folder/image_2.jpg' width=350/>

# 實際圖
* 接線
<img src='https://github.com/Appmedia06/Parking_System/blob/master/image_folder/image_2.jpg' width=350/>
* 側面
<img src='https://github.com/Appmedia06/Parking_System/blob/master/image_folder/image_2.jpg' width=350/>

# Youtube Video Link
* <a href="https://www.youtube.com/watch?v=LmQXcGfePbY/">Parking System(Number Plate Recognition)</a>