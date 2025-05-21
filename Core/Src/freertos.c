/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iImu.h"
#include "MPU6050/mpu6050.h"
#include "stdio.h"
#include "iDrive_Config.h"
#include "iPrint.h"
#include <math.h>
#include "firstOrderFilter.h"
#include "Bgc32.h"
#include "board.h"
#include "iHMC5883.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define Kp 7.2f
#define Ki 0.0f
#define halfT 0.001f
float q_0 = 1, q_1 = 0, q_2 = 0, q_3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;
float P,R, Y;
float accelOneG=9.8065;
#define magneto 0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
IMU_DATA_STRUCT ImuData;
IMU_TEMP_STRUCT ImuTemp;
HMC5883_DATA_STRUCT HMC5883Data;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ACCEL_SCALE_FACTOR 0.00059854//0.00119708f  // (1/8192) * 9.8065  (8192 LSB = 1 G)
#define GYRO_SCALE_FACTOR  0.00026646f  // (1/65.5) * pi/180   (65.5 LSB = 1 DPS)
float dt500Hz=0;
float accel_TC_Bias_Slope_x;
float accel_TC_Bias_Slope_y;
float accel_TC_Bias_Slope_z;
float accel_TC_Bias_Intercept_x;
float accel_TC_Bias_Intercept_y;
float accel_TC_Bias_Intercept_z;
float gyro_TC_Bias_Slope_r;
float gyro_TC_Bias_Slope_p;
float gyro_TC_Bias_Slope_y;
float gyro_TC_Bias_Intercept_r;
float gyro_TC_Bias_Intercept_p;
float gyro_TC_Bias_Intercept_y;
float accelTCBias[3] = { 0.0f, 0.0f, 0.0f };
float gyroTCBias[3];
float accel500Hz[3] = {0.0f, 0.0f, 0.0f};
float gyro500Hz[3] = {0.0f, 0.0f, 0.0f};
float gyroRTBias[3] = {0.0f, 0.0f, 0.0f};
float evvgcCFAttitude500Hz[3] = {0.0f, 0.0f, 0.0f};
float accAngleSmooth[3];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


/* USER CODE END Variables */
osThreadId GimbalCoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


void MPU6050_Calibration()
{
    uint16_t sampleRate      = 100;
    uint16_t numberOfSamples = 100;

    float accelBias1[3]       = { 0.0f, 0.0f, 0.0f };
    float gyroBias1[3]        = { 0.0f, 0.0f, 0.0f };
    float mpu6050Temperature1 = 0.0f;

    float accelBias2[3]       = { 0.0f, 0.0f, 0.0f };
    float gyroBias2[3]        = { 0.0f, 0.0f, 0.0f };
    float mpu6050Temperature2 = 0.0f;

    uint16_t index;
    //mpu6050Calibrating = true;

    printf("\nMPU6050 Calibration:\n\r");

    ///////////////////////////////////
    // Get samples at temperature1
    ///////////////////////////////////
    IMU_STRUCT* pImu = GetImuStr_Addr();

    printf("\nBegin 1st MPU6050 Measurements...\n");

    for (index = 0; index < numberOfSamples; index++)
    {
    	pImu->GetData(&ImuData);
    	float temp = Read_Temperature();

        accelBias1[0]    += ImuData.accel_x;
        accelBias1[1]    += ImuData.accel_y;
        accelBias1[2]	 += (ImuData.accel_z -9.8065);
        gyroBias1[0]     += ImuData.gyro_r;
        gyroBias1[1]     += ImuData.gyro_p;
        gyroBias1[2]     += ImuData.gyro_y;
        mpu6050Temperature1  += temp;

        printf("x=%f, y=%f, z=%f, accelBias1[0]=%f, accelBias1[1]=%f, accelBias1[2]=%f \r\n", ImuData.accel_x, ImuData.accel_y, ImuData.accel_z, accelBias1[0], accelBias1[1], accelBias1[2]);
        osDelay(sampleRate);
    }

    accelBias1[0]   /= (float) numberOfSamples;
    accelBias1[1]   /= (float) numberOfSamples;
    accelBias1[2]   /= (float) numberOfSamples;
    gyroBias1[0]    /= (float) numberOfSamples;
    gyroBias1[1]    /= (float) numberOfSamples;
    gyroBias1[2]    /= (float) numberOfSamples;
    mpu6050Temperature1 /= (float) numberOfSamples;
    printf("accelBias1[0]=%f, accelBias1[1]=%f, accelBias1[2]=%f \r\n",accelBias1[0],accelBias1[1],accelBias1[2]);
    printf("mpu6050Temperature1=%f",mpu6050Temperature1);
    printf("\n\nEnd 1st MPU6050 Measurements\n\r");

    osDelay(10000);
    ///////////////////////////////////
    // Get samples at temperature2
    ///////////////////////////////////
    printf("\nBegin 2nd MPU6050 Measurements...\n\r");

    for (index = 0; index < numberOfSamples; index++)
    {
    	pImu->GetData(&ImuData);
    	float temp = Read_Temperature();
        //rawAccel[ZAXIS].value = rawAccel[ZAXIS].value - 8192;
//    	float accel_z = ImuData.accel_z-16384;

        accelBias2[0]    += ImuData.accel_x;
        accelBias2[1]    += ImuData.accel_y;
        accelBias2[2]	 += (ImuData.accel_z -9.8065);
        gyroBias2[0]     += ImuData.gyro_r;
        gyroBias2[1]     += ImuData.gyro_p;
        gyroBias2[2]     += ImuData.gyro_y;
        mpu6050Temperature2  += temp;
        printf("x=%f, y=%f, z=%f, accelBias1[0]=%f, accelBias1[1]=%f, accelBias1[2]=%f \r\n", ImuData.accel_x, ImuData.accel_y, ImuData.accel_z, accelBias2[0], accelBias2[1], accelBias2[2]);
        osDelay(sampleRate);
    }

    accelBias2[0]   /= (float) numberOfSamples;
    accelBias2[1]   /= (float) numberOfSamples;
    accelBias2[2]   /= (float) numberOfSamples;
    gyroBias2[0]    /= (float) numberOfSamples;
    gyroBias2[1]    /= (float) numberOfSamples;
    gyroBias2[2]    /= (float) numberOfSamples;
    mpu6050Temperature2 /= (float) numberOfSamples;
    printf("accelBias2[0]=%f, accelBias2[1]=%f, accelBias2[2]=%f \r\n",accelBias2[0],accelBias2[1],accelBias2[2]);
    printf("mpu6050Temperature2=%f",mpu6050Temperature2);
    printf("\n\nEnd 2nd MPU6050 Measurements\n\r");

//    pCalibrationImu->CalibrationData(&ImuCalData);
	accel_TC_Bias_Slope_x = (accelBias2[0] - accelBias1[0]) / (mpu6050Temperature2 - mpu6050Temperature1);
	accel_TC_Bias_Slope_y = (accelBias2[1] - accelBias1[1]) / (mpu6050Temperature2 - mpu6050Temperature1);
	accel_TC_Bias_Slope_z = (accelBias2[2] - accelBias1[2]) / (mpu6050Temperature2 - mpu6050Temperature1);

	printf("accel_TC_Bias_Slope_x=%f, accel_TC_Bias_Slope_y=%f, accel_TC_Bias_Slope_z=%f \r\n", accel_TC_Bias_Slope_x, accel_TC_Bias_Slope_y, accel_TC_Bias_Slope_z);

	accel_TC_Bias_Intercept_x = accelBias2[0] - (accel_TC_Bias_Slope_x * mpu6050Temperature2);
	accel_TC_Bias_Intercept_y = accelBias2[1] - (accel_TC_Bias_Slope_y * mpu6050Temperature2);
	accel_TC_Bias_Intercept_z = accelBias2[2] - (accel_TC_Bias_Slope_z * mpu6050Temperature2);

	gyro_TC_Bias_Slope_r = (gyroBias2[0] - gyroBias1[0]) / (mpu6050Temperature2 - mpu6050Temperature1);
	gyro_TC_Bias_Slope_p = (gyroBias2[1] - gyroBias1[1]) / (mpu6050Temperature2 - mpu6050Temperature1);
	gyro_TC_Bias_Slope_y = (gyroBias2[2] - gyroBias1[2]) / (mpu6050Temperature2 - mpu6050Temperature1);

	gyro_TC_Bias_Intercept_r = gyroBias2[0] - (gyro_TC_Bias_Slope_r * mpu6050Temperature2);
	gyro_TC_Bias_Intercept_p = gyroBias2[1] - (gyro_TC_Bias_Slope_p * mpu6050Temperature2);
	gyro_TC_Bias_Intercept_y = gyroBias2[2] - (gyro_TC_Bias_Slope_y * mpu6050Temperature2);
    ///////////////////////////////////
}

void computeMPU6050TCBias()
{
	float Temperature = Read_Temperature();

	accelTCBias[0] =  accel_TC_Bias_Slope_x * Temperature + accel_TC_Bias_Intercept_x;
	accelTCBias[1] =  accel_TC_Bias_Slope_y * Temperature + accel_TC_Bias_Intercept_y;
	accelTCBias[2] =  accel_TC_Bias_Slope_z * Temperature + accel_TC_Bias_Intercept_z;
	gyroTCBias[0] = gyro_TC_Bias_Slope_r * Temperature + gyro_TC_Bias_Intercept_r;
	gyroTCBias[1]= gyro_TC_Bias_Slope_p * Temperature + gyro_TC_Bias_Intercept_p;
	gyroTCBias[2]= gyro_TC_Bias_Slope_y * Temperature + gyro_TC_Bias_Intercept_y;
}
void computeMPU6050RTData(void)
{
    uint8_t  axis;
    uint16_t samples;

    IMU_STRUCT* pImu = GetImuStr_Addr();

    double accelSum[3]    = { 0.0f, 0.0f, 0.0f };
    double gyroSum[3]     = { 0.0f, 0.0f, 0.0f };

//    mpu6050Calibrating = true;

//    for (samples = 0; samples < 5000; samples++)
    for (samples = 0; samples < 1; samples++)
    {
        //readMPU6050();
    	computeMPU6050TCBias();
        pImu->GetData(&ImuData);

        accelSum[0] += ImuData.accel_x - accelTCBias[0];
        accelSum[1] += ImuData.accel_y - accelTCBias[1];
        accelSum[2] += ImuData.accel_z - accelTCBias[2];

        gyroSum[0]  += ImuData.gyro_r  - gyroTCBias[0];
        gyroSum[1]  += ImuData.gyro_p  - gyroTCBias[1];
        gyroSum[2]  += ImuData.gyro_y  - gyroTCBias[2];

        //delayMicroseconds(500);
        osDelay(1000);
    }

    for (axis = 0; axis < 3; axis++)
    {
			//對之前積分的5000次加速度數據的結果求平均后然後乘以最小刻度值(1/8192) * 9.8065
    	//gyroSum[axis]   = accelSum[axis] / 50.0f * ACCEL_SCALE_FACTOR;
    	gyroSum[axis]   = accelSum[axis] / 50.0f;


        gyroRTBias[axis] = gyroSum[axis]  / 50.0f;
    }
    printf("gyroRTBias[0]=%f, gyroRTBias=%f, gyroRTBias[2]=%f, \r\n",gyroRTBias[0],gyroRTBias[1],gyroRTBias[2]);

    accelOneG = sqrt(SQR(accelSum[XAXIS]) + SQR(accelSum[YAXIS]) + SQR(accelSum[ZAXIS]));

//    mpu6050Calibrating = false;
}
void Angle_Update(float gx,float gy,float gz,float ax,float ay,float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    norm = sqrt((ax*ax) + (ay*ay) + (az*az));
    ax = accel500Hz[0] / norm;
    ay = accel500Hz[1] / norm;
    az = accel500Hz[2] / norm;

    vx = 2*(q_1*q_3 - q_0*q_2);
    vy = 2*(q_0*q_1 + q_2*q_3);
    vz = q_0*q_0 - q_1*q_1 - q_2*q_2 + q_3*q_3;


    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;

    if(fabs(ex)<= 0.01)
    exInt = 0;
    if(fabs(ey)<= 0.01)
    eyInt = 0;
    if(fabs(ez)<= 0.01)
    ezInt = 0;

    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    q_0 = q_0 + (-q_1*gx - q_2*gy - q_3*gz)*halfT;
    q_1 = q_1 + (q_0*gx + q_2*gz - q_3*gy)*halfT;
    q_2 = q_2 + (q_0*gy - q_1*gz + q_3*gx)*halfT;
    q_3 = q_3 + (q_0*gz + q_1*gy - q_2*gx)*halfT;

    norm = sqrt(q_0*q_0 + q_1*q_1 + q_2*q_2 + q_3*q_3);
    q_0 = q_0 / norm;
    q_1 = q_1 / norm;
    q_2 = q_2 / norm;
    q_3 = q_3 / norm;

    P  = asin(-2 * q_1 * q_3 + 2 * q_0* q_2)* 57.3; // pitch
    R = atan2(2 * q_2 * q_3 + 2 * q_0 * q_1, -2 * q_1 * q_1 - 2 * q_2* q_2 + 1)* 57.3; // rollv
//	        R = normalizeAngle180(R);
    Y = atan2(2*(q_1*q_2 + q_0*q_3),q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3) * 57.3;

    printf("Pitch = %f,", P);
    printf("Roll= %f ,", R);
    printf("Yaw= %f\r\n", Y);
}
void initOrientation()
{
    int initLoops = 150;
    float accAngle[3] = { 0.0f, 0.0f, 0.0f };
    int i;
    IMU_STRUCT* pImu = GetImuStr_Addr();

    for (i = 0; i < initLoops; i++)
    {
       // readMPU6050();//從MPU6050得到加速度和陀螺儀數據，並進行 與初始化方位估計矩陣（根據IMU單元的方位確定的矩陣A ） 相乘后的數據
    	pImu->GetData(&ImuData);

        computeMPU6050TCBias();//	計算溫度補償偏差值

			//（矩陣相乘后的加速度數據-溫度補償偏差）* （(1/8192) * 9.8065）
		//(1/8192) * 9.8065  (8192 LSB = 1 G)
		 //1G量程的8192個數字量分之1，對應重力加速度9.8065m/1G的8192分之1
        accel500Hz[0] = (ImuData.accel_x - accelTCBias[0]);// * ACCEL_SCALE_FACTOR;
        accel500Hz[1] = (ImuData.accel_y - accelTCBias[1]);// * ACCEL_SCALE_FACTOR;
        accel500Hz[2] = -(ImuData.accel_z - accelTCBias[2]);// * ACCEL_SCALE_FACTOR;

			//進行歐拉角積分運算
        accAngle[0]  += atan2f(-accel500Hz[1], -accel500Hz[2]);//Roll = Y & Z
        accAngle[1] += atan2f(accel500Hz[0], -accel500Hz[2]); // Pitch = X &Z

			//求取歐拉角算數平均值
        accAngleSmooth[0 ] = accAngle[0 ] / (float)initLoops;
        accAngleSmooth[1] = accAngle[1] / (float)initLoops;

        osDelay(2);
    }

		//得到目前方位 ,初始化一次，不要振動雲臺，因為這裡只用了加速度數據計算歐拉角（加速度數據是長期可信的），但是加速度計對
		//振動很敏感，所以?了減小誤差，初始化方位的時候不要振動雲臺。
    evvgcCFAttitude500Hz[0] = accAngleSmooth[0];
    evvgcCFAttitude500Hz[1 ] = accAngleSmooth[1];
    evvgcCFAttitude500Hz[2  ] = 0.0f;
    printf("accAngleSmooth[0]=%f, accAngleSmooth[1]=%f \r\n", accAngleSmooth[0]*57.3, accAngleSmooth[1]*57.3);
}
//此函式是???位估計翄核心函弿
void getOrientation(float *smoothAcc, float *orient, float *accData, float *gyroData, float dt)
{
    float accAngle[3];
    float gyroRate[3];

	//通過使用atan2f函式計算加速度數據得到歐拉角 滾轉角和 俯仰角。
    accAngle[0] = atan2f(-accData[1], -accData[2]); //Roll = Y & Z
    accAngle[1] = atan2f(accData[0], -accData[2]); //Pitch = X & Z
//printf("accAngle[0]=%f, accAngle[1]=%f, ", accAngle[0]*57.3,accAngle[1]*57.3);

	//其中 smoothAcc 是通過加速度數據經過atan2f函式計算得來的歐拉角，並且進行了一階滯後濾波
	//（此濾波演算法也屬於低通濾波的一種），優點： 對週期性干擾具有良好的抑制作用 適用於波動頻率較高的場合,
	// 缺點： 相位滯後，靈敏度低 滯後程度取決於a值大小， 不能消除濾波頻率高於採樣頻率的1/2的干擾訊號,程式碼中a的值是99.0f
    smoothAcc[0]  = ((smoothAcc[0 ] * 70.0f) + accAngle[0 ]*30.0f) / 100.0f;
    smoothAcc[1] = ((smoothAcc[1] *70.0f) + accAngle[1]*30.0f) / 100.0f;
//printf("smoothAcc[0]=%f, smoothAcc[1]=%f, gyroData[1]=%f ",smoothAcc[0]*57.3, smoothAcc[1]*57.3, gyroData[1]*57.3);

    gyroRate[1] =  gyroData[1];
	//通過互補濾波來融合根據加速度和陀螺儀計算出來的角度，orient[PITCH]是上次融合后的角度，gyroRate[PITCH] * dt是根據陀螺儀
	//數據計算得到的角度（角速度*持續時間結果就是弧度，弧度和角度很容易的可以相互轉換），為什麼要進行數據融合？
	//答：加速度計和陀螺儀都能計算出姿態，但為何要對它們融合，是因為加速度計對振動之類的擾動很敏感，但長期數據計算出的姿態可信，
	//而陀螺儀雖然對振動這些不敏感，但長期使用陀螺儀會出現漂移，因此我們要進行互補，短期相信陀螺儀，長期相信加速度計.
	//先通過加速度計得到的角度減去上一次融合后的角度然後乘以一個比例係數，這個比例係數越小，融合的加速度計的數據比重越小，
	//短期相信陀螺儀，所以陀螺儀的比重這裡是1，長期相信加速度計，加速度計的數據用來修正陀螺儀的漂移產生的誤差，
	//這樣對陀螺儀的漂移進行了修正，有效地抑制了加速度計和陀螺儀各自單獨工作時候的偏差.
//    orient[1]   = (orient[1] + gyroRate[1] * dt) + 0.0002f * (smoothAcc[1] - orient[1]);
    orient[1]   = (orient[1] + gyroRate[1] * dt) + 0.02f * (smoothAcc[1] - orient[1]);

		//可以用正弦或餘弦和x軸單獨算出角度,因為我們知道重力的大小
	 //但是IMU單元必須是靜止水平狀態
    gyroRate[0]  =  gyroData[0] * cosf(fabsf(orient[1])) + gyroData[2] * sinf(orient[1]);

	//通過互補濾波來融合根據加速度和陀螺儀計算出來的角度，orient[PITCH]是上次融合后的角度，gyroRate[PITCH] * dt是根據陀螺儀
	//數據計算得到的角度（角速度*持續時間結果就是弧度，弧度和角度很容易的可以相互轉換），為什麼要進行數據融合？
	//答：加速度計和陀螺儀都能計算出姿態，但為何要對它們融合，是因為加速度計對振動之類的擾動很敏感，但長期數據計算出的姿態可信，
	//而陀螺儀雖然對振動這些不敏感，但長期使用陀螺儀會出現漂移，因此我們要進行互補，短期相信陀螺儀，長期相信加速度計.
	//先通過加速度計得到的角度減去上一次融合后的角度然後乘以一個比例係數，這個比例係數越小，融合的加速度計的數據比重越小，
	//短期相信陀螺儀，所以陀螺儀的比重這裡是1，長期相信加速度計，加速度計的數據用來修正陀螺儀的漂移產生的誤差，
	//這樣對陀螺儀的漂移進行了修正，有效地抑制了加速度計和陀螺儀各自單獨工作時候的偏差.
    orient[0]    = (orient[0] + gyroRate[0] * dt) + 0.02f * (smoothAcc[0] - orient[0]);

		//可以用正弦或餘弦和x軸單獨算出角度,因為我們知道重力的大小
	 //但是IMU單元必須是靜止水平狀態
    gyroRate[2]   =  gyroData[2] * cosf(fabsf(orient[1])) - gyroData[0] * sinf(orient[1]);

    orient[2]     = (orient[2] + gyroRate[2] * dt);//對鿿??????迌翍忆志到???航角YAW
    printf("orient[0]=%f, orient[1]=%f, orient[2]=%f, \r\n", orient[0]*57.3, orient[1]*57.3, orient[2]*57.3);
}
// Magnetometer define

#if magneto

#define MagnetcDeclination 4.43

int calculateHeading(short x ,short y,short z, short *offsetX, short *offsetY)
{
  float headingRadians = atan2((double)((y)-*offsetY),(double)((x)-*offsetX));
  //保迉數???在0-2*PI之鿿
  if(headingRadians < 0)
    headingRadians += 2*PI;

  int headingDegrees = headingRadians * 57.3;
  	  headingDegrees += MagnetcDeclination; //磁忏迿

  // <span style="font-family: Arial, Helvetica, sans-serif; ??>保迉數???在0-360之鿿</span>
  if(headingDegrees > 360)
    headingDegrees -= 360;

  return headingDegrees;
}
#endif
/* USER CODE END FunctionPrototypes */

void StartGimbalTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of GimbalCore */
  osThreadDef(GimbalCore, StartGimbalTask, osPriorityNormal, 0, 256);
  GimbalCoreHandle = osThreadCreate(osThread(GimbalCore), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
  * @brief  Function implementing the GimbalCore thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE END Header_StartGimbalTask */
void StartGimbalTask(void const * argument)
{
  /* USER CODE BEGIN StartGimbalTask */

	uint32_t imuOldTick = xTaskGetTickCount();
	uint8_t id;
	int16_t mx, my, mz;

	IMU_STRUCT* pImu = GetImuStr_Addr();
	HMC5883_STRUCT* pHMC5883 = GetHMC5883Str_Addr();

	int result;
	result = mpu_init();
	printf("result = %d \r\n", result);

	pImu->Init();
	pHMC5883->Init();

	orientIMU();
	MPU6050_Calibration();
	computeMPU6050RTData();

//  initFirstOrderFilter();
	initOrientation();
	computeMPU6050TCBias();

  /* Infinite loop */
	float yaw=0;
	short offsetX ,offsetY , offsetZ;

  for(;;)
  {
	  if(xTaskGetTickCount() - imuOldTick >= 2)
	  {
		  taskENTER_CRITICAL();

		  pImu->GetData(&ImuData);
		  pHMC5883->GetData(&HMC5883Data);

		  printf("mx=%d, my=%d, mz=%d, \r\n", HMC5883Data.mag_x, HMC5883Data.mag_y, HMC5883Data.mag_z);

		  dt500Hz = (xTaskGetTickCount() - imuOldTick)/configTICK_RATE_HZ;

		  accel500Hz[0] = (ImuData.accel_x - accelTCBias[0]);//*ACCEL_SCALE_FACTOR;
		  accel500Hz[1] = (ImuData.accel_y - accelTCBias[1]);//;*ACCEL_SCALE_FACTOR;
		  accel500Hz[2] = -(ImuData.accel_z - accelTCBias[2]);
		  gyro500Hz[0] = (ImuData.gyro_r - gyroRTBias[0] - gyroTCBias[0]);//*GYRO_SCALE_FACTOR;
	      gyro500Hz[1] = (ImuData.gyro_p - gyroRTBias[1] - gyroTCBias[1]);//*GYRO_SCALE_FACTOR;
		  gyro500Hz[2] = -(ImuData.gyro_y - gyroRTBias[2] - gyroTCBias[2]);//*GYRO_SCALE_FACTOR;

			//目前方位估計運算，其中 accAngleSmooth 是通過加速度數據經過atan2f函式計算得來的歐拉角，並且進行了一階滯後濾波
			//（此濾波演算法也屬於低通濾波的一種），優點： 對週期性干擾具有良好的抑制作用 適用於波動頻率較高的場合,
			// 缺點： 相位滯後，靈敏度低 滯後程度取決於a值大小 不能消除濾波頻率高於採樣頻率的1/2的干擾訊號,
			//getOrientation函式內部使用了accAngleSmooth歐拉角與陀螺儀數據進行了互補濾波融合演算法得到穩定的歐拉角，並且
			//存放到sensors.evvgcCFAttitude500Hz裡面，sensors.accel500Hz和sensors.gyro500Hz是經過上面演算法處理后的加速度數據
			//和陀螺儀數據，dt500Hz是時間增量，也就是執行if (frame_500Hz){} 裡面的程式碼間隔
		  getOrientation(accAngleSmooth, evvgcCFAttitude500Hz, accel500Hz, gyro500Hz, dt500Hz);

/*		  printf("Roll=%f, Pitch=%f, Yaw=%f \r\n",gyro500Hz[0], gyro500Hz[1], gyro500Hz[2]);
		  accel500Hz[0] = firstOrderFilter(accel500Hz[0], &firstOrderFilters[ACCEL_X_500HZ_LOWPASS]);
		  accel500Hz[1] = firstOrderFilter(accel500Hz[1], &firstOrderFilters[ACCEL_Y_500HZ_LOWPASS]);
		  accel500Hz[2] = firstOrderFilter(accel500Hz[2], &firstOrderFilters[ACCEL_Z_500HZ_LOWPASS]);

		  //航姿參考系統更新，入口參數是三軸陀螺儀數據，三軸加速度數據，三軸磁力計數據,以及指示是否更新磁力計數據的magDataUpdate參數，
			//magDataUpdate=false表示不更新磁力計數據，magDataUpdate=true表示更新磁力計數據，最後一個參數是時間增量Dt，也就是此函式本
			//次執行與上次執行的時間間隔。
		  MargAHRSupdate(gyro500Hz[0], gyro500Hz[1], gyro500Hz[2], accel500Hz[0], accel500Hz[1], accel500Hz[2],1.0, 1.0, 1.0, false, dt500Hz);
*/
		  MargAHRSupdate(gyro500Hz[0], gyro500Hz[1], gyro500Hz[2], accel500Hz[0], accel500Hz[1], accel500Hz[2],HMC5883Data.mag_x, HMC5883Data.mag_y, HMC5883Data.mag_z, true, dt500Hz);

//		  float pitch = atan2(accel500Hz[1], sqrt(accel500Hz[0] * accel500Hz[0] + accel500Hz[2] * accel500Hz[2])) * 180.0 / PI;
//		  float roll = atan2(-accel500Hz[0], accel500Hz[2]) * 180.0 / PI;
//		  yaw +=  gyro500Hz[2] * dt500Hz;
//		  printf("p=%f, r=%f, y=%f, \r\n", pitch, roll,yaw);

		  //test quarant
//		  Angle_Update(gyro500Hz[0],gyro500Hz[1],gyro500Hz[2],accel500Hz[0],accel500Hz[1],accel500Hz[2]);
//		  Angle_Update(ImuData.gyro_r,ImuData.gyro_p,ImuData.gyro_y,ImuData.accel_x,ImuData.accel_y,ImuData.accel_z);

/*		    float accAngle_cal_0 = atan2f(-accel500Hz[1], -accel500Hz[2]); //Roll = Y & Z
		    float accAngle_cal_1 = atan2f(accel500Hz[0], -accel500Hz[2]); //Pitch = X & Z
		    printf("accAngle_cal_Roll=%f, accAngle_cal_Pitch=%f, ", accAngle_cal_0 * 57.3, accAngle_cal_1 * 57.3);

		    float accAngle_uncal_0 = atan2f(-ImuData.accel_y, ImuData.accel_z); //Roll = Y & Z
		    float accAngle_uncal_1 = atan2f(ImuData.accel_x, ImuData.accel_z); //Pitch = X & Z
		    printf("accAngle_uncal_Roll=%f, accAngle_uncal_Pitch=%f, \r\n", accAngle_uncal_0*57.3, accAngle_uncal_1*57.3);
*/
		  imuOldTick = xTaskGetTickCount();
		  taskEXIT_CRITICAL();
	  }

	//osDelay(2);
  }
  /* USER CODE END StartGimbalTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
