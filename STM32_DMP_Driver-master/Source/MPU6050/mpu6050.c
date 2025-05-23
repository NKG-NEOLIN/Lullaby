#include "MPU6050/inv_mpu.h"
#include "MPU6050/inv_mpu_dmp_motion_driver.h"
#include "MPU6050/I2C.h"
#include "MPU6050/mpu6050.h"
#include "string.h" //for reset buffer

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f

short gyro[3], accel[3], sensors;
float Pitch,Roll,Yaw;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static signed char gyro_orientation[9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };

///////////////////////////////////////////////////////////////////////////////
// Orient IMU
///////////////////////////////////////////////////////////////////////////////
int16_t orientationMatrix[9];
int16_t straightAccelData[3];
int16_t rotatedAccelData[3];
int16_t straightGyroData[3];
int16_t rotatedGyroData[3];
void orientIMU(void) //根據IMU單元的方位確定矩陣A的值
{
    switch (1)  //eepromConfig.imuOrientation==4
    {
        case 1: // Dot Front/Left/Top
            orientationMatrix[0] =  1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] =  1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 2: // Dot Front/Right/Top
            orientationMatrix[0] =  0;
            orientationMatrix[1] = -1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 3: // Dot Back/Right/Top
            orientationMatrix[0] = -1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] = -1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 4: // Dot Back/Left/Top
            orientationMatrix[0] =  0;
            orientationMatrix[1] =  1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] = -1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 5: // Dot Front/Left/Bottom
            orientationMatrix[0] =  0;
            orientationMatrix[1] = -1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] = -1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        case 6: // Dot Front/Right/Bottom
            orientationMatrix[0] =  1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] = -1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        case 7: // Dot Back/Right/Bottom
            orientationMatrix[0] =  0;
            orientationMatrix[1] =  1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        case 8: // Dot Back/Left/Bottom
            orientationMatrix[0] = -1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] =  1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        default: // Dot Front/Left/Top
            orientationMatrix[0] =  1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] =  1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;
    }
}

static unsigned short inv_row_2_scale(const signed char *row) {
  unsigned short b;

  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7;      // error
  return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {
  unsigned short scalar;
  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;

  return scalar;
}

static void run_self_test(void) {
  int result;
  long gyro[3], accel[3];

  result = mpu_run_self_test(gyro, accel);
  if (result == 0x7) {
    /* Test passed. We can trust the gyro data here, so let's push it down
     * to the DMP.
     */
    float sens;
    unsigned short accel_sens;
    mpu_get_gyro_sens(&sens);
    gyro[0] = (long) (gyro[0] * sens);
    gyro[1] = (long) (gyro[1] * sens);
    gyro[2] = (long) (gyro[2] * sens);
    dmp_set_gyro_bias(gyro);
    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel);
//    log_i("setting bias succesfully ......\r\n");
  }
}

uint8_t buffer[14];

int16_t MPU6050_FIFO[6][11];
int16_t Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setClockSource(uint8_t source)
 *功　　能:	    设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 *******************************************************************************/
void MPU6050_setClockSource(uint8_t source) {
  IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
  MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_Set_DLPF(uint8_t dlpf_bandwidth)
 *功　　能:	    set MPU6050 low pass filter bandwidth
 * DLPF_BW | Bandwidth
 * --------+--------------------------------------
 * 0       | MPU6050_DLPF_BW_256
 * 1       | MPU6050_DLPF_BW_188
 * 2       | MPU6050_DLPF_BW_98
 * 3       | MPU6050_DLPF_BW_42
 * 4       | MPU6050_DLPF_BW_20
 * 5       | MPU6050_DLPF_BW_10
 * 6       | MPU6050_DLPF_BW_5
 *******************************************************************************/
void MPU6050_Set_DLPF(uint8_t dlpf_bandwidth)
{
	IICwriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, dlpf_bandwidth);
}



/**************************实现函数********************************************
 // *函数原型:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
 // *功　　能:	    将新的ADC数据更新到 FIFO数组，进行滤波处理
 // *******************************************************************************/
void MPU6050_newValues(int16_t ax, int16_t ay, int16_t az, int16_t gx,
    int16_t gy, int16_t gz) {
  unsigned char i;
  int32_t sum = 0;
  for (i = 1; i < 10; i++) {	//FIFO 操作
    MPU6050_FIFO[0][i - 1] = MPU6050_FIFO[0][i];
    MPU6050_FIFO[1][i - 1] = MPU6050_FIFO[1][i];
    MPU6050_FIFO[2][i - 1] = MPU6050_FIFO[2][i];
    MPU6050_FIFO[3][i - 1] = MPU6050_FIFO[3][i];
    MPU6050_FIFO[4][i - 1] = MPU6050_FIFO[4][i];
    MPU6050_FIFO[5][i - 1] = MPU6050_FIFO[5][i];
  }
  MPU6050_FIFO[0][9] = ax;	//将新的数据放置到 数据的最后面
  MPU6050_FIFO[1][9] = ay;
  MPU6050_FIFO[2][9] = az;
  MPU6050_FIFO[3][9] = gx;
  MPU6050_FIFO[4][9] = gy;
  MPU6050_FIFO[5][9] = gz;

  sum = 0;
  for (i = 0; i < 10; i++) {	//求当前数组的合，再取平均值
    sum += MPU6050_FIFO[0][i];
  }
  MPU6050_FIFO[0][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[1][i];
  }
  MPU6050_FIFO[1][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[2][i];
  }
  MPU6050_FIFO[2][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[3][i];
  }
  MPU6050_FIFO[3][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[4][i];
  }
  MPU6050_FIFO[4][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[5][i];
  }
  MPU6050_FIFO[5][10] = sum / 10;
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
  IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
  MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
 *功　　能:	    设置  MPU6050 加速度计的最大量程
 *******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
  IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
  MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
 *功　　能:	    设置  MPU6050 是否进入睡眠模式
 enabled =1   睡觉
 enabled =0   工作
 *******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
 *函数原型:		uint8_t MPU6050_getDeviceID(void)
 *功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
 *******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {
  memset(buffer,0,sizeof(buffer));
  i2c_read(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
  return buffer[0];
}

/**************************实现函数********************************************
 *函数原型:		uint8_t MPU6050_testConnection(void)
 *功　　能:	    检测MPU6050 是否已经连接
 *******************************************************************************/
uint8_t MPU6050_testConnection(void) {
  if (MPU6050_getDeviceID() == 0x68)  //0b01101000;
    return 1;
  else
    return 0;
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
 *功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
 *******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT,
      enabled);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
 *功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
 *******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT,
      enabled);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_initialize(void)
 *功　　能:	    初始化 	MPU6050 以进入可用状态。
 *******************************************************************************/
void MPU6050_initialize(void) {
  MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟
  MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_250); //陀螺仪最大量程 +-1000度每秒
  MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
  MPU6050_setSleepEnabled(0); //进入工作状态
  MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
  MPU6050_setI2CBypassEnabled(0);	//主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
  DMP_Init();
}

/**************************************************************************
 函数功能：MPU6050内置DMP的初始化
 入口参数：无
 返回  值：无
 作    者：平衡小车之家
 **************************************************************************/
void DMP_Init(void)
{
  if (MPU6050_getDeviceID() != 0x68)
    NVIC_SystemReset();
  if (!mpu_init(NULL))
  {
    if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
//      log_i("mpu_set_sensor complete ......\r\n");
    if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
//      log_i("mpu_configure_fifo complete ......\r\n");
    if (!mpu_set_sample_rate(DEFAULT_MPU_HZ))
//      log_i("mpu_set_sample_rate complete ......\r\n");
    if (!dmp_load_motion_driver_firmware())
//      log_i("dmp_load_motion_driver_firmware complete ......\r\n");
    if (!dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation)))
//      log_i("dmp_set_orientation complete ......\r\n");
    if (!dmp_enable_feature(
        DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL
            | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))
//      log_i("dmp_enable_feature complete ......\r\n");
    if (!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
//      log_i("dmp_set_fifo_rate complete ......\r\n");
    	run_self_test();
    if (!mpu_set_dmp_state(0))
      printf("mpu_set_dmp_state complete ......\r\n");
  }
}
/**************************************************************************
 函数功能：读取MPU6050内置DMP的姿态信息
 入口参数：无
 返回  值：无
 作    者：平衡小车之家
 **************************************************************************/
void Read_DMP(void) {
  unsigned long sensor_timestamp;
  unsigned char more;
  long quat[4];

  dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
  if (sensors & INV_WXYZ_QUAT) {
    q0 = quat[0] / q30;
    q1 = quat[1] / q30;
    q2 = quat[2] / q30;
    q3 = quat[3] / q30;
    Pitch = sinf(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
		Roll = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
		Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
  }

}

int Read_DMPdirect(float* q0, float* q1, float* q2, float* q3, float* roll, float* pitch, float* yaw)
{
  unsigned long sensor_timestamp;
  unsigned char more;
  long quat[4];

	float _q0,_q1,_q2,_q3;
  dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
  if (sensors & INV_WXYZ_QUAT) {
    _q0 = quat[0] / q30;
    _q1 = quat[1] / q30;
    _q2 = quat[2] / q30;
    _q3 = quat[3] / q30;
    Pitch = sinf(-2 * _q1 * _q3 + 2 * _q0 * _q2) * 57.3;
		Roll = atan2f(2 * _q2 * _q3 + 2 * _q0 * _q1, -2 * _q1 * _q1 - 2 * _q2* _q2 + 1)* 57.3;
		Yaw = atan2(2*(_q1*_q2 + _q0*_q3),_q0*_q0+_q1*_q1-_q2*_q2-_q3*_q3) * 57.3;

		*q0 = _q0;
		*q1 = _q1;
		*q2 = _q2;
		*q3 = _q3;

		*roll = Roll;
		*pitch = Pitch;
		*yaw = Yaw;

		return 1;
  }
	return 0;
}
/**************************************************************************
 函数功能：读取MPU6050内置温度传感器数据
 入口参数：无
 返回  值：摄氏温度
 作    者：平衡小车之家
 **************************************************************************/
//int Read_Temperature(void) {
float Read_Temperature(void) {
  float Temp;
  uint8_t H, L;
  i2c_read(devAddr, MPU6050_RA_TEMP_OUT_H, 1, &H);
  i2c_read(devAddr, MPU6050_RA_TEMP_OUT_L, 1, &L);
  Temp = (H << 8) + L;
  if (Temp > 32768)
    Temp -= 65536;
//  Temp = (36.53 + Temp / 340) * 10;
  Temp = 36.53 +( Temp / 340);
//  return (int) Temp;
  return Temp;
}
//------------------End of File----------------------------

#define G_TO_ACCEL 9.8065
#define DEG_TO_RAD 0.01745329

void Read_NativeData(float* ax,float* ay,float* az,float* gx,float* gy, float* gz)
{
  uint8_t Rec_Data[6];
  int16_t rawAx,rawAy,rawAz;


  i2c_read(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, (uint8_t*)Rec_Data);
  
  rawAx = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
	rawAy = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
	rawAz = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);

	straightAccelData[0]=rawAx;
	straightAccelData[1]=rawAy;
	straightAccelData[2]=rawAz;
	matrixMultiply(3, 3, 1, rotatedAccelData, orientationMatrix, straightAccelData);

//	*ax = G_TO_ACCEL * rawAx/16384.0;
//	*ay = G_TO_ACCEL * rawAy/16384.0;
//	*az = G_TO_ACCEL * rawAz/16384.0;
	*ax = G_TO_ACCEL * rotatedAccelData[0]/16384.0;
	*ay = G_TO_ACCEL * rotatedAccelData[1]/16384.0;
	*az = G_TO_ACCEL * rotatedAccelData[2]/16384.0;

  i2c_read(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, (uint8_t*)Rec_Data);

  rawAx = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
	rawAy = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
	rawAz = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);

	straightGyroData[0]=rawAx;
	straightGyroData[1]=rawAy;
	straightGyroData[2]=rawAz;
	matrixMultiply(3, 3, 1, rotatedGyroData, orientationMatrix, straightGyroData);

//	*gx = DEG_TO_RAD * rawAx/131.0;
//	*gy = DEG_TO_RAD * rawAy/131.0;
//	*gz = DEG_TO_RAD * rawAz/131.0;
	*gx = DEG_TO_RAD * rotatedGyroData[0]/131.0;
	*gy = DEG_TO_RAD * rotatedGyroData[1]/131.0;
	*gz = DEG_TO_RAD * rotatedGyroData[2]/131.0;
  //HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 100);
}
