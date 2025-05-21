#include "iImu.h"
#include "stdint.h"
#include "MPU6050/mpu6050.h"

static IMU_STRUCT        ImuExternData;

Result ImuInit()
{
//	if (!MPU6050_testConnection()) {
//		return RESULT_FAILURE;
//	}
	MPU6050_initialize();
//#if IMU_ENABLE_DMP
//	DMP_Init();
//#endif
  return RESULT_SUCCESS;
}

Result GetImuDevID(uint8_t* devID)
{
	*devID = MPU6050_getDeviceID();
	return RESULT_SUCCESS;
}

Result GetImuData(IMU_DATA_STRUCT *Data)
{
	if (!MPU6050_testConnection()) {
		return RESULT_FAILURE;
	}
	Read_NativeData(
		&Data->accel_x,
		&Data->accel_y,
		&Data->accel_z,
		&Data->gyro_r,
		&Data->gyro_p,
		&Data->gyro_y
	);
#if IMU_ENABLE_DMP
	Read_DMPdirect(
			&Data->q0,
			&Data->q1,
			&Data->q2,
			&Data->q3,
			&Data->roll,
			&Data->pitch,
			&Data->yaw
	);
#endif
	return RESULT_SUCCESS;
}
Result GetImuTemp(IMU_TEMP_STRUCT *Data)
{
	int temp;
	if (!MPU6050_testConnection()) {
		return RESULT_FAILURE;
	}
	temp = Read_Temperature();
	Data->temperature = temp;
}
pIMU_STRUCT GetImuStr_Addr()
{
  ImuExternData.Init = &ImuInit;	
  ImuExternData.GetDevID = &GetImuDevID;
	ImuExternData.GetData = &GetImuData;
  return &ImuExternData;	
}
