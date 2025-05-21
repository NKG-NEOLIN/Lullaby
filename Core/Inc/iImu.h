/*!
 * @date
 *   2021/03/15
 *
 * @version
 *   1.0.0
 *
 * @author
 *   NinoLiu
 *
 * @file iDrive_Config.h
 *
 * @defgroup Config
 * @defgroup iImu_Config
 * @ingroup iImu_Config
 *
 */

#include "iType.h"
#include "MPU6050/mpu6050.h"
#include "stdint.h"

#ifndef IIMU_H
#define IIMU_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{                
								float accel_x;
								float accel_y;
								float accel_z;
								float gyro_r;
								float gyro_p;
								float gyro_y;
								
#if IMU_ENABLE_DMP
								float q0;
								float q1;
								float q2;
								float q3;
								float roll;
								float pitch;
								float yaw;
#endif
								
}IMU_DATA_STRUCT, *pIMU_DATA_STRUCT;


typedef struct{
	 float temperature;
}IMU_TEMP_STRUCT;

typedef struct{
                Result (*Init)();
				Result (*GetDevID)(uint8_t *Data);
                Result (*GetData)(IMU_DATA_STRUCT *Data);
                Result (*GetImuTemp)(IMU_TEMP_STRUCT *Data);
              }IMU_STRUCT, *pIMU_STRUCT;

extern pIMU_STRUCT GetImuStr_Addr();							
	
#ifdef __cplusplus
}
#endif // extern "C" 

#endif /* IIMU_H */
