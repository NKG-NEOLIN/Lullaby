/*
 * iHMC5883.c
 *
 *  Created on: May 19, 2025
 *      Author: nino
 */

#include "iHMC5883.h"
#include "stm32f4xx_hal.h"

static HMC5883_STRUCT        HMC5883ExternData;


extern I2C_HandleTypeDef hi2c1;

#define HMC5883L_I2C_ADDR        (0x1E << 1) // 7-bit 位址左移一位
#define HMC5883L_I2C_ADDR_WRITE	 0x3C	 // 8-bit write address
#define HMC5883L_I2C_ADDR_READ	 0x3D	 //8-bit read address

#define HMC5883L_REG_CONFIG_A    0x00
#define HMC5883L_REG_CONFIG_B    0x01
#define HMC5883L_REG_MODE        0x02
#define HMC5883L_REG_DATA_X_MSB  0x03
#define HMC5883L_REG_DATA_X_LSB  0x04
#define HMC5883L_REG_DATA_Z_MSB  0x05
#define HMC5883L_REG_DATA_Z_LSB  0x06
#define HMC5883L_REG_DATA_Y_MSB  0x07
#define HMC5883L_REG_DATA_Y_LSB  0x08
#define HMC5883L_REG_STATUS      0x09
#define HMC5883L_REG_IDENT_A     0x0A
#define HMC5883L_REG_IDENT_B     0x0B
#define HMC5883L_REG_IDENT_C     0x0C
// HMC5883L 配置數值
#define HMC5883L_CONFIG_A_VALUE  0x70//(0b01110000) // 8 平均, 15 Hz 輸出, 正常測量
#define HMC5883L_CONFIG_B_VALUE  0x40//(0b00100000) // 增益 +/- 1.3 高斯
#define HMC5883L_MODE_CONTINUOUS 0x00//(0b00000000)
#define HMC5883L_MODE_SINGLE     0x01       // 單次測量模式
#define STATUS_RDY      		 0x01 // Data Ready
// I2C 超時時間 (毫秒)
#define HMC5883L_I2C_TIMEOUT     300
#define HMC5883L_CONTINUOUS_DELAY 70 // 略大於 67ms
#define XAXIS    0
#define YAXIS    1
#define ZAXIS    2

///////// Calibration ///////////////////
// if Xsf>1 Xsf=1; else Xsf = (Ymax-Ymin)/(Xmax-Xmin);
// if Ysf>1 Ysf=1; else Ysf = (Xmax-min)/(Ymax-Ymin);
// X_bias = ((Xmax-Xmin)/2-Xmax) x Xsf
// Y_bias = ((Ymax-Ymin)/2-Ymax_ x Ysf
#define x_bias   -73
#define y_bias   -163

Result HMC5883_Init()
{
	HAL_StatusTypeDef status;
	int16_t mag_x, mag_y, mag_z;
	uint8_t data[2];
	data[0] = 0x00;		// Register A
	data[1] = 0x70;    // 8-average, 15Hz data output, normal measurement
	status = HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_I2C_ADDR_WRITE, data, 2, HAL_MAX_DELAY); // Write-mode

	data[0] = 0x01; 	// Register B
	data[1] = 0x20; 	// Gain=1.3
	status = HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_I2C_ADDR_WRITE, data, 2, HAL_MAX_DELAY); // Write-mode

/*	for(int i=0;i<10;i++)
	{
    	data[0] = 0x02;     // Select Mode Regiater
    	data[1] = 0x01;    // MD1 | MD0 01 = single-measeurement mode,
		status = HAL_I2C_Master_Transmit(&hi2c1, 0x3C, data, 2, HAL_MAX_DELAY); // Write-mode SET single mode
		HAL_Delay(20);

		HMC5883L_ReadData(&mag_x, &mag_y, &mag_z);
		magScaleFactor[0] += (1.16 * 1090 ) / (float)mag_x;
		magScaleFactor[1] += (1.16 * 1090 ) / (float)mag_y;
		magScaleFactor[2] += (1.16 * 1090 ) / (float)mag_z;
		printf("magScaleFactor[0]=%f, magScaleFactor[1]=%f, magScaleFactor[2]=%f, \r\n", magScaleFactor[0], magScaleFactor[1], magScaleFactor[2]);
	}

	magScaleFactor[0] = fabs(magScaleFactor[0]/10);
	magScaleFactor[1] = fabs(magScaleFactor[1]/10);
	magScaleFactor[2] = fabs(magScaleFactor[2]/10);
	printf("magScaleFactor[0]=%f, magScaleFactor[1]=%f, magScaleFactor[2]=%f, \r\n", magScaleFactor[0], magScaleFactor[1], magScaleFactor[2]);
*/
	data[0] = 0x02;     // Select Mode Regiater
	data[1] = 0x00;    // MD1 | MD0 01 = continuous-measeurement mode,
	status = HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_I2C_ADDR_WRITE, data, 2, HAL_MAX_DELAY); // Write-mode SET continuous mode
	HAL_Delay(20);

	uint8_t buffer[6];
	HAL_I2C_Mem_Read(&hi2c1, HMC5883L_I2C_ADDR_READ, HMC5883L_REG_DATA_X_MSB, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);
	mag_x = (((int16_t)buffer[0]) << 8) | buffer[1];
	mag_y = (((int16_t)buffer[4]) << 8) | buffer[5];
	mag_z = (((int16_t)buffer[2]) << 8) | buffer[3];

	uint8_t address;
	for (address = 1; address < 128; address++) {
		HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1), 10, 10);
		if (res == HAL_OK) {
			printf("Found device at 0x%02X (7-bit address)\r\n", address);
		}
	}

	return RESULT_SUCCESS;
}

Result GetHMC5883L_ReadData(HMC5883_DATA_STRUCT *Data)
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_I2C_ADDR_READ, HMC5883L_REG_DATA_X_MSB, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);

    Data->mag_x = ((((int16_t)buffer[0]) << 8) | buffer[1]) - x_bias; //X_corrected = X_raw - X_bias
    Data->mag_y = ((((int16_t)buffer[4]) << 8) | buffer[5]) - y_bias; //Y_corrected = Y_raw - Y_bias
    Data->mag_z = (((int16_t)buffer[2]) << 8) | buffer[3];

//    Data->mag_x = (((((int16_t)buffer[0]) << 8) | buffer[1])/1090) - X_bias; //X_corrected = X_raw - X_bias
//    Data->mag_y = (((((int16_t)buffer[4]) << 8) | buffer[5])/1090) - y_bias; //Y_corrected = Y_raw - Y_bias
//    Data->mag_z = ((((int16_t)buffer[2]) << 8) | buffer[3])/1090;

    return RESULT_SUCCESS;
}
Result GetDevID(uint8_t *Data)
{
	uint8_t address;
	for (address = 1; address < 128; address++) {
		HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1), 10, 10);
		if (res == HAL_OK) {
			*Data = address;
		}
	}
}
pHMC5883_STRUCT GetHMC5883Str_Addr()
{
	HMC5883ExternData.Init = &HMC5883_Init;
	HMC5883ExternData.GetDevID = &GetDevID;
	HMC5883ExternData.GetData = &GetHMC5883L_ReadData;

	return &HMC5883ExternData;
}
