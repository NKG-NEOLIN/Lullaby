/*
 * iHMC5883.h
 *
 *  Created on: May 19, 2025
 *      Author: nino
 */

#ifndef INC_IHMC5883_H_
#define INC_IHMC5883_H_

#include "iType.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct{
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
}HMC5883_DATA_STRUCT, *pHMC5883_DATA_STRUCT;

typedef struct{
                Result (*Init)();
				Result (*GetDevID)(uint8_t *Data);
                Result (*GetData)(HMC5883_DATA_STRUCT *Data);
              }HMC5883_STRUCT, *pHMC5883_STRUCT;

extern pHMC5883_STRUCT GetHMC5883Str_Addr();



#ifdef __cplusplus
}
#endif // extern "C"

#endif /* INC_IHMC5883_H_ */
