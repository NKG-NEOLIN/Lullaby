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
 * @defgroup iDrive_Config
 * @ingroup iDrive_Config
 * 
 */ 

#ifndef _I_DRIVE_CONFIG_H_
#define _I_DRIVE_CONFIG_H_

/**--------------------------------------------------------------------------------------------------
* Setup the setting of robot power drive system 
*
* Setting value including: 
*
* QEP resolution
* 
* Maximum speed at a motor in maximum efficency region
* 
* QEP resolution
* 
* Position of motor in robot chassis 
*
* Gear ratio from drive motor to drive wheel, ..etc
*  
------------------------------------------------------------------------------------------------**/
/**
 *@def DEBUG_Drive   
 * @ingroup Drive_Config
 *DEBUG_Drive Describe it used to switch descriptive sentence output
 */ 
#define DEBUG_Drive  		0

/**
 *@def MAX_RPM   
 * @ingroup Drive_Config
 *MAX_RPM Describe motor's maximum RPM
 */ 
#define MAX_RPM         366 

/**
 *@def COUNTS_PER_REV   
 * @ingroup Drive_Config
 *COUNTS_PER_REV Describe wheel encoder's number of ticks per revolution(gear_ratio * pulse_per_rev)
 */ 
#define COUNTS_PER_REV  1560 

/**
 *@def WHEEL_DIAMETER   
 * @ingroup Drive_Config
 *WHEEL_DIAMETER Describe wheel's diameter<m> 
 */ 
#define WHEEL_DIAMETER  0.07 

/**
 *@def LR_WHEELS_DISTANCE   
 * @ingroup Drive_Config
 *LR_WHEELS_DISTANCE Describe disatnce<m> between with  left and right wheels
 */
#define LR_WHEELS_DISTANCE 0.238

/**
 *@def FR_WHEELS_DISTANCE   
 * @ingroup Drive_Config
 *LR_WHEELS_DISTANCE Describe disatnce<m> between with front and rear wheels
 */
#define FR_WHEELS_DISTANCE 0.16

/**
 *@def GEAR_RATIO   
 * @ingroup Drive_Config
 *GEAR_RATIO Describe motor's reduction rate  Gear : 1
 */
#define GEAR_RATIO         65.56

/**
 *@def IMU_PUBLISH_HZ   
 * @ingroup Drive_Config
 *IMU_PUBLISH_HZ Describe IMU data upload frequency<Hz> 
 */
#define IMU_PUBLISH_HZ		 1000



/**
 *@def Timer_parameter   
 * @ingroup Drive_Config
 *Timer_parameter Describe Timer frequency division<Hz> 
 */
#define PRESCALE	 	 34
#define PERIOD       254

/**
 *@def PID Controller Config   
 * @ingroup Drive_Config
 *PID Controller Describe Parameter definition 
 */
#define KP    0.09
#define KI    0.2
#define KD    0.2

/**
 *@def VACUUM Controller Config   
 * @ingroup Drive_Config
 *VACUUM Controller switch definition 
 */
#define VACUUM_EN			0

//--------------------------------------------------------------------------------------------------
//Math
#define PI 							3.1415926
//--------------------------------------------------------------------------------------------------
#endif //ICONFIG_H


