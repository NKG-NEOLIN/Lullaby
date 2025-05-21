/*!
 * @date  
 *   2021/07/07
 *
 * @version 
 *   1.0.0
 *
 * @author 
 *   NinoLiu
 *
 * @file iType.h  
 *
 * @defgroup Config
 * @defgroup TypeDefine
 * @ingroup Config
 */
//--------------------------------------------------------------------------------------------------
#ifndef ITYPE_H
#define ITYPE_H
//-------------------------------------------------------------------------------------------------
//Variable whose value is to remain constant during execution. 

#define CONST (const)

#define TRUE true
#define FALSE false

//Boolean variable (should be TRUE or FALSE). 
#define BOOL bool

//VOID
#define VOID void
//Byte (8 bits). 
#define BYTE unsigned char
//Boolean variable (should be TRUE or FALSE). 


//8-bit (ANSI) character. For more information, see Character Sets Used By Fonts. 
#define CHAR char
//16-bit signed integer.
#define SHORT short
//32-bit signed integer. 
#define INT int
//Floating-point variable. 
#define FLOAT float
//Floating-point variable. 
#define DOUBLE double
//32-bit signed integer. 
#define LONG long

/*
//8-bit signed integer. 
#define int8_t signed char
//8-bit unsigned integer. 
#define uint8_t unsigned char
//16-bit signed integer. 
#define int16_t signed short
//16-bit unsigned integer. 
#define uint16_t unsigned short
//32-bit signed integer. 
#define int32_t signed int
//32-bit unsigned integer. 
#define uint32_t unsigned int
//64-bit signed integer. 
#define int64_t signed long long
//64-bit unsigned integer. 
#define uint64_t unsigned long long
*/
#define __IO volatile


//--------------------------------------------------------------------------------------------------
/*
RESULT_SUCCESS 0: Successful result.
RESULT_FAILURE 1: Generic, unspecified failure. Avoid using.
RESULT_INTERNAL_ERROR 2: An internal, implementation error.
RESULT_PRECONDITION_ERROR 3: A precondition failed.
RESULT_INVALID_ARGUMENT 4: Invalid argument to a method.
RESULT_INVALID_STATE 5: The state requirements of an object method were violated.
RESULT_OUT_OF_MEMORY 6: Out of memory.
RESULT_SYSTEM_ERROR 7: System error.
RESULT_UNKNOWN_ERROR 8: Unknown error, generally an unexpected exception was detected.
RESULT_NOT_IMPLEMENTED 9: This message is not implemented. Please try again later.
RESULT_NOT_FOUND 10: The requested item or information was not found.
RESULT_ALREADY_EXISTS 11: The item to be stored already exists.
RESULT_NOT_EMPTY 12: The container is not empty.
RESULT_OUT_OF_BOUND 13: An index or value is out of the accepted bounds.
RESULT_INVALID_CONFIGURATION 14: Error in configuration files.
RESULT_NOT_ACTIVE 15: The object is not active.
RESULT_BUFFER_TOO_SMALL 16: Required buffer size is too small.
RESULT_IO_ERROR 17: Local input/output error.
RESULT_INSUFFICIENT_ACCESS 18: Insufficient access to perform the requested operation.
RESULT_BUSY 19: An object needed for the operation was busy with another operation.
RESULT_ACQUISITION_ERROR 20: Some abstract resource could not be acquired.
RESULT_NETWORK_ERROR 21: Generic network error.
RESULT_CORBA_ERROR 22: Generic CORBA error.
RESULT_CRC_FAILURE 23: Data set failed CRC check.
RESULT_CHECK_SUM_FAILURE 24: Data set failed check sum check. 
RESULT_INVALID_TYPE 25: Data is of invalid type.
RESULT_TIMEOUT 26: The function is timed out.
RESULT_FILE_NOT_OPEN 27: An error when trying to use a file which is not open yet.
RESULT_UNSUPPORTED_FORMAT 28: Format not supported
RESULT_OVER_BOUND   29:?????
RESULT_UNDER_BOUND  30:?????
RESULT_READ_FILE_ERROR 31:????
*/
typedef enum{
  RESULT_SUCCESS, 
  RESULT_FAILURE, 
  RESULT_INTERNAL_ERROR, 
  RESULT_PRECONDITION_ERROR,
  RESULT_INVALID_ARGUMENT, 
  RESULT_INVALID_STATE, 
  RESULT_OUT_OF_MEMORY, 
  RESULT_SYSTEM_ERROR,
  RESULT_UNKNOWN_ERROR, 
  RESULT_NOT_IMPLEMENTED, 
  RESULT_NOT_FOUND, 
  RESULT_ALREADY_EXISTS,
  RESULT_NOT_EMPTY, 
  RESULT_OUT_OF_BOUND, 
  RESULT_INVALID_CONFIGURATION, 
  RESULT_NOT_ACTIVE,
  RESULT_BUFFER_TOO_SMALL, 
  RESULT_IO_ERROR, 
  RESULT_INSUFFICIENT_ACCESS, 
  RESULT_BUSY,
  RESULT_ACQUISITION_ERROR, 
  RESULT_NETWORK_ERROR, 
  RESULT_CORBA_ERROR, 
  RESULT_CRC_FAILURE,
  RESULT_CHECK_SUM_FAILURE,
  RESULT_INVALID_TYPE, 
  RESULT_TIMEOUT, 
  RESULT_FILE_NOT_OPEN, 
  RESULT_UNSUPPORTED_FORMAT,
  RESULT_OVER_BOUND, 
  RESULT_UNDER_BOUND,
  RESULT_READ_FILE_ERROR
}Result;

//--------------------------------------------------------------------------------------------------
#endif //TYPES_H

