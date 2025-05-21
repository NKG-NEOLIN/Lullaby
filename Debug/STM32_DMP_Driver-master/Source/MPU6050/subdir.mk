################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32_DMP_Driver-master/Source/MPU6050/I2C.c \
../STM32_DMP_Driver-master/Source/MPU6050/inv_mpu.c \
../STM32_DMP_Driver-master/Source/MPU6050/inv_mpu_dmp_motion_driver.c \
../STM32_DMP_Driver-master/Source/MPU6050/mpu6050.c 

OBJS += \
./STM32_DMP_Driver-master/Source/MPU6050/I2C.o \
./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu.o \
./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu_dmp_motion_driver.o \
./STM32_DMP_Driver-master/Source/MPU6050/mpu6050.o 

C_DEPS += \
./STM32_DMP_Driver-master/Source/MPU6050/I2C.d \
./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu.d \
./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu_dmp_motion_driver.d \
./STM32_DMP_Driver-master/Source/MPU6050/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
STM32_DMP_Driver-master/Source/MPU6050/%.o STM32_DMP_Driver-master/Source/MPU6050/%.su STM32_DMP_Driver-master/Source/MPU6050/%.cyclo: ../STM32_DMP_Driver-master/Source/MPU6050/%.c STM32_DMP_Driver-master/Source/MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"../STM32_DMP_Driver-master/Inlcude" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32_DMP_Driver-2d-master-2f-Source-2f-MPU6050

clean-STM32_DMP_Driver-2d-master-2f-Source-2f-MPU6050:
	-$(RM) ./STM32_DMP_Driver-master/Source/MPU6050/I2C.cyclo ./STM32_DMP_Driver-master/Source/MPU6050/I2C.d ./STM32_DMP_Driver-master/Source/MPU6050/I2C.o ./STM32_DMP_Driver-master/Source/MPU6050/I2C.su ./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu.cyclo ./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu.d ./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu.o ./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu.su ./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu_dmp_motion_driver.cyclo ./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu_dmp_motion_driver.d ./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu_dmp_motion_driver.o ./STM32_DMP_Driver-master/Source/MPU6050/inv_mpu_dmp_motion_driver.su ./STM32_DMP_Driver-master/Source/MPU6050/mpu6050.cyclo ./STM32_DMP_Driver-master/Source/MPU6050/mpu6050.d ./STM32_DMP_Driver-master/Source/MPU6050/mpu6050.o ./STM32_DMP_Driver-master/Source/MPU6050/mpu6050.su

.PHONY: clean-STM32_DMP_Driver-2d-master-2f-Source-2f-MPU6050

