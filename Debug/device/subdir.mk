################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../device/system_LPC54608.c 

OBJS += \
./device/system_LPC54608.o 

C_DEPS += \
./device/system_LPC54608.d 


# Each subdirectory must supply rules for building sources it contributes
device/%.o: ../device/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_LPC54608J512BD208 -DSDK_OS_FREE_RTOS -DFSL_RTOS_FREE_RTOS -DPRINTF_FLOAT_ENABLE=1 -DHDC2010_TEST=1 -DCPU_LPC54608J512BD208_cm4 -D__REDLIB__ -I../drivers/freertos -I../drivers -I../source -I../component/lists -I../device -I../amazon-freertos/freertos/portable -I../amazon-freertos/include -I../CMSIS -I../component/serial_manager -I../utilities -I../component/uart -I../board -I../ -I../board/boards -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

