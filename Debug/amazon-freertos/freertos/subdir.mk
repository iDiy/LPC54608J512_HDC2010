################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../amazon-freertos/freertos/event_groups.c \
../amazon-freertos/freertos/list.c \
../amazon-freertos/freertos/queue.c \
../amazon-freertos/freertos/stream_buffer.c \
../amazon-freertos/freertos/tasks.c \
../amazon-freertos/freertos/timers.c 

OBJS += \
./amazon-freertos/freertos/event_groups.o \
./amazon-freertos/freertos/list.o \
./amazon-freertos/freertos/queue.o \
./amazon-freertos/freertos/stream_buffer.o \
./amazon-freertos/freertos/tasks.o \
./amazon-freertos/freertos/timers.o 

C_DEPS += \
./amazon-freertos/freertos/event_groups.d \
./amazon-freertos/freertos/list.d \
./amazon-freertos/freertos/queue.d \
./amazon-freertos/freertos/stream_buffer.d \
./amazon-freertos/freertos/tasks.d \
./amazon-freertos/freertos/timers.d 


# Each subdirectory must supply rules for building sources it contributes
amazon-freertos/freertos/%.o: ../amazon-freertos/freertos/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_LPC54608J512BD208 -DSDK_OS_FREE_RTOS -DFSL_RTOS_FREE_RTOS -DPRINTF_FLOAT_ENABLE=1 -DHDC2010_TEST=1 -DCPU_LPC54608J512BD208_cm4 -D__REDLIB__ -I../drivers/freertos -I../drivers -I../source -I../component/lists -I../device -I../amazon-freertos/freertos/portable -I../amazon-freertos/include -I../CMSIS -I../component/serial_manager -I../utilities -I../component/uart -I../board -I../ -I../board/boards -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


