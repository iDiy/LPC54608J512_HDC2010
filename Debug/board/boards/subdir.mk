################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../board/boards/board.c \
../board/boards/clock_config.c \
../board/boards/peripherals.c \
../board/boards/pin_mux.c 

OBJS += \
./board/boards/board.o \
./board/boards/clock_config.o \
./board/boards/peripherals.o \
./board/boards/pin_mux.o 

C_DEPS += \
./board/boards/board.d \
./board/boards/clock_config.d \
./board/boards/peripherals.d \
./board/boards/pin_mux.d 


# Each subdirectory must supply rules for building sources it contributes
board/boards/%.o: ../board/boards/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_LPC54608J512BD208 -DSDK_OS_FREE_RTOS -DFSL_RTOS_FREE_RTOS -DPRINTF_FLOAT_ENABLE=1 -DHDC2010_TEST=1 -DCPU_LPC54608J512BD208_cm4 -D__REDLIB__ -I../drivers/freertos -I../drivers -I../source -I../component/lists -I../device -I../amazon-freertos/freertos/portable -I../amazon-freertos/include -I../CMSIS -I../component/serial_manager -I../utilities -I../component/uart -I../board -I../ -I../board/boards -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


