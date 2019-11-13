################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utilities/subdir.mk
-include startup/subdir.mk
-include source/subdir.mk
-include drivers/freertos/subdir.mk
-include drivers/subdir.mk
-include device/subdir.mk
-include component/uart/subdir.mk
-include component/serial_manager/subdir.mk
-include component/lists/subdir.mk
-include board/boards/subdir.mk
-include amazon-freertos/freertos/portable/subdir.mk
-include amazon-freertos/freertos/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: LPC54608J512_hdc2010.axf

# Tool invocations
LPC54608J512_hdc2010.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"D:\temp\mcuxpresso\LPC54608J512_hdc2010\libs" -Xlinker -Map="LPC54608J512_hdc2010.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -T LPC54608J512_hdc2010_Debug.ld -o "LPC54608J512_hdc2010.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) LPC54608J512_hdc2010.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "LPC54608J512_hdc2010.axf"; # arm-none-eabi-objcopy -v -O binary "LPC54608J512_hdc2010.axf" "LPC54608J512_hdc2010.bin" ; # checksum -p LPC54608J512 -d "LPC54608J512_hdc2010.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
