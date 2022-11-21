################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
/home/carlos/STM32CubeIDE/workspace_1.9.0/LTDC_Display_1Layer/SW4STM32/startup_stm32h745xx.s 

OBJS += \
./Example/SW4STM32/startup_stm32h745xx.o 

S_DEPS += \
./Example/SW4STM32/startup_stm32h745xx.d 


# Each subdirectory must supply rules for building sources it contributes
Example/SW4STM32/startup_stm32h745xx.o: /home/carlos/STM32CubeIDE/workspace_1.9.0/LTDC_Display_1Layer/SW4STM32/startup_stm32h745xx.s Example/SW4STM32/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Example-2f-SW4STM32

clean-Example-2f-SW4STM32:
	-$(RM) ./Example/SW4STM32/startup_stm32h745xx.d ./Example/SW4STM32/startup_stm32h745xx.o

.PHONY: clean-Example-2f-SW4STM32

