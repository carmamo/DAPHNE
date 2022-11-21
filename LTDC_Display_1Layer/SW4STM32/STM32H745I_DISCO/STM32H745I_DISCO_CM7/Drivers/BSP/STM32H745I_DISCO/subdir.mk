################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/carlos/STM32CubeIDE/workspace_1.9.0/LTDC_Display_1Layer/Drivers/BSP/STM32H745I-DISCO/stm32h745i_discovery.c 

OBJS += \
./Drivers/BSP/STM32H745I_DISCO/stm32h745i_discovery.o 

C_DEPS += \
./Drivers/BSP/STM32H745I_DISCO/stm32h745i_discovery.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32H745I_DISCO/stm32h745i_discovery.o: /home/carlos/STM32CubeIDE/workspace_1.9.0/LTDC_Display_1Layer/Drivers/BSP/STM32H745I-DISCO/stm32h745i_discovery.c Drivers/BSP/STM32H745I_DISCO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H745xx -DUSE_STM32H745I_DISCO -DUSE_IOEXPANDER -DCORE_CM7 -c -I../../../Common/Inc -I../../../CM7/Inc -I../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32H745I-DISCO -I../../../Drivers/BSP/Components/Common -I../../../Utilities/Fonts -I../../../Utilities/CPU -I../../../Drivers/CMSIS/Include -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32H745I_DISCO

clean-Drivers-2f-BSP-2f-STM32H745I_DISCO:
	-$(RM) ./Drivers/BSP/STM32H745I_DISCO/stm32h745i_discovery.d ./Drivers/BSP/STM32H745I_DISCO/stm32h745i_discovery.o ./Drivers/BSP/STM32H745I_DISCO/stm32h745i_discovery.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32H745I_DISCO

