################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/carlos/STM32CubeIDE/workspace_1.9.0/LTDC_Display_1Layer/CM7/Src/main.c \
/home/carlos/STM32CubeIDE/workspace_1.9.0/LTDC_Display_1Layer/CM7/Src/stm32h7xx_hal_msp.c \
/home/carlos/STM32CubeIDE/workspace_1.9.0/LTDC_Display_1Layer/CM7/Src/stm32h7xx_it.c 

OBJS += \
./Example/User/CM7/main.o \
./Example/User/CM7/stm32h7xx_hal_msp.o \
./Example/User/CM7/stm32h7xx_it.o 

C_DEPS += \
./Example/User/CM7/main.d \
./Example/User/CM7/stm32h7xx_hal_msp.d \
./Example/User/CM7/stm32h7xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Example/User/CM7/main.o: /home/carlos/STM32CubeIDE/workspace_1.9.0/LTDC_Display_1Layer/CM7/Src/main.c Example/User/CM7/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H745xx -DUSE_STM32H745I_DISCO -DUSE_IOEXPANDER -DCORE_CM7 -c -I../../../Common/Inc -I../../../CM7/Inc -I../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32H745I-DISCO -I../../../Drivers/BSP/Components/Common -I../../../Utilities/Fonts -I../../../Utilities/CPU -I../../../Drivers/CMSIS/Include -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/CM7/stm32h7xx_hal_msp.o: /home/carlos/STM32CubeIDE/workspace_1.9.0/LTDC_Display_1Layer/CM7/Src/stm32h7xx_hal_msp.c Example/User/CM7/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H745xx -DUSE_STM32H745I_DISCO -DUSE_IOEXPANDER -DCORE_CM7 -c -I../../../Common/Inc -I../../../CM7/Inc -I../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32H745I-DISCO -I../../../Drivers/BSP/Components/Common -I../../../Utilities/Fonts -I../../../Utilities/CPU -I../../../Drivers/CMSIS/Include -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/CM7/stm32h7xx_it.o: /home/carlos/STM32CubeIDE/workspace_1.9.0/LTDC_Display_1Layer/CM7/Src/stm32h7xx_it.c Example/User/CM7/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H745xx -DUSE_STM32H745I_DISCO -DUSE_IOEXPANDER -DCORE_CM7 -c -I../../../Common/Inc -I../../../CM7/Inc -I../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32H745I-DISCO -I../../../Drivers/BSP/Components/Common -I../../../Utilities/Fonts -I../../../Utilities/CPU -I../../../Drivers/CMSIS/Include -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Example-2f-User-2f-CM7

clean-Example-2f-User-2f-CM7:
	-$(RM) ./Example/User/CM7/main.d ./Example/User/CM7/main.o ./Example/User/CM7/main.su ./Example/User/CM7/stm32h7xx_hal_msp.d ./Example/User/CM7/stm32h7xx_hal_msp.o ./Example/User/CM7/stm32h7xx_hal_msp.su ./Example/User/CM7/stm32h7xx_it.d ./Example/User/CM7/stm32h7xx_it.o ./Example/User/CM7/stm32h7xx_it.su

.PHONY: clean-Example-2f-User-2f-CM7

