################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2/cmsis_os2.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2/cmsis_os2.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2/cmsis_os2.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2/%.o Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2/%.su: ../Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2/%.c Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_RTOS_LEPTON/Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include" -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_RTOS_LEPTON/SEGGER/Config" -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_RTOS_LEPTON/SEGGER/OS" -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_RTOS_LEPTON/SEGGER/SEGGER" -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_RTOS_LEPTON/Middlewares/Third_Party/FreeRTOS/org/Source/include" -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_RTOS_LEPTON/Middlewares/Third_Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F" -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_RTOS_LEPTON/Middlewares/Third_Party/FreeRTOS/org/Source/portable/MemMang" -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_RTOS_LEPTON/Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-org-2f-Source-2f-CMSIS_RTOS_V2

clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-org-2f-Source-2f-CMSIS_RTOS_V2:
	-$(RM) ./Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2/cmsis_os2.d ./Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2/cmsis_os2.o ./Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2/cmsis_os2.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-org-2f-Source-2f-CMSIS_RTOS_V2

