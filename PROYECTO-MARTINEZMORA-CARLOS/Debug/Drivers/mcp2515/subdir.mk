################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/mcp2515/CANSPI.c \
../Drivers/mcp2515/MCP2515.c 

OBJS += \
./Drivers/mcp2515/CANSPI.o \
./Drivers/mcp2515/MCP2515.o 

C_DEPS += \
./Drivers/mcp2515/CANSPI.d \
./Drivers/mcp2515/MCP2515.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/mcp2515/%.o Drivers/mcp2515/%.su: ../Drivers/mcp2515/%.c Drivers/mcp2515/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F070xB -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/PROYECTO-MARTINEZMORA-CARLOS/Drivers/u8g2" -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/PROYECTO-MARTINEZMORA-CARLOS/Drivers/mcp2515" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-mcp2515

clean-Drivers-2f-mcp2515:
	-$(RM) ./Drivers/mcp2515/CANSPI.d ./Drivers/mcp2515/CANSPI.o ./Drivers/mcp2515/CANSPI.su ./Drivers/mcp2515/MCP2515.d ./Drivers/mcp2515/MCP2515.o ./Drivers/mcp2515/MCP2515.su

.PHONY: clean-Drivers-2f-mcp2515

