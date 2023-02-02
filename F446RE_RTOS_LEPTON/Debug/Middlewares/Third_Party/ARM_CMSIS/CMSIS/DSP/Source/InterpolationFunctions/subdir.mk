################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q7.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q7.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q7.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/%.su: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_RTOS_LEPTON/Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/dsp" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I"/home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_RTOS_LEPTON/Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-CMSIS-2f-DSP-2f-Source-2f-InterpolationFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-CMSIS-2f-DSP-2f-Source-2f-InterpolationFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q7.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q7.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q7.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-CMSIS-2f-DSP-2f-Source-2f-InterpolationFunctions

