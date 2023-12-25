################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q31.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q31.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.c 

OBJS += \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q31.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q31.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.o 

C_DEPS += \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q31.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q31.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/%.o Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/%.su Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/%.cyclo: ../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/%.c Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source-2f-FastMathFunctions

clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source-2f-FastMathFunctions:
	-$(RM) ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.cyclo ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.d ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.o ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.su ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q15.cyclo ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q15.d ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q15.o ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q15.su ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q31.cyclo ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q31.d ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q31.o ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q31.su ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.cyclo ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.d ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.o ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.su ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q15.cyclo ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q15.d ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q15.o ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q15.su ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q31.cyclo ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q31.d ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q31.o ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q31.su ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q15.cyclo ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q15.d ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q15.o ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q15.su ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.cyclo ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.d ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.o ./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source-2f-FastMathFunctions

