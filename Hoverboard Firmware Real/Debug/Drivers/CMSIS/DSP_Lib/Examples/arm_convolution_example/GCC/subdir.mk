################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/arm_convolution_example_f32.c \
../Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/math_helper.c 

OBJS += \
./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/arm_convolution_example_f32.o \
./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/math_helper.o 

C_DEPS += \
./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/arm_convolution_example_f32.d \
./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/math_helper.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/%.o Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/%.su Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/%.cyclo: ../Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/%.c Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Examples-2f-arm_convolution_example-2f-GCC

clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Examples-2f-arm_convolution_example-2f-GCC:
	-$(RM) ./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/arm_convolution_example_f32.cyclo ./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/arm_convolution_example_f32.d ./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/arm_convolution_example_f32.o ./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/arm_convolution_example_f32.su ./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/math_helper.cyclo ./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/math_helper.d ./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/math_helper.o ./Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/math_helper.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Examples-2f-arm_convolution_example-2f-GCC

