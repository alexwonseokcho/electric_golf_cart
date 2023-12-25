################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.c \
../Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.c 

OBJS += \
./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.o \
./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.o 

C_DEPS += \
./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.d \
./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP_Lib/Source/CommonTables/%.o Drivers/CMSIS/DSP_Lib/Source/CommonTables/%.su Drivers/CMSIS/DSP_Lib/Source/CommonTables/%.cyclo: ../Drivers/CMSIS/DSP_Lib/Source/CommonTables/%.c Drivers/CMSIS/DSP_Lib/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source-2f-CommonTables

clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source-2f-CommonTables:
	-$(RM) ./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.cyclo ./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.d ./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.o ./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.su ./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.cyclo ./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.d ./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.o ./Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source-2f-CommonTables

