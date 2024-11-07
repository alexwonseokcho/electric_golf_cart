################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/iar/subdir.mk
-include Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/subdir.mk
-include Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/arm/subdir.mk
-include Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/TransformFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/SupportFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/StatisticsFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/MatrixFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/FilteringFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/ControllerFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/ComplexMathFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/CommonTables/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/BasicMathFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_variance_example/ARM/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_sin_cos_example/ARM/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_signal_converge_example/ARM/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_matrix_example/ARM/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_linear_interp_example/ARM/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_graphic_equalizer_example/ARM/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_fir_example/ARM/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_fft_bin_example/GCC/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_fft_bin_example/ARM/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_dotproduct_example/GCC/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_dotproduct_example/ARM/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/GCC/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_convolution_example/ARM/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_class_marks_example/GCC/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Examples/arm_class_marks_example/ARM/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Hoverboard Firmware Real
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Hoverboard\ Firmware\ Real.elf \

MAP_FILES += \
Hoverboard\ Firmware\ Real.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Hoverboard\ Firmware\ Real.list \


# All Target
all: main-build

# Main-build Target
main-build: Hoverboard\ Firmware\ Real.elf secondary-outputs

# Tool invocations
Hoverboard\ Firmware\ Real.elf Hoverboard\ Firmware\ Real.map: $(OBJS) $(USER_OBJS) C:\Users\techn\Documents\Hoverboard\ Firmware\ Real\STM32F103RCTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Hoverboard Firmware Real.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\techn\Documents\Hoverboard Firmware Real\STM32F103RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Hoverboard Firmware Real.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Hoverboard\ Firmware\ Real.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Hoverboard Firmware Real.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Hoverboard\ Firmware\ Real.elf Hoverboard\ Firmware\ Real.list Hoverboard\ Firmware\ Real.map Hoverboard\\ Firmware\\ Real.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
