################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := Downloads
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Downloads.elf \

MAP_FILES += \
Downloads.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Downloads.list \


# All Target
all: main-build

# Main-build Target
main-build: Downloads.elf secondary-outputs

# Tool invocations
Downloads.elf Downloads.map: $(OBJS) $(USER_OBJS) C:\Users\tkach\Embedded-C\My_workspace\target\Downloads\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Downloads.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\tkach\Embedded-C\My_workspace\target\Downloads\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Downloads.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Downloads.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Downloads.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Downloads.elf Downloads.list Downloads.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
