################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./F2837xS_Adc.obj" \
"./F2837xS_CodeStartBranch.obj" \
"./F2837xS_CpuTimers.obj" \
"./F2837xS_DefaultISR.obj" \
"./F2837xS_EPwm.obj" \
"./F2837xS_EQep.obj" \
"./F2837xS_GlobalVariableDefs.obj" \
"./F2837xS_Gpio.obj" \
"./F2837xS_PieCtrl.obj" \
"./F2837xS_PieVect.obj" \
"./F2837xS_SysCtrl.obj" \
"./F2837xS_usDelay.obj" \
"./pm_stepper_motor_controller.obj" \
$(GEN_CMDS__FLAG) \
-l"IQmath_fpu32.lib" \
-l"rts2800_fpu32.lib" \
-l"2837xS_RAM_IQMATH_lnk_cpu1.cmd" \
-l"F2837xS_Headers_nonBIOS.cmd" \
-l"libc.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
pm_stepper_motor_controller_v2.out \

EXE_OUTPUTS__QUOTED += \
"pm_stepper_motor_controller_v2.out" \

BIN_OUTPUTS += \
pm_stepper_motor_controller_v2.hex \

BIN_OUTPUTS__QUOTED += \
"pm_stepper_motor_controller_v2.hex" \


# All Target
all: pm_stepper_motor_controller_v2.out

# Tool invocations
pm_stepper_motor_controller_v2.out: $(OBJS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --tmu_support=tmu0 --vcu_support=vcu2 --float_support=fpu32 --cla_support=cla1 --fp_mode=relaxed --advice:performance=all -g --display_error_number --diag_wrap=off --diag_warning=225 -z -m"pm_stepper_motor_controller_v2.map" --stack_size=0x200 --warn_sections -i"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/lib" -i"C:/ti/controlSUITE/libs/math/IQmath/v160/lib" -i"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" -i"C:/ti/controlSUITE/device_support/F2837xS/v191/F2837xS_common/cmd" -i"C:/ti/controlSUITE/device_support/F2837xS/v191/F2837xS_headers/cmd" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="pm_stepper_motor_controller_v2_linkInfo.xml" --rom_model -o "pm_stepper_motor_controller_v2.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

pm_stepper_motor_controller_v2.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: C2000 Hex Utility'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/hex2000"  -o "pm_stepper_motor_controller_v2.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "F2837xS_Adc.pp" "F2837xS_CpuTimers.pp" "F2837xS_DefaultISR.pp" "F2837xS_EPwm.pp" "F2837xS_EQep.pp" "F2837xS_GlobalVariableDefs.pp" "F2837xS_Gpio.pp" "F2837xS_PieCtrl.pp" "F2837xS_PieVect.pp" "F2837xS_SysCtrl.pp" "pm_stepper_motor_controller.pp" 
	-$(RM) "F2837xS_Adc.obj" "F2837xS_CodeStartBranch.obj" "F2837xS_CpuTimers.obj" "F2837xS_DefaultISR.obj" "F2837xS_EPwm.obj" "F2837xS_EQep.obj" "F2837xS_GlobalVariableDefs.obj" "F2837xS_Gpio.obj" "F2837xS_PieCtrl.obj" "F2837xS_PieVect.obj" "F2837xS_SysCtrl.obj" "F2837xS_usDelay.obj" "pm_stepper_motor_controller.obj" 
	-$(RM) "F2837xS_CodeStartBranch.pp" "F2837xS_usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

