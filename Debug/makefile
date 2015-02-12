################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv5/tools/compiler/msp430

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./F5xx_F6xx_Core_Lib/HAL_UCS.obj" \
"./F5xx_F6xx_Core_Lib/HAL_TLV.obj" \
"./F5xx_F6xx_Core_Lib/HAL_PMM.obj" \
"./F5xx_F6xx_Core_Lib/HAL_PMAP.obj" \
"./F5xx_F6xx_Core_Lib/HAL_FLASH.obj" \
"./USB_API/USB_CDC_API/UsbCdc.obj" \
"./USB_API/USB_Common/usb.obj" \
"./USB_API/USB_Common/dma.obj" \
"./USB_API/USB_HID_API/UsbHidReq.obj" \
"./USB_API/USB_HID_API/UsbHid.obj" \
"./USB_API/USB_MSC_API/UsbMscStateMachine.obj" \
"./USB_API/USB_MSC_API/UsbMscScsi.obj" \
"./USB_API/USB_MSC_API/UsbMscReq.obj" \
"./USB_API/USB_PHDC_API/UsbPHDC.obj" \
"./USB_config/descriptors.obj" \
"./USB_config/UsbIsr.obj" \
"./usbEventHandling.obj" \
"./usbConstructs.obj" \
"./timers.obj" \
"./main.obj" \
"./i2c.obj" \
"./flashMemory.obj" \
"./calibrateMode.obj" \
-l"libc.a" \
"../lnk_msp430f5528.cmd" \
"../msp430USB.cmd" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include USB_config/subdir_vars.mk
-include USB_API/USB_PHDC_API/subdir_vars.mk
-include USB_API/USB_MSC_API/subdir_vars.mk
-include USB_API/USB_HID_API/subdir_vars.mk
-include USB_API/USB_Common/subdir_vars.mk
-include USB_API/USB_CDC_API/subdir_vars.mk
-include F5xx_F6xx_Core_Lib/subdir_vars.mk
-include subdir_rules.mk
-include USB_config/subdir_rules.mk
-include USB_API/USB_PHDC_API/subdir_rules.mk
-include USB_API/USB_MSC_API/subdir_rules.mk
-include USB_API/USB_HID_API/subdir_rules.mk
-include USB_API/USB_Common/subdir_rules.mk
-include USB_API/USB_CDC_API/subdir_rules.mk
-include F5xx_F6xx_Core_Lib/subdir_rules.mk
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
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
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

# All Target
all: LightMaskManualOnOff.out

# Tool invocations
LightMaskManualOnOff.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: MSP430 Linker'
	"C:/ti/ccsv5/tools/compiler/msp430/bin/cl430" -vmspx --abi=coffabi --code_model=large --data_model=large -g --define=__MSP430F5528__ --diag_warning=225 --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=minimal -z -m"LightMaskManualOnOff.map" --stack_size=160 --heap_size=160 --use_hw_mpy=F5 -i"C:/ti/ccsv5/ccs_base/msp430/include" -i"C:/ti/ccsv5/tools/compiler/msp430/lib" -i"C:/ti/ccsv5/tools/compiler/msp430/include" --reread_libs --warn_sections --rom_model -o "LightMaskManualOnOff.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(MSP430_EXECUTABLE_OUTPUTS__QUOTED) "LightMaskManualOnOff.out"
	-$(RM) "calibrateMode.pp" "flashMemory.pp" "i2c.pp" "main.pp" "timers.pp" "usbConstructs.pp" "usbEventHandling.pp" "USB_config\UsbIsr.pp" "USB_config\descriptors.pp" "USB_API\USB_PHDC_API\UsbPHDC.pp" "USB_API\USB_MSC_API\UsbMscReq.pp" "USB_API\USB_MSC_API\UsbMscScsi.pp" "USB_API\USB_MSC_API\UsbMscStateMachine.pp" "USB_API\USB_HID_API\UsbHid.pp" "USB_API\USB_HID_API\UsbHidReq.pp" "USB_API\USB_Common\dma.pp" "USB_API\USB_Common\usb.pp" "USB_API\USB_CDC_API\UsbCdc.pp" "F5xx_F6xx_Core_Lib\HAL_FLASH.pp" "F5xx_F6xx_Core_Lib\HAL_PMAP.pp" "F5xx_F6xx_Core_Lib\HAL_PMM.pp" "F5xx_F6xx_Core_Lib\HAL_TLV.pp" "F5xx_F6xx_Core_Lib\HAL_UCS.pp" 
	-$(RM) "calibrateMode.obj" "flashMemory.obj" "i2c.obj" "main.obj" "timers.obj" "usbConstructs.obj" "usbEventHandling.obj" "USB_config\UsbIsr.obj" "USB_config\descriptors.obj" "USB_API\USB_PHDC_API\UsbPHDC.obj" "USB_API\USB_MSC_API\UsbMscReq.obj" "USB_API\USB_MSC_API\UsbMscScsi.obj" "USB_API\USB_MSC_API\UsbMscStateMachine.obj" "USB_API\USB_HID_API\UsbHid.obj" "USB_API\USB_HID_API\UsbHidReq.obj" "USB_API\USB_Common\dma.obj" "USB_API\USB_Common\usb.obj" "USB_API\USB_CDC_API\UsbCdc.obj" "F5xx_F6xx_Core_Lib\HAL_FLASH.obj" "F5xx_F6xx_Core_Lib\HAL_PMAP.obj" "F5xx_F6xx_Core_Lib\HAL_PMM.obj" "F5xx_F6xx_Core_Lib\HAL_TLV.obj" "F5xx_F6xx_Core_Lib\HAL_UCS.obj" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
