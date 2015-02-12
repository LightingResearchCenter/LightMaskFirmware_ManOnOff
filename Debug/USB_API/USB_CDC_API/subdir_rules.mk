################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
USB_API/USB_CDC_API/UsbCdc.obj: ../USB_API/USB_CDC_API/UsbCdc.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccsv5/tools/compiler/msp430/bin/cl430" -vmspx --abi=coffabi --code_model=large --data_model=large -g --include_path="C:/ti/ccsv5/ccs_base/msp430/include" --include_path="C:/AndyOct2012/CodeComposerWorkspaceV5_1/LightMaskManualOnOff" --include_path="C:/AndyOct2012/CodeComposerWorkspaceV5_1/LightMaskManualOnOff/F5xx_F6xx_Core_Lib" --include_path="C:/AndyOct2012/CodeComposerWorkspaceV5_1/LightMaskManualOnOff/USB_API" --include_path="C:/AndyOct2012/CodeComposerWorkspaceV5_1/LightMaskManualOnOff/USB_config" --include_path="C:/ti/ccsv5/tools/compiler/msp430/include" --define=__MSP430F5528__ --diag_warning=225 --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=minimal --preproc_with_compile --preproc_dependency="USB_API/USB_CDC_API/UsbCdc.pp" --obj_directory="USB_API/USB_CDC_API" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


