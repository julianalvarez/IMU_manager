################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../component/lists/fsl_component_generic_list.c 

C_DEPS += \
./component/lists/fsl_component_generic_list.d 

OBJS += \
./component/lists/fsl_component_generic_list.o 


# Each subdirectory must supply rules for building sources it contributes
component/lists/%.o: ../component/lists/%.c component/lists/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_MIMXRT1064DVL6A -DCPU_MIMXRT1064DVL6A_cm7 -DSDK_OS_BAREMETAL -DXIP_EXTERNAL_FLASH=1 -DXIP_BOOT_HEADER_ENABLE=1 -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DSERIAL_PORT_TYPE_UART=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSYSTEM_STEER -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\IMU_manager\filter" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\IMU_manager\canmsg" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\IMU_manager" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\IMU_manager\include" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\j1939_STEER_LIB\Include" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\common_package\include" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\CSP_rt1064\device" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\CSP_rt1064\utilities" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\CSP_rt1064\xip" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\CSP_rt1064\drivers" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\CSP_rt1064\CMSIS" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\CSP_rt1064\component\lists" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\CSP_rt1064\component\serial_manager" -I"C:\Users\PLANTIUM\Documents\MCUXpressoIDE_11.8.0_1165\workspace\CSP_rt1064\component\uart" -O0 -fno-common -g3 -gdwarf-4 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-component-2f-lists

clean-component-2f-lists:
	-$(RM) ./component/lists/fsl_component_generic_list.d ./component/lists/fsl_component_generic_list.o

.PHONY: clean-component-2f-lists

