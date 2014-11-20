################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../cr_startup_lpc17.c \
../main.c 

OBJS += \
./cr_startup_lpc17.o \
./main.o 

C_DEPS += \
./cr_startup_lpc17.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__USE_CMSIS=CMSISv1p30_LPC17xx -D__CODE_RED -D__REDLIB__ -I"/Users/Rhemi25/Dropbox/Rich Stuff/School/Fall '14/Comp Op Sys/LPCXpresso Workspace/FreeRTOS_Library/demo_code" -I"/Users/Rhemi25/Dropbox/Rich Stuff/School/Fall '14/Comp Op Sys/LPCXpresso Workspace/CMSISv1p30_LPC17xx/inc" -I"/Users/Rhemi25/Dropbox/Rich Stuff/School/Fall '14/Comp Op Sys/LPCXpresso Workspace/FreeRTOS_Library/include" -I"/Users/Rhemi25/Dropbox/Rich Stuff/School/Fall '14/Comp Op Sys/LPCXpresso Workspace/FreeRTOS_Library/portable" -Og -g3 -fsigned-char -c -fmessage-length=0 -fno-builtin -ffunction-sections -mcpu=cortex-m3 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


