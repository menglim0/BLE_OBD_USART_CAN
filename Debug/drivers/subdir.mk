################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/can.c \
../drivers/fsl_clock.c \
../drivers/fsl_common.c \
../drivers/fsl_emc.c \
../drivers/fsl_flexcomm.c \
../drivers/fsl_gpio.c \
../drivers/fsl_inputmux.c \
../drivers/fsl_power.c \
../drivers/fsl_reset.c \
../drivers/fsl_usart.c 

OBJS += \
./drivers/can.o \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_emc.o \
./drivers/fsl_flexcomm.o \
./drivers/fsl_gpio.o \
./drivers/fsl_inputmux.o \
./drivers/fsl_power.o \
./drivers/fsl_reset.o \
./drivers/fsl_usart.o 

C_DEPS += \
./drivers/can.d \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_emc.d \
./drivers/fsl_flexcomm.d \
./drivers/fsl_gpio.d \
./drivers/fsl_inputmux.d \
./drivers/fsl_power.d \
./drivers/fsl_reset.d \
./drivers/fsl_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DDEBUG -D__USE_CMSIS -DCPU_LPC54608J512ET180=1 -DUSE_FD -D__MCUXPRESSO -DCPU_LPC54608J512ET180 -DCPU_LPC54608J512ET180_cm4 -D__REDLIB__ -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\source" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\drivers" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\startup" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\CMSIS" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\board" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\utilities" -O0 -fno-common -g -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


