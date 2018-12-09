################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../board/board.c \
../board/clock_config.c \
../board/pin_mux.c 

OBJS += \
./board/board.o \
./board/clock_config.o \
./board/pin_mux.o 

C_DEPS += \
./board/board.d \
./board/clock_config.d \
./board/pin_mux.d 


# Each subdirectory must supply rules for building sources it contributes
board/%.o: ../board/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DDEBUG -D__USE_CMSIS -DCPU_LPC54608J512ET180=1 -DUSE_FD -D__MCUXPRESSO -DCPU_LPC54608J512ET180 -DCPU_LPC54608J512ET180_cm4 -D__REDLIB__ -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\source" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\drivers" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\startup" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\CMSIS" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\board" -I"C:\Users\Andy\Documents\MCUXpressoIDE_10.0.0_344\workspace\fd_msgobjs\utilities" -O0 -fno-common -g -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


