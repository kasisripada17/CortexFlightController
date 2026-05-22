################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/altitude_hold.c \
../Core/Src/barometer.c \
../Core/Src/comms.c \
../Core/Src/filters.c \
../Core/Src/flight_control.c \
../Core/Src/gyro_calibration.c \
../Core/Src/lsm6ds3.c \
../Core/Src/main.c \
../Core/Src/motors.c \
../Core/Src/pid_control.c \
../Core/Src/print.c \
../Core/Src/radio.c \
../Core/Src/sensor_fusion.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/telemetry.c 

OBJS += \
./Core/Src/altitude_hold.o \
./Core/Src/barometer.o \
./Core/Src/comms.o \
./Core/Src/filters.o \
./Core/Src/flight_control.o \
./Core/Src/gyro_calibration.o \
./Core/Src/lsm6ds3.o \
./Core/Src/main.o \
./Core/Src/motors.o \
./Core/Src/pid_control.o \
./Core/Src/print.o \
./Core/Src/radio.o \
./Core/Src/sensor_fusion.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/telemetry.o 

C_DEPS += \
./Core/Src/altitude_hold.d \
./Core/Src/barometer.d \
./Core/Src/comms.d \
./Core/Src/filters.d \
./Core/Src/flight_control.d \
./Core/Src/gyro_calibration.d \
./Core/Src/lsm6ds3.d \
./Core/Src/main.d \
./Core/Src/motors.d \
./Core/Src/pid_control.d \
./Core/Src/print.d \
./Core/Src/radio.d \
./Core/Src/sensor_fusion.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_MotionGC_Library/Inc -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../Middlewares/ST/STM32_MotionAC_Library/Inc -I../Middlewares/ST/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/altitude_hold.cyclo ./Core/Src/altitude_hold.d ./Core/Src/altitude_hold.o ./Core/Src/altitude_hold.su ./Core/Src/barometer.cyclo ./Core/Src/barometer.d ./Core/Src/barometer.o ./Core/Src/barometer.su ./Core/Src/comms.cyclo ./Core/Src/comms.d ./Core/Src/comms.o ./Core/Src/comms.su ./Core/Src/filters.cyclo ./Core/Src/filters.d ./Core/Src/filters.o ./Core/Src/filters.su ./Core/Src/flight_control.cyclo ./Core/Src/flight_control.d ./Core/Src/flight_control.o ./Core/Src/flight_control.su ./Core/Src/gyro_calibration.cyclo ./Core/Src/gyro_calibration.d ./Core/Src/gyro_calibration.o ./Core/Src/gyro_calibration.su ./Core/Src/lsm6ds3.cyclo ./Core/Src/lsm6ds3.d ./Core/Src/lsm6ds3.o ./Core/Src/lsm6ds3.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motors.cyclo ./Core/Src/motors.d ./Core/Src/motors.o ./Core/Src/motors.su ./Core/Src/pid_control.cyclo ./Core/Src/pid_control.d ./Core/Src/pid_control.o ./Core/Src/pid_control.su ./Core/Src/print.cyclo ./Core/Src/print.d ./Core/Src/print.o ./Core/Src/print.su ./Core/Src/radio.cyclo ./Core/Src/radio.d ./Core/Src/radio.o ./Core/Src/radio.su ./Core/Src/sensor_fusion.cyclo ./Core/Src/sensor_fusion.d ./Core/Src/sensor_fusion.o ./Core/Src/sensor_fusion.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/telemetry.cyclo ./Core/Src/telemetry.d ./Core/Src/telemetry.o ./Core/Src/telemetry.su

.PHONY: clean-Core-2f-Src

