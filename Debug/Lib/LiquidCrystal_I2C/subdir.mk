################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/LiquidCrystal_I2C/LiquidCrystal_I2C.c 

OBJS += \
./Lib/LiquidCrystal_I2C/LiquidCrystal_I2C.o 

C_DEPS += \
./Lib/LiquidCrystal_I2C/LiquidCrystal_I2C.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/LiquidCrystal_I2C/%.o Lib/LiquidCrystal_I2C/%.su Lib/LiquidCrystal_I2C/%.cyclo: ../Lib/LiquidCrystal_I2C/%.c Lib/LiquidCrystal_I2C/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/DinhTU/STM32/blink/DEVI/Lib/LiquidCrystal_I2C" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Lib-2f-LiquidCrystal_I2C

clean-Lib-2f-LiquidCrystal_I2C:
	-$(RM) ./Lib/LiquidCrystal_I2C/LiquidCrystal_I2C.cyclo ./Lib/LiquidCrystal_I2C/LiquidCrystal_I2C.d ./Lib/LiquidCrystal_I2C/LiquidCrystal_I2C.o ./Lib/LiquidCrystal_I2C/LiquidCrystal_I2C.su

.PHONY: clean-Lib-2f-LiquidCrystal_I2C

