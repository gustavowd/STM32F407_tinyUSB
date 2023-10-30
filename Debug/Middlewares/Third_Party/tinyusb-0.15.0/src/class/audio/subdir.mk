################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/audio_device.c 

OBJS += \
./Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/audio_device.o 

C_DEPS += \
./Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/audio_device.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/%.o Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/%.su Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/%.cyclo: ../Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/%.c Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/gustavo/STM32CubeIDE/workspace_1.10.1/STM32F407_tinyUSB/Middlewares/Third_Party/tinyusb-0.15.0/src" -I"/home/gustavo/STM32CubeIDE/workspace_1.10.1/STM32F407_tinyUSB/Middlewares/Third_Party/tinyusb-0.15.0/hw" -I"/home/gustavo/STM32CubeIDE/workspace_1.10.1/STM32F407_tinyUSB/Middlewares/Third_Party/tinyusb-0.15.0/hw/bsp/stm32f4/boards/stm32f407disco" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-tinyusb-2d-0-2e-15-2e-0-2f-src-2f-class-2f-audio

clean-Middlewares-2f-Third_Party-2f-tinyusb-2d-0-2e-15-2e-0-2f-src-2f-class-2f-audio:
	-$(RM) ./Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/audio_device.cyclo ./Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/audio_device.d ./Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/audio_device.o ./Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/audio_device.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-tinyusb-2d-0-2e-15-2e-0-2f-src-2f-class-2f-audio

