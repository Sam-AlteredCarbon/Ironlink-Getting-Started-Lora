################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Ironlink/ironlink-library.c 

OBJS += \
./Drivers/Ironlink/ironlink-library.o 

C_DEPS += \
./Drivers/Ironlink/ironlink-library.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Ironlink/ironlink-library.o: ../Drivers/Ironlink/ironlink-library.c Drivers/Ironlink/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F070xB -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F0xx_HAL_Driver/Inc -I"C:/Users/WS/Dropbox/Code/Ironlink-Getting-Started-Lora-v1-1/Drivers/Ironlink" -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Ironlink/ironlink-library.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

