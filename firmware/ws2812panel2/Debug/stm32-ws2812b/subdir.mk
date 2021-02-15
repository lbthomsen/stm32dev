################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32-ws2812b/ws2812b.c 

OBJS += \
./stm32-ws2812b/ws2812b.o 

C_DEPS += \
./stm32-ws2812b/ws2812b.d 


# Each subdirectory must supply rules for building sources it contributes
stm32-ws2812b/ws2812b.o: ../stm32-ws2812b/ws2812b.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../stm32-ws2812b/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"stm32-ws2812b/ws2812b.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

