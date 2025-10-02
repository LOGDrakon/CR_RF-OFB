################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32wl55ccux.s 

OBJS += \
./Core/Startup/startup_stm32wl55ccux.o 

S_DEPS += \
./Core/Startup/startup_stm32wl55ccux.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Middlewares" -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Core/Src" -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Core" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32wl55ccux.d ./Core/Startup/startup_stm32wl55ccux.o

.PHONY: clean-Core-2f-Startup

