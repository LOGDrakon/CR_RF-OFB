################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/trace/adv_trace/stm32_adv_trace.c 

OBJS += \
./Utilities/trace/adv_trace/stm32_adv_trace.o 

C_DEPS += \
./Utilities/trace/adv_trace/stm32_adv_trace.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/trace/adv_trace/%.o Utilities/trace/adv_trace/%.su Utilities/trace/adv_trace/%.cyclo: ../Utilities/trace/adv_trace/%.c Utilities/trace/adv_trace/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Middlewares" -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Core/Src" -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Core" -I../SubGHz_Phy/App -I../SubGHz_Phy/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Drivers" -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/SubGHz_Phy" -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Utilities" -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Core/Startup" -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Drivers/CMSIS" -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Drivers/STM32WLxx_HAL_Driver" -I"S:/SAH_GMOY/S 02 02 MAINTENANCE/Kenzo (Machines - Logiciels - Documentations)/01 - PROJETS/CR_RF - Compresseur connecté Radio/Programmes/STM32 - CR_RF/CR_RF-OFB/Middlewares/Third_Party" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Utilities-2f-trace-2f-adv_trace

clean-Utilities-2f-trace-2f-adv_trace:
	-$(RM) ./Utilities/trace/adv_trace/stm32_adv_trace.cyclo ./Utilities/trace/adv_trace/stm32_adv_trace.d ./Utilities/trace/adv_trace/stm32_adv_trace.o ./Utilities/trace/adv_trace/stm32_adv_trace.su

.PHONY: clean-Utilities-2f-trace-2f-adv_trace

