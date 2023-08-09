cmd_src/hal/src/pca9555.o := arm-none-eabi-gcc -Wp,-MD,src/hal/src/.pca9555.o.d    -I/home/bitcraze/crazyflie-firmware/src/hal/src -Isrc/hal/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/bitcraze/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/bitcraze/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/bitcraze/crazyflie-firmware/vendor/libdw1000/inc   -I/home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/bitcraze/crazyflie-firmware/src/config   -I/home/bitcraze/crazyflie-firmware/src/platform/interface   -I/home/bitcraze/crazyflie-firmware/src/deck/interface   -I/home/bitcraze/crazyflie-firmware/src/deck/drivers/interface   -I/home/bitcraze/crazyflie-firmware/src/drivers/interface   -I/home/bitcraze/crazyflie-firmware/src/drivers/bosch/interface   -I/home/bitcraze/crazyflie-firmware/src/drivers/esp32/interface   -I/home/bitcraze/crazyflie-firmware/src/hal/interface   -I/home/bitcraze/crazyflie-firmware/src/modules/interface   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/cpx   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/controller   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/estimator   -I/home/bitcraze/crazyflie-firmware/src/utils/interface   -I/home/bitcraze/crazyflie-firmware/src/utils/interface/kve   -I/home/bitcraze/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/bitcraze/crazyflie-firmware/src/utils/interface/tdoa   -I/home/bitcraze/crazyflie-firmware/src/lib/FatFS   -I/home/bitcraze/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/bitcraze/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/bitcraze/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/bitcraze/crazyflie-firmware/src/lib/vl53l1   -I/home/bitcraze/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/bitcraze/crazyflie-firmware/examples/demos/test/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -c -o src/hal/src/pca9555.o /home/bitcraze/crazyflie-firmware/src/hal/src/pca9555.c

source_src/hal/src/pca9555.o := /home/bitcraze/crazyflie-firmware/src/hal/src/pca9555.c

deps_src/hal/src/pca9555.o := \
    $(wildcard include/config/rega.h) \
    $(wildcard include/config/regb.h) \
  /home/bitcraze/crazyflie-firmware/src/drivers/interface/i2cdev.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stdint.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stdbool.h \
  /home/bitcraze/crazyflie-firmware/src/drivers/interface/i2c_drv.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h \
  /home/bitcraze/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /home/bitcraze/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/bitcraze/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /home/bitcraze/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/bitcraze/crazyflie-firmware/src/config/trace.h \
  /home/bitcraze/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/bitcraze/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/bitcraze/crazyflie-firmware/src/config/stm32fxxx.h \
  /home/bitcraze/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /home/bitcraze/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /home/bitcraze/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_version.h \
  /home/bitcraze/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /home/bitcraze/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /home/bitcraze/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/mpu_armv7.h \
  /home/bitcraze/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/system_stm32f4xx.h \
  /home/bitcraze/crazyflie-firmware/src/config/stm32f4xx_conf.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_crc.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h \
    $(wildcard include/config/it.h) \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_iwdg.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_wwdg.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_misc.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_cryp.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_hash.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rng.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_can.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dac.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dcmi.h \
  /home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fsmc.h \
  /home/bitcraze/crazyflie-firmware/src/hal/interface/pca9555.h \
  /home/bitcraze/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /home/bitcraze/crazyflie-firmware/src/config/config.h \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/console.h \
  /home/bitcraze/crazyflie-firmware/src/utils/interface/eprintf.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stdarg.h \

src/hal/src/pca9555.o: $(deps_src/hal/src/pca9555.o)

$(deps_src/hal/src/pca9555.o):
