cmd_/home/bitcraze/crazyflie-firmware/examples/demos/test/src/test.o := arm-none-eabi-gcc -Wp,-MD,/home/bitcraze/crazyflie-firmware/examples/demos/test/src/.test.o.d     -I/home/bitcraze/crazyflie-firmware/examples/demos/test/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/bitcraze/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/bitcraze/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/bitcraze/crazyflie-firmware/vendor/libdw1000/inc   -I/home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/bitcraze/crazyflie-firmware/src/config   -I/home/bitcraze/crazyflie-firmware/src/platform/interface   -I/home/bitcraze/crazyflie-firmware/src/deck/interface   -I/home/bitcraze/crazyflie-firmware/src/deck/drivers/interface   -I/home/bitcraze/crazyflie-firmware/src/drivers/interface   -I/home/bitcraze/crazyflie-firmware/src/drivers/bosch/interface   -I/home/bitcraze/crazyflie-firmware/src/drivers/esp32/interface   -I/home/bitcraze/crazyflie-firmware/src/hal/interface   -I/home/bitcraze/crazyflie-firmware/src/modules/interface   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/cpx   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/controller   -I/home/bitcraze/crazyflie-firmware/src/modules/interface/estimator   -I/home/bitcraze/crazyflie-firmware/src/utils/interface   -I/home/bitcraze/crazyflie-firmware/src/utils/interface/kve   -I/home/bitcraze/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/bitcraze/crazyflie-firmware/src/utils/interface/tdoa   -I/home/bitcraze/crazyflie-firmware/src/lib/FatFS   -I/home/bitcraze/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/bitcraze/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/bitcraze/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/bitcraze/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/bitcraze/crazyflie-firmware/src/lib/vl53l1   -I/home/bitcraze/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/bitcraze/crazyflie-firmware/examples/demos/test/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -c -o /home/bitcraze/crazyflie-firmware/examples/demos/test/src/test.o /home/bitcraze/crazyflie-firmware/examples/demos/test/src/test.c

source_/home/bitcraze/crazyflie-firmware/examples/demos/test/src/test.o := /home/bitcraze/crazyflie-firmware/examples/demos/test/src/test.c

deps_/home/bitcraze/crazyflie-firmware/examples/demos/test/src/test.o := \
  /usr/include/newlib/string.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/newlib.h \
  /usr/include/newlib/_newlib_version.h \
  /usr/include/newlib/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/include/newlib/machine/ieeefp.h \
  /usr/include/newlib/sys/features.h \
  /usr/include/newlib/sys/reent.h \
  /usr/include/newlib/_ansi.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h \
  /usr/include/newlib/sys/_types.h \
  /usr/include/newlib/machine/_types.h \
  /usr/include/newlib/machine/_default_types.h \
  /usr/include/newlib/sys/lock.h \
  /usr/include/newlib/sys/cdefs.h \
  /usr/include/newlib/sys/_locale.h \
  /usr/include/newlib/strings.h \
  /usr/include/newlib/sys/string.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stdint.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stdbool.h \
  /usr/include/newlib/stdio.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stdarg.h \
  /usr/include/newlib/sys/types.h \
  /usr/include/newlib/sys/_stdint.h \
  /usr/include/newlib/machine/endian.h \
  /usr/include/newlib/machine/_endian.h \
  /usr/include/newlib/sys/select.h \
  /usr/include/newlib/sys/_sigset.h \
  /usr/include/newlib/sys/_timeval.h \
  /usr/include/newlib/sys/timespec.h \
  /usr/include/newlib/sys/_timespec.h \
  /usr/include/newlib/sys/_pthreadtypes.h \
  /usr/include/newlib/sys/sched.h \
  /usr/include/newlib/machine/types.h \
  /usr/include/newlib/sys/stdio.h \
  /usr/include/newlib/math.h \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/app.h \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/commander.h \
  /home/bitcraze/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/bitcraze/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /home/bitcraze/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/bitcraze/crazyflie-firmware/src/config/trace.h \
  /home/bitcraze/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/bitcraze/crazyflie-firmware/src/hal/interface/imu_types.h \
  /home/bitcraze/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/crtp_commander_high_level.h \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/math3d.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /home/bitcraze/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /home/bitcraze/crazyflie-firmware/src/config/config.h \
  /home/bitcraze/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/bitcraze/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/bitcraze/crazyflie-firmware/src/hal/interface/radiolink.h \
  /home/bitcraze/crazyflie-firmware/src/hal/interface/syslink.h \
  /home/bitcraze/crazyflie-firmware/src/utils/interface/configblock.h \
  /home/bitcraze/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/console.h \
  /home/bitcraze/crazyflie-firmware/src/utils/interface/eprintf.h \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/log.h \
    $(wildcard include/config/debug/log/enable.h) \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/param.h \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/param_logic.h \
  /home/bitcraze/crazyflie-firmware/src/modules/interface/crtp.h \

/home/bitcraze/crazyflie-firmware/examples/demos/test/src/test.o: $(deps_/home/bitcraze/crazyflie-firmware/examples/demos/test/src/test.o)

$(deps_/home/bitcraze/crazyflie-firmware/examples/demos/test/src/test.o):
