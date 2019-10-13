# openocd -d0 -f ~/Documents/dev/STM32_dev/openocd-0.10.0/tcl/board/st_nucleo_f4.cfg -c "init;targets;halt;flash write_image erase build/stm32_try.hex;shutdown"
openocd -d0 -f ./openocd/tcl/board/st_nucleo_f4.cfg -c "init;targets;halt;flash write_image erase build/stm32_try.hex;shutdown"
