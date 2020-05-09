gdb=$([ `command -v gdb-multiarch` ] && echo "gdb-multiarch" || echo "arm-none-eabi-gdb")
$gdb -x cmd.gdb ./build/stm32_try.elf
