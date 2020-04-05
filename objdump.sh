elf_file=$1
if [ -z "$elf_file" ]
then
    elf_file=build/stm32_try.elf
fi
arm-none-eabi-objdump -h $elf_file
