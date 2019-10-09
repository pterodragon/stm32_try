# STM32F446RE Nucleo-64 development starter/notes

- Development environment: Ubuntu18.04

### Installing things
0. Install openjdk-8, openjfx-8 [[1]](#reference-sites)
    - $ `sudo apt-get install openjdk-8`
    - $ `sudo apt install openjfx=8u161-b12-1ubuntu2 libopenjfx-java=8u161-b12-1ubuntu2 libopenjfx-jni=8u161-b12-1ubuntu2`
    - $ `sudo update-alternatives --config java  # choose openjdk-8 jre`

    STM32CubeMX and STM32CubeProgrammer are Java programs and require the above
1. Install openocd (for debugging and flashing)
    - go to official website or
    - $ `git clone git://repo.or.cz/openocd.git`
        - $ `sudo make install`  # under openocd git repository
2. Install STM32CubeMX, STM32CubeProgrammer
    - go to ST official website to download. Extract the zipped file and launch the install executable

    At the time of writing, the version of STM32CubeMX is 5.3.0
3. Install arm-none-eabi toolchain
    - $ `sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa`
    - $ `sudo apt-get update`
    - $ `sudo apt-get install gcc-arm-embedded`

    This includes the `arm-none-eabi-gdb` for debugging

### Creating the project 
4. STM32CubeMX
    - launch it
    - click "Access to MCU selector"
    - go to the filter and select the relevant model
    - click "Start Project"
    - click the "System Core" of the left dock menu, under it click GPIO
        - click on PA5->GPIO_Output in the Pinout view
        - under the "GPIO Mode and Configuration" dock on the right of the left most dock, select the PA5 pin row. Change "GPIO output level" to High 
        - on the textfield of "User label", type in a name like "Ld2"
            - This would be reflected in `static void MX_GPIO_Init(void)` and some `#defines` in `main.c` of the generated code

        This makes sure the blink program work [[2]](#reference-sites)
    - Click the "Project Manager" the tab
        - type in the project name and the project location
        - select Toolchain/IDE to be Makefile
        - click "GENERATE CODE" button on the top right
            - a directory "\<project location>/\<project name>" will be created
5. Add the user code
    - go to the project directory (under the project name)
        - modify Src/main.c and add 
        ```
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            HAL_Delay(100);
        ```
        inside the main while loop
6. Compile the code
    - under the project directory
        - $ `make`

        This generates a .hex file and a .elf under the `build` directory which will be used later
# Flashing, running and debugging the code
7. Flash the code into the chip
    - connect the development board to a PC using an A Male to micro B cable
    - First option: STM32CubeProgrammer
        - launch it
        - on the left dock click the second button (Erasing & Programming)
        - Under "Download" section select the file path to be the *.hex file generated under the \<project directory>/build
        - check the tickbox "Verify Programming" and "Run after Programming"
        - on the right dock, make sure the serial number of the board is detected (may need to hit the refresh button)
        - click the "Connect" button on top of right dock 
        - click "Start Programming" under the "Download" section

        The board green LED (on the right of red power LED) should blink continuously
    - Second option: openocd [[3]](#reference-sites)
        - $ `openocd -d0 -f openocd/tcl/board/st_nucleo_f4.cfg -c "init;targets;halt;flash write_image erase <hex file>;shutdown"`  # `st_nucleo_f4.cfg` is under openocd git repository; # hex file is generated under the \<project directory>/build

    Note that sometimes hitting the user reset button is needed
        
8. Debug the code using openocd and arm-none-eabi-gdb
    - connect the development board to a PC using an A Male to micro B cable
    - $ `openocd -f openocd/tcl/board/st_nucleo_f4.cfg`  # file is under openocd git repository
    - This will be shown on the terminal
    ```
    Info : Listening on port 6666 for tcl connections
    Info : Listening on port 4444 for telnet connections
    Info : clock speed 2000 kHz
    Info : STLINK V2J35M26 (API v2) VID:PID 0483:374B
    Info : Target voltage: 3.258383
    Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
    Info : Listening on port 3333 for gdb connections
    ```
    - In another terminal,
    - $ `arm-none-eabi-gdb <the <project name>.elf file under <project directory>/build>`
        - Inside `arm-none-eabi-gdb`
            - `target remote localhost:3333`
            - `break main.c:<line number on the while loop>`

            - `arm-non-eabi-gdb` can be used like `gdb` afterwards (stepping the code and toggling the green LED by stepping the line `HAL_GPIO_TogglePin`)

# Troubleshooting

- Sometimes flashing doesn't work
    - for openocd: `Error: init mode failed (unable to connect to the target)`
    - for STM32CubeProgrammer: unable to find device/other error
    
    Try connecting BOOT0 pin to VDD
    - This will cause the chip not boot from flash memory
    then flashing

# Reference sites
- [1] https://superuser.com/questions/1419623/cannot-get-javafxopenjfx-to-work-with-openjdk-8-using-netbeans-8-2
- [2] https://www.instructables.com/id/STM32F103-Blink-LED/
- [3] https://stackoverflow.com/questions/37644823/how-to-flash-stm32-using-only-linux-terminal


# License
- MIT License for `Src` directory
- Other softwares in this repository follow their respective license
