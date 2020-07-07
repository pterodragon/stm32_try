# Acronyms

- AAPCS = ARM Architecture Procedure Call Standard
- ADC = Analog to digital converter
- AHB = Advanced High-performance Bus
- APB = Advanced Peripheral Bus
- ARR = Auto reload register
- ART accelerator = Adaptive real time memory accelerator
- AXI = Advanced extensible interface
- BAM = Batch Acquisition Mode
- BAT = battery
- BFAR = BusFault Address Register
- BOM = bill of materials
- BOR = Brownout Reset
- CAL = calibration
- CMSIS = Cortex Microcontroller Software Interface Standard
- CCM = Core Coupled Memory 
- CCR = Capture/Compare register
- CFI = Common Flash Memory Interface
- CFSR = Configurable Fault Status Register
- COM = communication
- CPHA = clock phase
- CPOL = clock polarity
- CPLT = complete
- CPSR = current program status register
- CRC = Cyclic redundancy check
- CSS = Clock Security System
- D- = Data- (e.g. D-bus = data bus)
- DAC = Digital to analog converter
- DMIPS = Dhrystone MIPS (Million Instructions per Second)
- DFSR = Debug Fault Status Register
- DTS = Dead-time signal
- DVS = Dynamic Voltage Scaling
- DWT = Data Watchpoint and Tracing
- DMA = direct memory access
- EMI = Electromagnetic interference
- EOC = End of conversion
- EXTI = External Interrupt/Event Controller
- ESD = electrostatic discharge
- ETB = Embedded trace buffer
- ETM = Embedded Trace Macrocell
- ETR = External trigger
- FET = field-effect transistor
- FLITF = Flash memory interface
- FMC = Flexible memory controller
- FSMC = Flexible static memory controller
- GPIO = general-purpose input/output
- HAL = hardware abstraction layer
- HCLK = High Speed Clock
- HFSR = Hard fault status register
- HSE = High Speed External
- HSI = High Speed Internal
- H- = handle of- (in code e.g. hdma)
- IRQ = interrupt request
- ISR = interrupt service routine
- IV = Initialization vector
- IWDT = Independent watchdog
- I- = instruction- (e.g. I-bus = instruction bus)
- I2C = Inter-Integrated Circuit (eye squared see)
- ITM = Instrumentation Trace Macrocell
- ITR = Internal trigger
- JTAG = Joint Test Action Group
- LMA = Load memory address
- LP = Low Power
- LSE = Low Speed External
- LSI = Low Speed Internal
- MCO = Master Clock Output
- MCU = Microcontroller unit
- MISO = master input slave output
- MMFAR = MemManage Fault Address Register
- MMU = memory management unit
- MOSFET = metal–oxide–semiconductor field-effect transistor
- MOSI = master output slave input
- MPU = memory protection unit
- MSI = Multispeed Internal
- MSP = (1) MCU support package
- MSP = (2) main stack pointer
- MTB = Micro trace buffer
- NMI = non maskable interrupt
- NVIC = Nested Vectored Interrupt Controller
- NVM = non volatile memory
- OEM = Original Equipment Manufacturer
- OpenOCD = Open on chip debugger
- OPM = One pulse mode
- OTG = on the go
- OTP = one-time programmable
- OVR = overrun
- P- = peripheral-
- PCC = power consumption calculator
- PDR = Power Down Reset
- PER = peripheral
- PLL = Phase-Locked Loops
- POR = Power on reset
- PSP = process stack pointer
- PVD = programmable voltage detector
- PWM = Pulse-width modulation
- RCC = Reset and clock controller
- RTC = Real Time Clock
- S- = system- (e.g. S-bus = system bus)
- SAR = Successive approximation register
- SCB = System control block
- SCL = Serial clock line
- SCLK = Serial clock line
- SCK = Serial clock line
- SDA = Serial data line
- SEV = send event
- SHA = Sample-and-hold
- SISO = Slave in slave out (for 2-wire SPI)
- SPI = Serial Peripheral Interface
- SPIFI = SPI Flash Interface
- SS = Slave select
- SW = System Clock Switch
- SWD = Serial Wire Debug
- SWO = Serial Wire Output
- SWV = Serial Wire Viewer
- SYSCLK = System Clock Frequency
- SVC = Supervisor call
- TCM = Tightly Coupled Memory
- TFT = Thin-film-transistor
- TIM = Timer
- TRGO = trigger output
- TTL = transitor transitor logic
- UART = Universal Asynchronous Receiver/Transmitter
- TCB = Thread control block
- UIF = Update Interrupt Flag
- UEV = Update Event
- USART = Universal Synchronous and Asynchronous Receiver/Transmitter
- VBAT = voltage from battery (power supply when VDD is turned off)
- VCP = virtual COM port
- VMA = virtual memory address
- VTOR = vector table offset register
- WDT = watchdog timer
- WFE = wait for event
- WFI = wait for interrupt
- WWDG = window watchdog
- X- = trans- (e.g. Xfer = Transfer)

# Useful
- `HAL_UART_Init()` calls `HAL_UART_MspInit()`
- Registers for fault analysis (for STM32F446):
    - `SCB->CFSR == 0xe000ed28`
    - `p/x *(uint32_t*)0xE000ED28` in gdb
    - ref:
        - https://interrupt.memfault.com/blog/cortex-m-fault-debug
        - https://www.freertos.org/Debugging-Hard-Faults-On-Cortex-M-Microcontrollers.html

# Caveat
- When memory-to-memory mode is used, the circular and direct modes are not allowed. Only the DMA2 controller is able to perform memory-to-memory transfers. Ref: RM0390
- `DMA_MEMORY_TO_MEMORY` caveat: https://community.st.com/s/question/0D50X00009XkYz0SAF/timerdriven-single-word-memorytomemory-dma-transfer-can-it-be-done-in-hal
- `HAL_UART_Transmit_DMA` sets `XferCpltCallback` automatically
- Only USART2 is connected to the ST-LINK for nucleo-64 boards: Ref: UM1724; http://www.emcu.eu/how-to-manage-two-uart-usart2-and-usart1-under-interrupt/
    - USART1 can't receive data
- DMA1/2 might not connect to some peripherals: Ref: https://electronics.stackexchange.com/questions/298794/dma-triggered-by-timer-using-stm32f4-discovery-board/299169
- When memory-to-memory mode is used, the circular and direct modes are not allowed. Only the DMA2 controller is able to perform memory-to-memory transfers. Ref: RM0390

### Compiler/Linker options
- `-nostartfiles` prevents linking `crt0.o` which is a c runtime startup file
    - see `arm-none-eabi-objdump -d /usr/arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/hard/crt0.o`
- `-nodefaultlibs` prevents linking of `libgcc.a` which is to provide subroutine to override shortcoming of some archiecture, like providing a software emulation of long floating point division
    - see `/usr/lib/gcc/arm-none-eabi/7.3.1/thumb/v7e-m/fpv4-sp/hard/libgcc.a` (and the software floating point version for comparison)
- `-nostdlib` means both flags `-nostartfiles` and `-nodefaultlibs`
- reference: http://cs107e.github.io/guides/gcc/

### nosys/rdimon specs
- nosys means retargeting `printf` etc. to UART
- rdimon enables semihosting
- reference:
    - https://www.openstm32.org/Other%2BManuals
    - see `Drivers/STM32F4xx_HAL_Driver/Src/retarget/retarget.c`


### vPortSVCHandler
- reference:
    - http://www.openrtos.net/FreeRTOS_Support_Forum_Archive/June_2017/freertos_STM32F100RCT_MCU_STM32CubeMX_project_prvPortStartFirstTask_crash_af5cfb58j.html
STM32F100RCT MCU + STM32CubeMX project: prvPortStartFirstTask() crash
Posted by rtel on June 15, 2017

The SVC instruction effectively calls `vPortSVCHandler()`, which in turn starts the first task running. Often people step through to the SVC instruction when debugging and think the crash is occurring there - whereas in fact unknown to them the debugger has started a task executing and the crash actually happens in the task.

First set a break point in `vPortSVCHandler()` to see if it gets hit. If it does step through that program to see which task runs first. If you actually get into a task then something is going wrong in a task, not when starting the scheduler.


