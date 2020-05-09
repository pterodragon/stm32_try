# Setup

- `pip install intelhex`
- `pip install pyserial`
- `pip install pycrypto`
- do not use `minicom` while using `flasher.py`

# expected flasher.py output:

```
STM32 MCU Type:  STM32F446RE
Erasing Flash memory........... Done
Loading build/main_app.hex HEX file....
        Start address:  0x8004000
        End address:  0x8007a3b  # can be different
        Writing address: 0x8004010 --- 1
        Writing address: 0x8004030 --- 1
```
