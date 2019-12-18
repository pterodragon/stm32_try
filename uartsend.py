#!/usr/bin/env python
import serial, time

SERIAL_PORT = "/dev/ttyACM0" #Windows users, replace with "COMx"
ser = serial.Serial(SERIAL_PORT, 115200)

ser.write(list(range(4, 7)))
    # ser.write((0,))
    # time.sleep(1)
    
ser.close()
