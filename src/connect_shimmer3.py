#!/usr/bin/env python3

import os
import subprocess
import sys
import time
import serial

BT_ADDR = "74:D5:C6:52:65:C2"
DEVICE = "/dev/rfcomm0"
CHANNEL = 6
CONTAINER_NAME = "shimmer3" # give this container a name
WAIT_SEC = 2

print("connect_shimmer3")

def run(cmd):
    print(f"$ {cmd}")
    subprocess.run(cmd, shell=True, check=True) # runs cmd through (shell), and (check)s then raises a Python exception if command fails

def is_dev_connected():
    pass

def connect_shimmer3():
    while True:
        try:
            # run("sudo rfcomm release 0") # release rfcomm0
            # run(f"sudo rfcomm bind 0 {BT_ADDR} {str(CHANNEL)}")
            # run(f"sudo chmod a+rw {DEVICE}") # (ch)ange (mod)e and let (a)ll users (r)ead or (w)rite to device  
            ser = serial.Serial(DEVICE, 115200, timeout=1) # open port
            ser.write(b'\x07') # trigger data stream
            time.sleep(WAIT_SEC)
            data = ser.read(100) # read arbitrary 100B of data
            if data: 
                print(f"{BT_ADDR} connection successful on channel {CHANNEL} and rfcomm port device {DEVICE} in container {CONTAINER_NAME}!")
                return 0 # if the read data exists, success!
        except subprocess.CalledProcessError as e:
            print(f"Command failed: {e}")
            # ser = None
            return 1
        print("Connection unsuccessful, trying again")
    
if __name__ == "__main__":
    connect_shimmer3()