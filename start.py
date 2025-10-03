#!/usr/bin/env python3

import os
import subprocess
import sys
import time
import serial

BT_ADDR = "74:D5:C6:52:65:C2"
DEVICE_NODE = "/dev/rfcomm0"
CHANNEL = 6
CONTAINER_NAME = "shimmer3" # give this container a name
WAIT_SEC = 2
syst = os.system

def run(cmd): # for slight convenience only
    # print(f"$ {cmd}")
    return subprocess.run(cmd, shell=True, capture_output=True, text=True) # runs cmd through (shell), and (check)s then raises a Python exception if command fails
    

def is_dev_connected():
    pass

def bind_connect_shimmer3():
    while True:
        device_node_exists= os.path.exists('/dev/rfcomm0')

        try:
            if not device_node_exists:
                print(f"Device node {DEVICE_NODE} does not exist yet. Binding now...") 
                run("sudo rfcomm release 0") # release rfcomm0
                run(f"sudo rfcomm bind 0 {BT_ADDR} {str(CHANNEL)}")
                run(f"sudo chmod a+rw {DEVICE_NODE}") # (ch)ange (mod)e and let (a)ll users (r)ead or (w)rite to device_NODE  
                ser = serial.Serial(DEVICE_NODE, 115200, timeout=1) # open port
                ser.write(b'\x07') # trigger data stream
                time.sleep(WAIT_SEC)
                data = ser.read(100) # read arbitrary 100B of data
                if data: 
                    print(f"Bluetooth: {BT_ADDR} connection outside of '{CONTAINER_NAME}' container successful!")
                    break # if the data exists, success!
            else: 
                print(f"Device node {DEVICE_NODE} already exists. Moving onto Docker stage...")
                break 
        except subprocess.CalledProcessError as e:  
            print(f"Command failed: {e}")
        # print("Connection unsuccessful, trying again") # !! Right now except will end the process instead of allowing retrying


def build_run():
    syst("xhost +local:") # allows docker to connect to display (X11) for generating visualization windows
    run_output = subprocess.run('docker ps -a --format "{{.Names}}"', shell=True, capture_output=True, check=True, text=True)
    if run_output:
        container_names = run_output.stdout.splitlines()
        # print(f"container_names: {container_names}")
        if container_names and CONTAINER_NAME in container_names:
            print(f"'{CONTAINER_NAME}' container already exists. Starting now...")
            syst(f"docker start {CONTAINER_NAME}") # restart the existing container
            syst(f"docker exec -it -e DISPLAY=$DISPLAY {CONTAINER_NAME} bash") # open a persisting interactive bash in the container. Could potentially replace with -ai
        else: 
            print(f"{CONTAINER_NAME} container does not exist yet. Building and running now...")
            syst(f"docker build -t {CONTAINER_NAME}:jazzy .") # build (then soon run)
            syst(f"""docker run -it \
                --name {CONTAINER_NAME} \
                --device /dev/rfkill \
                --device /dev/ttyUSB0 \
                --device {DEVICE_NODE} \
                -v $(pwd)/src:/shimmer3/src \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -e DISPLAY=$DISPLAY \
                {CONTAINER_NAME}:jazzy \
                bash -c "source /opt/venv/bin/activate && \
                        source /opt/ros/jazzy/setup.bash && \
                        source /shimmer3/install/setup.bash && \
                        export PS1='[\\[\\e[38;2;13;183;237m\\]][CONTAINER \\u@\\h \\w]\\$ [\\[\\e[0m\\]]' && \
                        exec bash -i"
                """)
        
if __name__ == "__main__":
    bind_connect_shimmer3()
    build_run()