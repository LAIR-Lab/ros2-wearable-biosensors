# ros2-foxy-wearable-biosensors

*Future Improvements*
- Move on from obsolete BlueZ rfcomm in `bluez-obsolete-utils` to `bluetoothctl`

# Basic Code Structure
`start.py`
- Tests Bluetooth connection
- Checks for existing shimmer3 Docker container and starts existing or builds and runs new one. You are now inside the container.

`shimmer3.launch.py`
- Configures launch parameters and other default constants 
- Defines and spins (and eventually destroys on shutdown) nodes:
  -  `shimmer3_node`
      - Connects to Bluetooth
      - Streams and publishes shimmer3 data as singles or chunks 
  -  `bpm_detector_node`
      - Subscribes to  `shimmer3_node`'s `biosensors/shimmer3_gsr/ppg` topic for live ppg data.
      - Uses `heartpy`'s machine learning models to clean data and determine peaks from a rolling buffer.
      - Calculates bpm from peaklist, providing some cleaning using reasonable upper limits and sample-to-sample heartrate variability (e.g, its unreasonable for two samples 1/50th of a second apart to register heartrates as wide apart as 20bpm) 
      - Publishes heartrate as a single `live_bpm` and rolling buffer `bpm_buffer` for plotting. 
  -  `matplotter_node`
      - Subscribes to `bpm_detector_node`'s `bpm_buffer` and outputs a window
      - Can be toggle via launch parameter, on by default. 

# Requirements
* Install python libraries:
```bash
$ pip3 install open-e4-client pexpect websocket-client
```

# Installation
```bash
cd ~
source /opt/ros/humble/setup.bash
git clone https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors.git
```

# Usage

*Determine RFCOMM channel*

In our case, it is `6`
```
bluetoothctl
sdptool browse <MAC-ADDRESS>
```

*Establish Bluetooth Connection, and build and run Docker Container*

The Dcont is built including `-v $(pwd)/src:/shimmer3/src \`, mounting the `src` folder on the host's computer to the Dcont's running `src` folder, so any edits made to files in that folder (packages) will be reflected and updated.
```
python3 start.py
```

*Virtual Environment, Build and Source etc.* 

Copy paste this into the bash after running `python3 start.py`.
(There is probably a better and simple way to incorporate this into the Dockerfile, but I was not able to work it out successfully:'))
```
source /opt/venv/bin/activate # source virtual environment
source /opt/ros/jazzy/setup.bash # source ROS2
sudo chown -R ros:ros /shimmer3/install /shimmer3/build /shimmer3/log
colcon build --merge-install && source install/setup.bash
source /shimmer3/install/setup.bash # source workspace
export PS1="\[\e[38;2;13;183;237m\][CONTAINER \u@\h \w]\$ \[\e[0m\]" # recolor the container prompt to Docker blue for convenience and ✨aeshetic✨
```

*Launch Package*

The above commands do not launch the shimmer3 package. You can do this manually from within the container.
Launch Parameters:
```
ros2 launch shimmer3_pkg shimmer3.launch.py # launch file
```

*To stop and remove ALL containers*
```
deactivate # venv
sudo docker stop $(sudo docker ps -q)
sudo docker rm $(sudo docker ps -a -q)
```
