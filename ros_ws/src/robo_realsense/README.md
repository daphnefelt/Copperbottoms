# Setting up librealsense package on RaspberryPi 4

This README provides instructions for how you can set up your RaspberryPi 4 to interface with the RealSense D435 camera, 
which will be used for high-quality visible image data streaming. The `librealsense` package provides a Python package
`pyrealsense2` that is used to interface with RealSense cameras and send camera data to Desktop stream for testing and Ros2
topics for use in robot automation. 
### Requirements

- Ubuntu 22.04
- USB-C to USB 3.0 Connector Cable for RealSense D435

### Instructions
- RealSense cameras are very expensive (~$300), be careful with them!
- RealSense camera will be plugged into the blue USB ports on your RaspberryPi 4
- The cmake command in the 'Make the `librealsense` build' section will run much faster if you are on wired internet. You can use the
RaspberryPi stations in the basement of the ITLL for this step (they are by the stairs on the side under the skybridge from the main engineering building).
- Install and set up Ros2 Humble before starting this tutorial so that you can interface with Ros2 at the end of the tutorial. You can also
follow this tutorial and use the `desktop_stream.py` file to show the RealSense data stream (visible camera images) on your Desktop,
so it is not strictly necessary to install Ros2 before completing this guide

### Update System
Open your terminal on the Ubuntu desktop. We must first update our system and install the dependencies
we need to build the librealsense package.

```
sudo apt update && sudo apt upgrade -y
```

### Install build dependencies
We will now install the build dependencies for the librealsense package. In the same terminal, enter the following command. 
The `\` character is used in the terminal to continue entering a command on the next line for readability.
```
sudo apt install -y \
    build-essential cmake git pkg-config \
    libusb-1.0-0-dev libudev-dev \
    libssl-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libpsl-dev libidn2-0-dev \
    python3-dev python3-opencv python3-pip
```
### Clone the librealsense repository from GitHub
```
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
```

### Set up udev rules
This allows non-root users to access the camera. Unplug the RealSense camera and run these commands,
then plug the camera back in afterwards.
```
sudo ./scripts/setup_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Make the librealsense build and install tool shortcuts
We are now ready to install the full librealsense package and build it from source. The cmake command will take 
a while to run (40 minutes to an hour over wifi).
```
cd ~/librealsense
rm -rf build third-party/libcurl/libcurl_install
mkdir build && cd build

cmake .. \        
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_TOOLS=ON \
  -DBUILD_EXAMPLES=ON \
  -DCHECK_FOR_UPDATES=OFF    

make -j"$(nproc)"      # this line builds the librealsense package
sudo make install      # this line installs shortcuts like 'realsense-viewer'

```

### Verify SDK
Check that the Python package is installed:
```
python3 -c "import pyrealsense2 as rs; print('pyrealsense2 OK:', rs.__version__)"
```
Check that the camera is detected and that the USB cable is 3.0. This should show 5000M for the USB data stream:
```
lsusb -t
```
Enumerate devices to see the camera listed:
```
rs-enumerate-devices
```
Install the Python dependencies, then run the example python script to see the data stream from the RealSense camera:
```
pip install pyrealsense2
python3 desktop_stream.py
```
If you get any other `module not found` errors in the call for the Python script, simply install the package Python needs using `pip install <package_name>`. The package imports are listed at the beginning of the `ros_stream.py` file.

We now know that the RealSense camera is detected, working properly, and can be interfaced with via Python. Our next step is to 
publish the RealSense data stream to Ros2 so that subscriber nodes can use the data from the RealSense camera.
### Install Ros2 RealSense dependencies
```
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport
```

### Run Ros2 data stream
We can now run a Python file to take data from the RealSense camera and publish it to a topic in Ros2. In your car,
you will eventually create a subscriber node that will subscribe to this topic and use the camera data to 
calculate and deliver control inputs for the car.

First, run the Python script to publish to the topic `/camera/color/image_raw`.
```
python3 ros_stream.py
```

Next, check that the topic shows up in the list of topics in Ros2, and then echo the data to make sure that the topic 
is receiving data. You can use Ctrl+C to kill a terminal process, which will be useful to do after running the `ros2 topic echo` command
and seeing the data stream. 
```
ros2 topic list
ros2 topic echo /camera/color/image_raw --field header
```

Check your frame rate for the topic:
```
ros2 topic hz /camera/color/image_raw
```

If you see the data stream being echoed in the terminal after the `ros2 topic echo` line above, you should be ready to start making Python files to process and utilize your RealSense data now!
