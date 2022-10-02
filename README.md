# compressed-realsense-rgbd-videos-transport
Transport compressed Realsense RGBD videos using ROS image_transport to reduce latency.

## Do the following three steps on both publisher and receiver

### Install dependencies
#### OpenCV
`sudo apt install libopencv-dev`
#### ROS
https://www.ros.org/
#### Realsense SDK
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms -y
sudo apt-get install librealsense2-utils -y
sudo apt-get install librealsense2-dev -y
sudo apt-get install librealsense2-dbg -y
```

### Create ROS workspace and download this package
```
mkdir -p ~/video_transport/src
cd video_transport/src
catkin_init_workspace 
cd ..
catkin_make
cd src
git clone https://github.com/AreteQin/compressed-realsense-rgbd-videos-transport.git
```

### Make
```
cd ..
catkin build
```
### Run the launch file
```
roslaunch compressed_realsense_rgbd_videos_transport compressed_rgbd_transport.launch
```
