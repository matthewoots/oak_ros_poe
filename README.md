
## Dependencies

1. There is no need to clone the official OAK ROS wrapper
2. The only required dependency is compiled and installed `depthai_core`. The installed directory is to be changed with in the `CMakeLists.txt`, declared as `depthai_DIR`

```
set(depthai_DIR "/home/$ENV{USER}/git/depthai-core/build/install/lib/cmake/depthai")
```

Other ROS packages needed
```
sudo apt install ros-noetic-pcl-conversions ros-noetic-pcl-ros
```

## Runing the Node
Automatic dicovery of all connected OAK devices and expose the stereo streams as ROS topics in the format `/oak{#no}/[left/right]`
```bash
rosrun oak_ros oak_ros
```
