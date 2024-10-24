# `lane_following_cam` package
Lane following based on camera as a ROS 2 python package.  

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
[![Static Badge](https://img.shields.io/badge/ROS_2-Jazzy-34aec5)](https://docs.ros.org/en/jazzy/)

## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/sze-info/lane_following_cam
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select lane_following_cam --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` r
ros2 run lane_following_cam lane_detect
```

``` r
ros2 launch lane_following_cam TODO: launch file
```


### Start the camera

``` r
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ~/ros2_ws/src/drivers/usb_cam-ros2/config/params.yaml
```

