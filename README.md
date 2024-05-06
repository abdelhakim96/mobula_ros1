# Running the simulation

## Run ROS-Bridge
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

## in ROS1 run 
```bash
roslaunch bluerov2_trajectory test.launch 
```

```bash
roslaunch bluerov2_nmpc bluerov2_nmpc.launch 
```

```bash
roslaunch gp_wind_regression gp_wind_regression_combined.launch
```

## in ROS2

```bash
source /opt/ros/foxy/setup.bash
```

```bash
cd ros2_ws/
```

```bash
source install/local_setup.sh
```


