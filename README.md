# Start: simulation or real-brov?
```bash
checkout Mobula branch: Mobula_External_Controller_AHA
```
 ## Running on simulated bluerov

  Create ROVType.txt file inside bin_Debug folder
  write the word "Simulator" on the first line

 ## Running on real bluerov

  remove the word Simulator from  ROVType.txt
 
 Inside MobularosManager us MavlinkIMUAcceleration
```bash
 	private void PublishLinearAcceleration()
		{
			EMatrix linearAccelerations = new EMatrix(3, 1)
			{
				status.MavlinkIMUAccelerationX,
				status.MavlinkIMUAccelerationY,
				status.MavlinkIMUAccelerationZ
				//status.IMURawAccelerationX,
				//status.IMURawAccelerationY,
				//status.IMURawAccelerationZ
			};
```



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


```bash
ros2 run mhe_lambda mhe_x_node 
```
run all three nodes x ,y and z
## Running the Nominal-GP
```bash
roslaunch gp_wind_regression gp_wind_regression_single.launch
```


## Running the DF-GP

```bash
roslaunch gp_wind_regression gp_wind_regression_combined.launch
```

and run ros2 nodes


