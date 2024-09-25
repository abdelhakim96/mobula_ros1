
# Installation Guide

## 1. Install Visual Studio (VS 2022)
Ensure that you have all the necessary libraries installed.
Follow this guide for installation:
`file://eiva.local/data/Software/IT/Guides/HowTo%20-%20Install%20compilers%20and%20build%20tools%20VS2019.pdf`

## 2. Install Boost Library
- Download Boost from: `\\eiva.local\Data\Software\Util\Boost\boost_1_66_0_VS2017.zip`
- Extract the folder `boost_1_66_0` to `C:\Boost`.

## 3. Install Redistributables
Download and install the following redistributables:
- [x86 Redistributable](https://download.microsoft.com/download/1/6/5/165255E7-1014-4D0A-B094-B6A430A6BFFC/vcredist_x86.exe)
- [x64 Redistributable](https://download.microsoft.com/download/1/6/5/165255E7-1014-4D0A-B094-B6A430A6BFFC/vcredist_x64.exe)

## 4. Add Database Files
Copy the following database files to `C:\temp`:
- `diffAdditiveDBTmp.db`
- `diffDBTmp.db`

## 5. Add ROV Control Parameters
Copy the control parameters file `cntrl_prms.txt` to the following path:
```bash
C:\ProgramData\EIVA\Mobula\MobulaConfigurationFiles\BlueROV2
```

## 6. Checkout Mobula branch

```bash
checkout Mobula branch using SVN: https://heket.eiva.com/svn/NaviSuite/features/Mobula/Mobula_External_Controller_AHA/
```





# Start: simulation or real brov?
```bash
checkout Mobula branch using SVN: https://heket.eiva.com/svn/NaviSuite/features/Mobula/Mobula_External_Controller_AHA/
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
				status.MavlinkIMUAccelerationX,    //for real
				status.MavlinkIMUAccelerationY,    
				status.MavlinkIMUAccelerationZ     
				//status.IMURawAccelerationX,       
				//status.IMURawAccelerationY,
				//status.IMURawAccelerationZ
			};
```






##  Install some ros dependencies

```bash
sudo apt-get install python3-catkin-tools
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-msgs

```


## build gp library
navigate to the `libgp_sparse` folder and create a `build` directory.

```bash
cd libgp_sparse
mkdir -p build
cd build
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


