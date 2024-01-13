# Wind GP Regression

This repository presents the implementation of a method proposed in our paper. The method utilizes Gaussian Process (GP) regression to estimate the wind-induced forces acting on a drone in the body-fixed frame. The GP implementation is developed using the open-source library [LIBGP](https://github.com/mblum/libgp). The method employs two concatenated GP architectures: GP1, representing the initial GP, is established based on the initially recorded data. GP2, an online GP, continually updates samples with incoming data, replacing the 10 oldest samples with the 10 most recent data points. This architecture is independently implemented for each axis. Consequently, a dedicated GP is employed for learning the disturbances along each body axis of the drone.


![Concatenated GP Architecture](gp_wind_regression/conc_GP.png)

## Inputs

Each GP utilizes the feature data, which includes regressors and corresponding output pairs. It then generates predictions for the force at the next time step along its respective axis (x, y, and z).

Topics for features that the GP subscribes to:

- `/dev/gp/feature_x_filtered` (Type: `std_msgs/Float64MultiArray`) - Input Features for GP-x
- `/dev/gp/feature_y_filtered` (Type: `std_msgs/Float64MultiArray`) - Input Features for GP-y
- `/dev/gp/feature_z_filtered` (Type: `std_msgs/Float64MultiArray`) - Input Features for GP-z

## Outputs

Topics that the GP outputs:

- `/gp_disturb_reg/mu/x` (Type: `std_msgs/Float64MultiArray`) - Prediction of GP-x
- `/gp_disturb_reg/mu/y` (Type: `std_msgs/Float64MultiArray`) - Prediction of GP-y
- `/gp_disturb_reg/mu/z` (Type: `std_msgs/Float64MultiArray`) - Prediction of GP-z

## Important Launch File Parameters

The GP has several parameters that can affect its performance. These parameters can be found in `gp_wind_regression/launch/common_params.xml`:

- `max_initial_rec_time_sec` Initial recording time window for training GP1
- `num_window_points_past` Number of window points in the past
- `num_predict_points_future` Number of prediction points in the future
- `gp_input_data_type` GP input data type (0 for type0, 1 for type1)
- `feature_data_model_type` Feature data model type (0 for subscribed, 1 for compute)
- `use_model_difference` Whether to use model difference values in feature data
- `use_states` Whether to use state values in feature data
- `use_controls` Whether to use control values in feature data
- `num_features` Number of features (only for feature_data_model_type = 0)
- `data_history` Input data history


## Launch Instructions

To launch each GP separately:
```bash
roslaunch gp_wind_regression gp_wind_regression_x.launch
```

```bash
roslaunch gp_wind_regression gp_wind_regression_y.launch
```
```bash
roslaunch gp_wind_regression gp_wind_regression_z.launch
```

To launch all three GPs simultaneously:

```bash
roslaunch gp_wind_regression gp_wind_regression_combined.launch
```



