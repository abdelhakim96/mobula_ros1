/**
 * @file   gp_wind_regression_main.cpp
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

#include <gp_wind_regression/gp_wind_regression_main.h>

int main(int argc, char** argv)
{
    /// Modify ROS severity to display messages starting at DEBUG level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "gp_wind_regression");
    ros::NodeHandle node("~");

    /// Instantiate GP model node
    wind_regression::GPWindRegressionNode gp_wind_regression_node(node);
    /// Run the node
    gp_wind_regression_node.runNode();

    return 0;
}
