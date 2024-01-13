/**
 * @file   utils.h
 * @author Mohit Mehndiratta
 * @date   Sep 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

/**
 * @brief  utilities
 */

#ifndef _UTILS_H_
#define _UTILS_H_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Bad headers with problem go here
#include <std_msgs/Float64MultiArray.h>
// restore compiler switches
#pragma GCC diagnostic pop

#include <fstream>
#include <iostream>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>

namespace wind_regression
{
namespace utils
{
double* getarrayFromStdVector(const std::vector<double>& vector);
double meanFilter(double val, std::vector<double>& data_unfiltered);

void initMultiArrayMessage(std_msgs::Float64MultiArray& msg, const int& size, const std::string& label = "");

void extractMultiArrayMsg(std::vector<double>& vector,
                          const std::vector<double>::const_iterator& msg_begin,
                          const std::vector<double>::const_iterator& msg_end);

int checkAndCreateDirectory(const char* path);

bool isFileExist(const std::string& file);

void printStdVector(const std::vector<double>& data, const std::string& name = "");
}  // namespace utils
}  // namespace wind_regression

#endif
