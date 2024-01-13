/**
 * @file   utils.cpp
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

#include <gp_wind_regression/utils.h>

namespace wind_regression
{
namespace utils
{
double* getarrayFromStdVector(const std::vector<double>& vector)
{
    double* array = new double[vector.size()];
    std::copy(vector.begin(), vector.end(), array);
    return array;
}

double meanFilter(double val, std::vector<double>& data_unfiltered)
{
    // Erase from the front!
    data_unfiltered.erase(data_unfiltered.begin());
    data_unfiltered.push_back(val);

    double new_val = 0.0;
    for (int i = 0; i < data_unfiltered.size(); ++i)
        new_val += data_unfiltered[i];
    new_val = new_val / (data_unfiltered.size());

    return new_val;
}

void initMultiArrayMessage(std_msgs::Float64MultiArray& msg, const int& size, const std::string& label)
{
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = size;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = label;
}

void extractMultiArrayMsg(std::vector<double>& vector,
                          const std::vector<double>::const_iterator& msg_begin,
                          const std::vector<double>::const_iterator& msg_end)
{
    vector.clear();
    for (std::vector<double>::const_iterator itr = msg_begin; itr != msg_end; ++itr)
    {
        vector.push_back(*itr);
    }
}

int checkAndCreateDirectory(const char* path)
{
    struct stat info;
    if (stat(path, &info) != 0)
    {
        if (mkdir(path, 0777) != 0)
        {
            return 0;
        }
        else
        {
            return 2;
        }
    }
    else if (info.st_mode & S_IFDIR)  // S_ISDIR() doesn't exist on my windows
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

bool isFileExist(const std::string& file)
{
    std::ifstream in_file(file);
    return in_file.good();
}

void printStdVector(const std::vector<double>& data, const std::string& name)
{
    if (not name.empty())
    {
        std::cout << name << " = ";
    }
    else
    {
        std::cout << "data = ";
    }
    for (size_t idx = 0; idx < data.size(); idx++)
    {
        std::cout << data.at(idx) << ", ";
    }
    std::cout << "\n";
}

}  // namespace utils
}  // namespace wind_regression
