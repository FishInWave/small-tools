/**
 * @file a.cpp
 * @author Yu-wei XU
 * @brief 用于验证erase的机制。
 * @version 0.1
 * @date 2020-09-16
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>(5, 1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the CloudIn data
    for (auto &point : *cloud_in)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cout << "Saved " << cloud_in->size() << " data points to input:" << std::endl;

    for (auto &point : *cloud_in)
        std::cout << point << std::endl;

    *cloud_out = *cloud_in;
    cloud_out->erase(cloud_out->begin() + 2);
    cloud_out->erase(cloud_out->begin()+3);
    std::cout << "size:" << cloud_out->size() << std::endl;

    for (auto &point : *cloud_out)
        std::cout << point << std::endl;

    return (0);
}