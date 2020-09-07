/**
 * @file removeDynamic.cpp
 * @author Yu-wei XU
 * @brief  使用聚类的方法找出并删除地图中的移动物体
 * @version 0.1
 * @date 2020-09-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

//pcl
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/sac_segmentation.h>
//std
#include <cstdio>
#include <fstream>
#include <iostream>
//third party
#include "yaml-cpp/yaml.h"

int user_data;
using namespace std;
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

void filters(PointCloud::Ptr, PointCloud::Ptr);
void parseYamlFile();

int main(int argc, char **argv)
{
    std::string pcd_name, out_name;
    if (argc >= 2)
    {
        pcd_name = argv[1];
    }
    else
    {
        PCL_WARN("You must add a pcd file name and an out name");
        return 0;
    }

    PointCloud::Ptr cloud(new PointCloud);
    pcl::io::loadPCDFile(pcd_name, *cloud);

    cout << "original: " << cloud->size() << endl;

    parseYamlFile();
    PointCloud::Ptr cloud_filtered(new PointCloud);
    filters(cloud, cloud_filtered);

    cout << "have already finished" << endl;
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(cloud_filtered);

    while (!viewer.wasStopped())
    {
        user_data++;
    }
    return 0;
}

/**
 * @brief 导入yaml文件，并赋值相关变量。
 */
void parseYamlFile()
{
    YAML::Node node = YAML::LoadFile("./config.yaml");

}
/**
 * @brief 包含了高程滤波，离散点滤波和体素滤波。
 * 
 * @param input 
 * @param output 
 */
void filters(PointCloud::Ptr input, PointCloud::Ptr output)
{
    //高程滤波
    pcl::PassThrough<PointT> pf(false); //false表示不想管被删除的索引
    pf.setInputCloud(input);
    pf.setFilterFieldName("z");
    // pf.setFilterLimits(pf_height_min, pf_height_max); //只保留小车高度内的点
    PointCloud::Ptr cloud_pf(new PointCloud);
    pf.filter(*cloud_pf);

    cout << "height " << cloud_pf->size() << endl;
    //统计学滤波
    pcl::StatisticalOutlierRemoval<PointT> sf;
    sf.setMeanK(25);
    sf.setStddevMulThresh(0.5);
    sf.setInputCloud(cloud_pf);
    PointCloud::Ptr cloud_spf(new PointCloud);
    pcl::console::TicToc tt;
    tt.tic();
    sf.filter(*cloud_spf);
    cout << "statistic spend " << tt.toc() << " ms" << endl;
    cout << "statistic " << cloud_spf->size() << endl;

    //体素化,目的是为了建立索引
    pcl::VoxelGrid<PointT> vf;
    vf.setInputCloud(cloud_spf);
    // vf.setLeafSize(leafsize, leafsize, leafsize);
    vf.setSaveLeafLayout(true);
    vf.filter(*output);
    cout << "voxel: " << output->size() << endl;
    // minbox = vf.getMinBoxCoordinates();
    // maxbox = vf.getMaxBoxCoordinates();
    // divisionbox = vf.getNrDivisions();
    // leaflayout = vf.getLeafLayout();
    // auto size = leaflayout.size();
    // cout << "minbox:" << endl;
    // cout << minbox << endl;
    // cout << "maxbox:" << endl;
    // cout << maxbox << endl;
    // cout << "divisionbox:" << endl;
    // cout << divisionbox << endl;
    // cout << "leaflayout size: " << leaflayout.size() << endl;
}

std::pair<PointCloud::Ptr, PointCloud::Ptr> SegmentPlane(PointCloud::Ptr cloud, 
                                        int maxIterations, float distanceThreshold)
{
    // 记录处理所需时间
    pcl::console::TicToc tt;
    tt.tic();

    // 调用PCL平面分割的函数
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    // 指定处理的算法
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.sud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "could not estimate a planar model for the given dataset." << std::endl;
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
