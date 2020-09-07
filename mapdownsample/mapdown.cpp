/**
 * @file mapdown.cpp
 * @author your name (you@domain.com)
 * @brief 
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

Eigen::Vector3i minbox;
Eigen::Vector3i maxbox;
Eigen::Vector3i divisionbox;
vector<int> leaflayout;
bool enable_below_detect(false);
float leafsize = 0.05;
float pf_height_min = 0.1;
float pf_height_max = 0.9;
unsigned int mode = 1;

enum GRID
{
    OCCUPY,
    FREE,
    UNKNOW
};

GRID calGrid(int, int);
GRID calGridFloor(int x, int y);
GRID calGridBelow(int x, int y);
GRID calGridBelowWithoutFloor(int, int);
void filters(PointCloud::Ptr, PointCloud::Ptr);
void parseYamlFile();

int main(int argc, char **argv)
{
    std::string pcd_name, out_name;
    if (argc >= 3)
    {
        pcd_name = argv[1];
        out_name = argv[2];
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

    pcl::io::savePCDFile(out_name + ".pcd", *cloud_filtered);

    //打印
    string mapfile = out_name + ".pgm";
    FILE *out = fopen(mapfile.c_str(), "w");
    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n", leafsize, divisionbox[0], divisionbox[1]);
    for (unsigned int y = 0; y < divisionbox[1]; y++)
    {
        for (unsigned int x = 0; x < divisionbox[0]; x++)
        {
            GRID grid;
            switch (mode)
            {
            case 1:
                grid = calGrid(x, divisionbox[1] - y - 1);
                break;
            case 2:
                grid = calGridBelowWithoutFloor(x, divisionbox[1] - y - 1);
                break;
            case 3:
                grid = calGridBelow(x, divisionbox[1] - y - 1);
                break;
            default:
                break;
            }

            switch (grid)
            {
            case OCCUPY:
                fputc(000, out);
                break;
            case FREE:
                fputc(254, out);
                break;
            case UNKNOW:
                fputc(205, out);
                break;

            default:
                break;
            }
        }
    }
    fclose(out);
    string yamlname = out_name + ".yaml";
    FILE *yaml = fopen(yamlname.c_str(), "w");
    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            mapfile.c_str(), leafsize, leafsize * minbox[0], leafsize * minbox[1], 0.0);
    fclose(yaml);

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
 * @brief 仅考虑地面上空的占有
 * 
 * @param x 
 * @param y 
 * @return GRID 
 */
GRID calGrid(int x, int y)
{
    static int m = divisionbox[0] * divisionbox[1];
    static int n = divisionbox[0];
    int count = 0;
    for (size_t z = 0; z < divisionbox[2]; z++)
    {
        if (leaflayout.at(z * m + y * n + x) != -1)
        {
            count++;
        }
    }
    float occ = static_cast<float>(count) / divisionbox[2];
    if (occ >= 0.1)
        return OCCUPY;
    else if (occ <= 0.036)
        return FREE;
    else
        return UNKNOW;
}

/**
 * @brief 计算0上下各两个格子视为floor，并考虑地面上方的占有。
 * 
 * @param x 
 * @param y 
 * @return GRID 
 */
GRID calGridFloor(int x, int y)
{
    static int m = divisionbox[0] * divisionbox[1];
    static int n = divisionbox[0];
    int count = 0;
    bool floor(false); //floor为true说明，是有地板的。
    static int z_floor_max = 2 * abs(minbox[2]) + 1;
    static int z_max = divisionbox[2];
    for (size_t z = z_floor_max; z < z_max; z++)
    {
        if (leaflayout.at(z * m + y * n + x) != -1)
        {
            count++;
        }
    }
    float occ = static_cast<float>(count) / (z_max - z_floor_max);
    for (size_t z = 0; z < z_floor_max; z++)
    {
        if (leaflayout.at(z * m + y * n + x) != -1)
        {
            floor = true;
            break;
        }
    }

    if (occ >= 0.1)
        return OCCUPY;
    else if (occ <= 0.036 && floor)
        return FREE;
    else
        return UNKNOW;
}
/**
 * @brief 不考虑floor，而是被floor隔开的两部分
 * 
 * @param x 
 * @param y 
 * @return GRID 
 */
GRID calGridBelowWithoutFloor(int x, int y)
{
    static int m = divisionbox[0] * divisionbox[1];
    static int n = divisionbox[0];
    int count = 0;
    bool below(false);
    //地面是+-0.05则为-1，+3
    //地面是+-0.1，则为-2，5
    static int z_floor_min = abs(minbox[2]) - 1;
    static int z_floor_max = z_floor_min + 3;
    static int z_max = divisionbox[2];
    for (size_t z = z_floor_max; z < z_max; z++)
    {
        if (leaflayout.at(z * m + y * n + x) != -1)
        {
            count++;
        }
    }
    float occ = static_cast<float>(count) / (z_max - z_floor_max);
    for (size_t z = 0; z < z_floor_min; z++)
    {
        if (leaflayout.at(z * m + y * n + x) != -1)
        {
            below = true;
            break;
        }
    }

    if (occ >= 0.25 || below)
        return OCCUPY;
    else if (occ <= 0.036)
        return FREE;
    else
        return UNKNOW;
}
/**
 * @brief 考虑below，floor，和上方
 * 
 * @param x 
 * @param y 
 * @return GRID 
 */
GRID calGridBelow(int x, int y)
{
    static int m = divisionbox[0] * divisionbox[1];
    static int n = divisionbox[0];
    int count = 0;
    bool floor(false); //floor为true说明，是有地板的。
    bool below(false);
    //地面是+-0.05则为-1，+3
    //地面是+-0.1，则为-2，5
    static int z_floor_min = abs(minbox[2]) - 1;
    static int z_floor_max = z_floor_min + 3;
    static int z_max = divisionbox[2];
    for (size_t z = z_floor_max; z < z_max; z++)
    {
        if (leaflayout.at(z * m + y * n + x) != -1)
        {
            count++;
        }
    }
    float occ = static_cast<float>(count) / (z_max - z_floor_max);
    for (size_t z = z_floor_min; z < z_floor_max; z++)
    {
        if (leaflayout.at(z * m + y * n + x) != -1)
        {
            floor = true;
            break;
        }
    }
    for (size_t z = 0; z < z_floor_min; z++)
    {
        if (leaflayout.at(z * m + y * n + x) != -1)
        {
            below = true;
            break;
        }
    }

    if (occ >= 0.1 || below)
        return OCCUPY;
    else if (occ <= 0.036 && floor)
        return FREE;
    else
        return UNKNOW;
}
/**
 * @brief 导入yaml文件，并赋值相关变量。
 */
void parseYamlFile()
{
    YAML::Node node = YAML::LoadFile("./config.yaml");
    leafsize = node["voxel"]["leafsize"].as<float>();
    mode = node["mode"].as<unsigned int>();
    pf_height_max = node["passthrough"]["height_max"].as<float>();
    pf_height_min = node["passthrough"]["height_min"].as<float>();
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
    pf.setFilterLimits(pf_height_min, pf_height_max); //只保留小车高度内的点
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
    vf.setLeafSize(leafsize, leafsize, leafsize);
    vf.setSaveLeafLayout(true);
    vf.filter(*output);
    cout << "voxel: " << output->size() << endl;
    minbox = vf.getMinBoxCoordinates();
    maxbox = vf.getMaxBoxCoordinates();
    divisionbox = vf.getNrDivisions();
    leaflayout = vf.getLeafLayout();
    auto size = leaflayout.size();
    cout << "minbox:" << endl;
    cout << minbox << endl;
    cout << "maxbox:" << endl;
    cout << maxbox << endl;
    cout << "divisionbox:" << endl;
    cout << divisionbox << endl;
    cout << "leaflayout size: " << leaflayout.size() << endl;
}

