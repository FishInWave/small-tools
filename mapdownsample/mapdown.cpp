/** 这个代码可以作为学习体素滤波的资料。
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

int user_data;
using namespace std;
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

enum GRID
{
    OCCUPY,
    FREE,
    UNKNOW
};

Eigen::Vector3i minbox;
Eigen::Vector3i maxbox;
Eigen::Vector3i divisionbox;
vector<int> leaflayout;
bool enable_below_detect(false);

float calGrid(int x, int y);
GRID calGridFloor(int x, int y);
GRID calGridBelow(int x, int y);

int main(int argc, char **argv)
{
    std::string pcd_name;
    if (argc >= 2)
    {
        pcd_name = argv[1];
    }
    else
    {
        PCL_WARN("You must add a pcd file name");
        return 0;
    }

    PointCloud::Ptr cloud(new PointCloud);
    pcl::io::loadPCDFile(pcd_name, *cloud);

    //SLAM时已经考虑了激光雷达高度了，因此滤波时，直接按照实际高度删即可
    const float h_vehicle = 0.9226; //测于scout1.0,雷达中心
    const float h_tolerance = 0.3;
    cout << "original: " << cloud->size() << endl;

    //高程滤波
    pcl::PassThrough<PointT> pf(false); //false表示不想管被删除的索引
    pf.setInputCloud(cloud);
    pf.setFilterFieldName("z");
    //不考虑floor，（0.1,0.9），调用calGrid
    //只考虑floor，（-0.1,0.9），调用calGridFloor
    //考虑floor和below，（-0.4,0.9）,调用calGridBelow
    pf.setFilterLimits(-0.4, 0.9); //只保留小车高度内的点
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

    const float leafsize = 0.05;
    //体素化
    pcl::VoxelGrid<PointT> vf;
    vf.setInputCloud(cloud_spf);
    vf.setLeafSize(leafsize, leafsize, leafsize);
    PointCloud::Ptr cloud_vpf(new PointCloud);
    vf.setSaveLeafLayout(true);
    vf.filter(*cloud_vpf);
    cout << "voxel: " << cloud_vpf->size() << endl;
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

    string mapname = "map_out";
    pcl::io::savePCDFile(mapname + ".pcd", *cloud_vpf);

    //打印
    string mapfile = mapname + ".pgm";
    FILE *out = fopen(mapfile.c_str(), "w");
    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n", leafsize, divisionbox[0], divisionbox[1]);
    for (unsigned int y = 0; y < divisionbox[1]; y++)
    {
        for (unsigned int x = 0; x < divisionbox[0]; x++)
        {
            // float occ = calGrid(x, divisionbox[1] - y - 1);
            // if (occ <= 0.036)
            //     fputc(254, out);
            // else if (occ >= 0.1)
            //     fputc(000, out);
            // else
            //     fputc(205, out);
            GRID grid = calGridBelow(x, divisionbox[1] - y - 1);
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
    string yamlname = mapname + ".yaml";
    FILE *yaml = fopen(yamlname.c_str(), "w");
    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            mapfile.c_str(), leafsize, leafsize * minbox[0], leafsize * minbox[1], 0.0);
    fclose(yaml);

    cout << "have already finished" << endl;
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(cloud_vpf);

    while (!viewer.wasStopped())
    {
        user_data++;
    }
    return 0;
}

//计算第（x,y）体素格子对应的value
float calGrid(int x, int y)
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
    return occ;
}

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

GRID calGridBelow(int x, int y)
{
    static int m = divisionbox[0] * divisionbox[1];
    static int n = divisionbox[0];
    int count = 0;
    bool floor(false); //floor为true说明，是有地板的。
    bool below(false);
    static int z_floor_min = abs(minbox[2]) - 2;
    static int z_floor_max = z_floor_min + 5;
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