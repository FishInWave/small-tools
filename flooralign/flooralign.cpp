/** 第一个视角是直接用NDT做匹配，第二个视角是删除地面点后做匹配
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
#include <pcl/registration/ndt.h>
//std
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>

int user_data;
using namespace std;
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

Eigen::Vector3i minbox;
Eigen::Vector3i maxbox;
Eigen::Vector3i divisionbox;
vector<int> leaflayout;

float calGrid(int x, int y);

int main(int argc, char **argv)
{
    std::string pcd_name_1, pcd_name_2;
    if (argc >= 3)
    {
        pcd_name_1 = argv[1];
        pcd_name_2 = argv[2];
    }
    else
    {
        PCL_WARN("You must add 2 pcd file name");
        return 0;
    }

    PointCloud::Ptr target_cloud(new PointCloud);
    PointCloud::Ptr source_cloud(new PointCloud);
    PointCloud::Ptr target_cloud_floor(new PointCloud);
    PointCloud::Ptr source_cloud_floor(new PointCloud);
    pcl::io::loadPCDFile(pcd_name_1, *target_cloud);

    // target_cloud->width = target_cloud->width * target_cloud->height;
    // target_cloud->height = 1;
    // target_cloud->resize(target_cloud->width * target_cloud->height);

    pcl::io::loadPCDFile(pcd_name_2, *source_cloud);
    // source_cloud->width = source_cloud->width * source_cloud->height;
    // source_cloud->height = 1;
    // source_cloud->resize(source_cloud->width * source_cloud->height);
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, index);
    pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, index);
    //SLAM时已经考虑了激光雷达高度了，因此滤波时，直接按照实际高度删即可
    const float h_vehicle = 0.9226; //测于scout1.0,雷达中心
    const float h_tolerance = 0.3;

    cout << "1" << endl;
    //高程滤波
    pcl::PassThrough<PointT> pf(false); //false表示不想管被删除的索引
    pf.setInputCloud(target_cloud);
    pf.setFilterFieldName("z");
    pf.setFilterLimits(0.1, 4); //只保留小车高度内的点
    pf.filter(*target_cloud_floor);
    pf.setInputCloud(source_cloud);
    pf.filter(*source_cloud_floor);

    cout << "2" << endl;
    PointCloud cloud;

    cout << "target_cloud: " << target_cloud->size() << endl;
    cout << "width: " << target_cloud->width << " height: " << target_cloud->height << " organized: " << target_cloud->isOrganized() << endl;
    cout << "source_cloud: " << source_cloud->size() << endl;
    cout << "width: " << source_cloud->width << " height: " << source_cloud->height << " organized: " << source_cloud->isOrganized() << endl;
    cout << "target_cloud_floor: " << target_cloud_floor->size() << endl;
    cout << "width: " << target_cloud_floor->width << " height: " << target_cloud_floor->height << " organized: " << target_cloud_floor->isOrganized() << endl;
    cout << "source_cloud_floor: " << source_cloud_floor->size() << endl;
    cout << "width: " << source_cloud_floor->width << " height: " << source_cloud_floor->height << " organized: " << source_cloud_floor->isOrganized() << endl;
    //统计学滤波
    pcl::StatisticalOutlierRemoval<PointT> sf;
    sf.setMeanK(25);
    sf.setStddevMulThresh(0.5);
    sf.setInputCloud(target_cloud);
    // pcl::console::TicToc tt;
    // tt.tic();
    sf.filter(*target_cloud);
    cout << "2.1" << endl;
    // cout << "statistic spend " << tt.toc() << " ms" << endl;
    sf.setInputCloud(source_cloud);
    sf.filter(*source_cloud);
    cout << "2.2" << endl;
    sf.setInputCloud(target_cloud_floor);
    sf.filter(*target_cloud_floor);
    cout << "2.3" << endl;
    sf.setInputCloud(source_cloud_floor);
    sf.filter(*source_cloud_floor);
    cout << "3" << endl;
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setTransformationEpsilon(0.1);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(30);
    ndt.setInputTarget(target_cloud);
    ndt.setInputSource(source_cloud);
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    // Eigen::Matrix4f init_guess;
    // init_guess << 0.93, 0.35, -0.01, 0.43, -0.35, -0.93, 0.00, -0.03, 0.00, 0.00, 1, 0.02, 0, 0, 0, 1;
    PointCloud::Ptr sum_cloud(new PointCloud());
    ndt.align(*sum_cloud, init_guess);
    auto Transform = ndt.getFinalTransformation();
    cout << "with floor:" << Transform << endl;
    *sum_cloud += *target_cloud;
    //     with floor:
    //    0.933347     0.358845  -0.00967435     0.435304
    //    -0.358823     0.933397   0.00395579   -0.0388212
    //    0.0104495 -0.000220739     0.999945    0.0153533
    //            0            0            0            1
    cout << "4" << endl;
    PointCloud::Ptr sum_cloud_floor(new PointCloud());
    ndt.setInputTarget(target_cloud_floor);
    ndt.setInputSource(source_cloud_floor);
    ndt.align(*sum_cloud_floor, init_guess);
    auto Transform_floor = ndt.getFinalTransformation();
    cout << "with floor:" << Transform_floor << endl;
    *sum_cloud_floor += *target_cloud_floor;
    // with floor:
    // 0.934451    0.355998  -0.0081436    0.437691
    //   -0.356018    0.934478 -0.00118113  -0.0358165
    //  0.00718954  0.00400298    0.999966   0.0425516
    //           0           0           0           1
    cout << "have already finished" << endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    cout << "5" << endl;
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("with floor: ", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(sum_cloud, 0, 255, 0);
    viewer->addPointCloud<PointT>(sum_cloud, single_color, "cloud1", v1);
    pcl::io::savePCDFileBinary("floor.pcd", *sum_cloud);
    int v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("without floor: ", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> mul_color(sum_cloud, 255, 0, 0);
    viewer->addPointCloud<PointT>(sum_cloud_floor, mul_color, "cloud2", v2);
    pcl::io::savePCDFileBinary("without.pcd", *sum_cloud_floor);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");
    viewer->addCoordinateSystem(1.0);
    while (!viewer->wasStopped())
    {
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
