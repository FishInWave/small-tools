#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
    
int user_data;
    
    
int 
main (int argc, char** argv)
{
    std::string pcd_name;
    if(argc >= 2)
    {
        pcd_name = argv[1];
    }
    else
    {
        PCL_WARN("You must add a pcd file name");
        return 0;
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile (pcd_name, *cloud);
    
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    cloud->points.at(10).intensity = 20;
    pcl::console::TicToc tt;

    std::cout << cloud->points.at(10) << std::endl;
    std::cout << cloud->points.at(10).getVector4fMap() << std::endl;

    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
    
    while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
    }
    return 0;
}