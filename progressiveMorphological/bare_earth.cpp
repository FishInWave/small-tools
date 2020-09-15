 /**
 * @file bare_earth.cpp
 * @author Yu-wei XU
 * @brief 官方例程，修改了一些参数，用于拟合出地面。
 * 
 * @version 0.1
 * @date 2020-09-10
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

int main(int argc, char **argv)
{
  std::string pcd_name;
  if (argc >= 2)
  {
    pcd_name = argv[1];
  }else
  {
    PCL_WARN("Youi must add a pcd file name and an out name");
    return 0;
  }
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground(new pcl::PointIndices);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ>(pcd_name, *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud(cloud);
  pmf.setMaxWindowSize(5);
  pmf.setSlope(1.0f);
  pmf.setInitialDistance(0.1f);
  pmf.setMaxDistance(0.5f);
  pmf.extract(ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(ground);
  extract.filter(*cloud_filtered);

  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("samp11-utm_ground.pcd", *cloud_filtered, false);

  // Extract non-ground returns
  extract.setNegative(true);
  extract.filter(*cloud_filtered);

  std::cerr << "Object cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  writer.write<pcl::PointXYZ>("samp11-utm_object.pcd", *cloud_filtered, false);

  return (0);
}