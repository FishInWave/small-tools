/** @brief 这个程序宣告失败，我至今无法理解。
*/
//std
#include <iostream>
#include <math.h>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
//other
#include "annobox.hpp"
#include <jsoncpp/json/json.h>
//通过Eigen::vector3f给点赋值
void valuePoints(pcl::PointXYZI& point,Eigen::Vector4f val)
{
    point.x = val[0];
    point.y = val[1];
    point.z = val[2];
    point.intensity = val[3];
}

int main (int argc, char** argv)
{
    //自定义点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_ptr->width=10;
    cloud_ptr->height=1;
    cloud_ptr->resize(cloud_ptr->width*cloud_ptr->height);
    // for (size_t i = 0; i < cloud_ptr->points.size(); i++)
    // {
    //     cloud_ptr->points[i].x=64.0f*rand()/(RAND_MAX+1.0f);
    //     cloud_ptr->points[i].y=64.0f*rand()/(RAND_MAX+1.0f);
    //     cloud_ptr->points[i].z=64.0f*rand()/(RAND_MAX+1.0f);
    //     if(cloud_ptr->points[i].x>-3 && cloud_ptr->points[i].x<3)
    //         {
    //             std::cout << cloud_ptr->points[i].x << " " << cloud_ptr->points[i].y << " "<< cloud_ptr->points[i].z << std::endl;
    //         }
    // }
    Eigen::Vector4f val1(-1,-1,-1,25);
    Eigen::Vector4f val2(1,1,1,25);
    Eigen::Vector4f val3(2,2,2,55);
    Eigen::Vector4f val4(1,2,5,55);
    Eigen::Vector4f val5(5,1,5,55);
    Eigen::Vector4f val6(1,6,6,255);
    Eigen::Vector4f val7(0,0,0,255);
    Eigen::Vector4f val8(3,3,3,255);
    Eigen::Vector4f val9(10,10,10,255);
    Eigen::Vector4f val10(-10,-10,-10,100);
    valuePoints(cloud_ptr->points[0],val1);
    valuePoints(cloud_ptr->points[1],val2);
    valuePoints(cloud_ptr->points[2],val3);
    valuePoints(cloud_ptr->points[3],val4);
    valuePoints(cloud_ptr->points[4],val5);
    valuePoints(cloud_ptr->points[5],val6);
    valuePoints(cloud_ptr->points[6],val7);
    valuePoints(cloud_ptr->points[7],val8);
    valuePoints(cloud_ptr->points[8],val9);
    valuePoints(cloud_ptr->points[9],val10);
    std::cout << "original points number: " << cloud_ptr->points.size() << std::endl;
    //CropBox
    Eigen::Vector4f min_point(0,0,0,-1.0e-6);
    Eigen::Vector4f max_point(3,3,3,1.0e6);

    pcl::CropBox<pcl::PointXYZI> clipper;
    pcl::PointCloud<pcl::PointXYZI> cloud_final;
    clipper.setMin(min_point);
    clipper.setMax(max_point);
    clipper.setInputCloud(cloud_ptr);
    clipper.setNegative(false);
    clipper.filter(cloud_final);
    for (size_t i = 0; i < cloud_final.size(); i++)
    {
        std::cout << cloud_final.points[i].x << " " << cloud_final.points[i].y << " "<< cloud_final.points[i].z << " "<< cloud_final.points[i].intensity << std::endl;
        std::cout << "condition: "<< std::endl;
    }
    
    //condition
    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond_x(new pcl::ConditionOr<pcl::PointXYZI>());
    range_cond_x->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison
        <pcl::PointXYZI>("x",pcl::ComparisonOps::GT,3.0)));
    range_cond_x->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison
        <pcl::PointXYZI>("x",pcl::ComparisonOps::LT,0)));
    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond_y(new pcl::ConditionOr<pcl::PointXYZI>());
    range_cond_y->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison
        <pcl::PointXYZI>("y",pcl::ComparisonOps::GT,3.0)));
    range_cond_y->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison
        <pcl::PointXYZI>("y",pcl::ComparisonOps::LT,0)));
    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond_z(new pcl::ConditionOr<pcl::PointXYZI>());
    range_cond_z->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison
        <pcl::PointXYZI>("z",pcl::ComparisonOps::GT,3.0)));
    range_cond_z->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison
        <pcl::PointXYZI>("z",pcl::ComparisonOps::LT,0)));
    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZI>());
    range_cond->addCondition(range_cond_x);
    range_cond->addCondition(range_cond_y);
    range_cond->addCondition(range_cond_z);


    pcl::ConditionalRemoval<pcl::PointXYZI> range_filter;
    range_filter.setCondition(range_cond);
    range_filter.setKeepOrganized(false);
    range_filter.setInputCloud(cloud_ptr);
    pcl::PointCloud<pcl::PointXYZI> cloud_cond;
    range_filter.filter(cloud_cond);
    
    for (size_t i = 0; i < cloud_cond.points.size(); i++)
    {
        std::cout << cloud_cond.points[i].x << " " << cloud_cond.points[i].y << " "<< cloud_cond.points[i].z << " " << cloud_cond.points[i].intensity<< std::endl;
    }

}
