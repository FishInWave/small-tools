/** @brief 这个程序宣告失败，我至今无法理解。
*/
//std
#include <iostream>
#include <math.h>
#include <thread>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/visualization/pcl_visualizer.h>
//other
#include "annobox.hpp"
#include <jsoncpp/json/json.h>
int v1(0);
int v2(0);

pcl::PointCloud<pcl::PointXYZI>::Ptr removeCondition(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                                     Eigen::Vector4f minpoint, Eigen::Vector4f maxpoint)
{

    //condition
    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond_x(new pcl::ConditionOr<pcl::PointXYZI>());
    range_cond_x->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, maxpoint[0])));
    range_cond_x->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, minpoint[0])));

    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond_y(new pcl::ConditionOr<pcl::PointXYZI>());
    range_cond_y->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, maxpoint[1])));
    range_cond_y->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, minpoint[1])));

    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond_z(new pcl::ConditionOr<pcl::PointXYZI>());
    range_cond_z->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, maxpoint[2])));
    range_cond_z->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::LT, minpoint[2])));

    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZI>());
    range_cond->addCondition(range_cond_x);
    range_cond->addCondition(range_cond_y);
    range_cond->addCondition(range_cond_z);

    pcl::ConditionalRemoval<pcl::PointXYZI> range_filter;
    range_filter.setCondition(range_cond);
    range_filter.setKeepOrganized(false);
    range_filter.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZI>);
    range_filter.filter(*cloud_final);

    return cloud_final;
}

/** @brief 删除box中的点,返回处理后的cloud_ptr
 * @param cloud 输入点云的指针
 * @param boxlist 存储解析以后的feature box对象
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr removeBoxPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<jsonanno::annobox> &boxlist)
{
    // for (size_t i = 0; i < boxlist.size(); ++i)
    for (size_t i = 8; i < 9; ++i)
    {
        jsonanno::annobox boxitem;
        boxitem = boxlist.at(i);
        float width = boxitem.featureValue_.width;
        float length = boxitem.featureValue_.lenth;
        float height = boxitem.featureValue_.height;
        float cx = boxitem.featureValue_.x;
        float cy = boxitem.featureValue_.y;
        float cz = boxitem.featureValue_.z;
        auto yaw = boxitem.featureValue_.yaw; //assume yaw is in degree
        //transform coordinate
        std::cout << yaw << std::endl;
        Eigen::AngleAxisf anti_yawAngle(yaw * M_PI / 180 * (-1), Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f anti_yawMatrix(Eigen::Matrix4f::Identity());

        anti_yawMatrix.block<3, 3>(0, 0) = anti_yawAngle.toRotationMatrix();
        std::cout << cloud->at(18860) << " 18860" << std::endl;
        // std::cout << anti_yawMatrix << std::endl;
        pcl::transformPointCloud(*cloud, *cloud, anti_yawMatrix);
        std::cout << cloud->at(18860) << " 18860" << std::endl;
        Eigen::Vector4f center(cx, cy, cz, 1.0);
        // std::cout << center << " center1" << std::endl;
        center = anti_yawMatrix * center;
        // std::cout << center << " center2" << std::endl;
        //filter
        Eigen::Vector4f minpoint(center[0] - 0.5 * width, center[1] - 0.5 * length, center[2] - 0.5 * height, 1);
        Eigen::Vector4f maxpoint(center[0] + 0.5 * width, center[1] + 0.5 * length, center[2] + 0.5 * height, 1);
        std::cout << maxpoint << " " << minpoint << std::endl;
        pcl::CropBox<pcl::PointXYZI>::Ptr cropbox(new pcl::CropBox<pcl::PointXYZI>);
        cropbox->setMin(minpoint);
        cropbox->setMax(maxpoint);
        cropbox->setNegative(true);
        cropbox->setInputCloud(cloud);
        std::cout << "origin cloud points number: " << cloud->size() << std::endl;
        cropbox->filter(*cloud);
        pcl::transformPointCloud(*cloud, *cloud, anti_yawMatrix.inverse());
        std::cout << "cloud at " << i << ": " << cloud->size() << std::endl;
    }

    return cloud;
}

/** @brief boxlist -> addcube
 *  //FIXME:存在BUG，无法可视化cube。
 */
void addCubeFromJson(pcl::visualization::PCLVisualizer::Ptr &viewer,
                     std::vector<jsonanno::annobox> &boxlist)
{
    for (int i = 0; i < boxlist.size(); ++i)
    {
        jsonanno::annobox boxitem(boxlist.at(i));
        //rpy欧拉角->四元数
        Eigen::AngleAxisf rollAngle(boxitem.featureValue_.roll * M_PI / 180, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(boxitem.featureValue_.pitch * M_PI / 180, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(boxitem.featureValue_.yaw * M_PI / 180, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
        q.normalize();
        Eigen::Vector3f translation(boxitem.featureValue_.x, boxitem.featureValue_.y, boxitem.featureValue_.z);
        pcl::ModelCoefficients coeffs;
        coeffs.values.push_back(boxitem.featureValue_.x);
        coeffs.values.push_back(boxitem.featureValue_.y);
        coeffs.values.push_back(boxitem.featureValue_.z);
        coeffs.values.push_back(q.x());
        coeffs.values.push_back(q.y());
        coeffs.values.push_back(q.z());
        coeffs.values.push_back(q.w());
        coeffs.values.push_back(boxitem.featureValue_.width);
        coeffs.values.push_back(boxitem.featureValue_.lenth);
        coeffs.values.push_back(boxitem.featureValue_.height);
        std::string i_str = std::to_string(i) + "th cube";
        std::cout << i_str << " : "  << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
        viewer->addCube(coeffs, i_str);
    }
}

/** @brief 读取jsonfile文件中的所有参数并存入到box_list
 *  @param jsonfile json路径
 *  @param boxlist 用于存储annobox类型数据的vector
 */
int readJson(std::string jsonfile, std::vector<jsonanno::annobox> &boxlist)
{
    Json::Value jsonroot;
    Json::Reader jsonreader;
    jsonroot.clear();
    std::ifstream ifs;
    ifs.open(jsonfile);
    if (!jsonreader.parse(ifs, jsonroot))
    {
        return -1;
    }
    ifs.close();
    std::cout << "begin parse " << jsonroot.size() << std::endl;
    for (unsigned int i = 0; i < jsonroot.size(); ++i)
    {
        jsonanno::annobox box_obj;
        box_obj.id_ = jsonroot[i]["id"].asString();
        box_obj.class_ = jsonroot[i]["class"].asString();
        box_obj.featureValue_.x = jsonroot[i]["featureValue"][0].asFloat();
        box_obj.featureValue_.y = jsonroot[i]["featureValue"][1].asFloat();
        box_obj.featureValue_.z = jsonroot[i]["featureValue"][2].asFloat();
        box_obj.featureValue_.width = jsonroot[i]["featureValue"][3].asDouble();
        box_obj.featureValue_.lenth = jsonroot[i]["featureValue"][4].asDouble();
        box_obj.featureValue_.height = jsonroot[i]["featureValue"][5].asDouble();
        box_obj.featureValue_.roll = jsonroot[i]["featureValue"][6].asFloat();
        box_obj.featureValue_.pitch = jsonroot[i]["featureValue"][7].asFloat();
        box_obj.featureValue_.yaw = jsonroot[i]["featureValue"][8].asFloat();
        boxlist.push_back(box_obj);
    }

    return 0;
}

int main(int argc, char **argv)
{
    //读取pcd和json
    std::string pcd_name;
    std::string json_name;
    if (argc >= 3)
    {
        pcd_name = argv[1];
        json_name = argv[2];
    }
    else
    {
        PCL_WARN("You must enter a pcd file name and a json file name");
        return 0;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_name, *cloud_ptr);
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_ptr, *cloud_ptr, index);

    //单次condition filter
    std::cout << "original points number: " << cloud_ptr->size() << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_condition(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Vector4f maxpoint(-4, -1.5, 1.2, 1);
    Eigen::Vector4f minpoint(-6, -2.5, -0.8, 1);
    std::cout << "before condition" << std::endl;
    cloud_condition = removeCondition(cloud_ptr, minpoint, maxpoint);
    std::cout << "condition points number: " << cloud_condition->size() << std::endl;
    pcl::io::savePCDFile("cond" + pcd_name, *cloud_condition);

    //读取Json，进行多次的删除。
    std::vector<jsonanno::annobox> boxlist;
    readJson(json_name, boxlist);
    //可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->addPointCloud<pcl::PointXYZI>(cloud_ptr,"origin cloud");
    addCubeFromJson(viewer,boxlist);


    //多次cropbox
    std::cout << "origin number for crop" << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_filtered_ptr = removeBoxPoint(cloud_ptr, boxlist);
    pcl::io::savePCDFile("filtered" + pcd_name, *cloud_filtered_ptr);
    // //单次cropbox
    Eigen::Vector4f cropmin(-4.84876, -3.07064, -1.17801, 0);
    Eigen::Vector4f cropmax(-4.37676, -2.0056, 0.573095, 255);
    pcl::CropBox<pcl::PointXYZI>::Ptr cropbox(new pcl::CropBox<pcl::PointXYZI>);
    cropbox->setMin(cropmin);
    cropbox->setMax(cropmax);
    cropbox->setNegative(true);
    Eigen::Vector3f rotation(0, 0, -6.8877);
    std::cout << "before crop " << cloud_ptr->size() << std::endl;
    cropbox->setInputCloud(cloud_ptr);
    cropbox->filter(*cloud_filtered_ptr);
    std::cout << "for one crop :filtered points number: " << cloud_filtered_ptr->size() << std::endl;
    pcl::io::savePCDFile("crop" + pcd_name, *cloud_filtered_ptr);


    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    return 0;
}
