//并不知道自己为什么转出的二进制无法在博登app上识别。
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/io/boost.h>
#include <boost/program_options.hpp>
//std
#include <omp.h>
#include <ctime>
#include <algorithm>
#include <sys/stat.h>
#include <fstream>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <vector>
typedef pcl::PointXYZI XYZI;
typedef pcl::PointXYZ XYZ;

void getFileNames(std::string, std::vector<std::string> &);

bool convertPCDtoBin(std::string &in_file, std::string &out_file)
{
    pcl::PointCloud<XYZI>::Ptr cloud(new pcl::PointCloud<XYZI>);
    if (pcl::io::loadPCDFile<XYZI>(in_file, *cloud) == -1)
    {
        std::string err = "Could't read file: " + in_file;
        PCL_ERROR(err.c_str());
        return false;
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from"
              << in_file << " with the following fields: " << std::endl;

    std::ofstream myFile(out_file.c_str(), std::ios::out | std::ios::binary);
    for (int j = 0; j < cloud->size(); j++)
    {
        myFile.write((char *)&cloud->at(j).x, sizeof(cloud->at(j).x));
        myFile.write((char *)&cloud->at(j).y, sizeof(cloud->at(j).y));
        myFile.write((char *)&cloud->at(j).z, sizeof(cloud->at(j).z));
        myFile.write((char *)&cloud->at(j).intensity, sizeof(cloud->at(j).intensity));
    }
    myFile.close();
    return true;
}

int main(int argc, char *argv[])
{
    std::string path;
    std::vector<std::string> filesname;
    if (argc <= 1)
    {
        std::cout << "could not read pcdfile.Please enter a folder name" << std::endl;
        return (-1);
    }
    else
    {
        path = argv[1];
    }
    getFileNames(path, filesname);
    std::cout << "Have already read " << filesname.size() << " PCD files." << std::endl;
    std::cout << "Please wait for transforming all PCD to Bin ..." << std::endl;
    pcl::PointCloud<XYZI> cloud;
    int count = 0;
    for (int i = 0; i < filesname.size(); i++)
    {
        std::string filename = filesname[i];
        std::cout << filename << std::endl;
        std::string binname = filename;
        binname[binname.size()-3]='b';
        binname[binname.size()-2]='i';
        binname[binname.size()-1]='n';
        std::cout << binname << std::endl;
        if (convertPCDtoBin(filename, binname))
            count++;
    }
    std::cout << "Have already transform " << count << " PCD files into Bin files." << std::endl;

    return 0;
}

void getFileNames(std::string path, std::vector<std::string> &filenames)
{
    DIR *pDir;
    struct dirent *ptr;
    if (!(pDir = opendir(path.c_str())))
        return;
    while ((ptr = readdir(pDir)) != 0)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            filenames.push_back(path + "/" + ptr->d_name);
    }
    closedir(pDir);
}
