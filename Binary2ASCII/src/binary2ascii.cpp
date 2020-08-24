//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//std
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <vector>
void getFileNames(std::string,std::vector<std::string>&);

typedef pcl::PointXYZI XYZI;
typedef pcl::PointXYZ XYZ;
int main(int argc,char* argv[])
{
    std::string path;
    std::vector<std::string> filesname;
    if(argc <= 1){
        std::cout << "could not read pcdfile.Please enter a folder name" << std::endl;
        return (-1);
    } 
    else 
    {
        path = argv[1];
    } 
    getFileNames(path,filesname);
    std::cout << "Have already read " << filesname.size() << " binary PCD files." << std::endl;
    std::cout << "Please wait for transform..." << std::endl;
    pcl::PointCloud<XYZI> cloud;
    int count = 0;
    for(int i = 0;i < filesname.size();i++)
    {
        std::string filename = filesname[i];
        pcl::io::loadPCDFile(filename,cloud);
        if(pcl::io::savePCDFileASCII(filename,cloud) != (-1))
            count++;
    }
    std::cout << "Have already transform " << count << " binary PCD files into ASCII PCD files."  << std::endl;
    
    return 0;
}

void getFileNames(std::string path,std::vector<std::string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str())))
        return;
    while((ptr = readdir(pDir)) != 0){
        if(strcmp(ptr->d_name,".") != 0 && strcmp(ptr->d_name,"..") != 0)
            filenames.push_back(path + "/" + ptr->d_name);
    }
    closedir(pDir);
}

