/** @author Xu Yuwei
 *  @brief 用于读取博登的detection3D导出的json文件，并存入对应类型的annobox中。 
 *  @date 2020/7/23
 */

//std
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <fstream>
#include <string.h>
//json
#include <jsoncpp/json/json.h>
//costomer
#include "annobox.hpp"
using namespace jsonanno;
int getFileNames(std::string,std::vector<std::string>&);
int readJson(std::string,std::vector<annobox>&);

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
    std::cout << "Have already read " << filesname.size() << " json files." << std::endl;
    std::cout << "Please wait for parsing..." << std::endl;

    std::vector<std::vector<annobox>> folder_boxlist;

    for(int i = 0;i < filesname.size();++i)
    {
        std::string filename = filesname[i];
        std::vector<annobox> boxlist_;
        readJson(filename,boxlist_);
        std::cout << filename << " include " << boxlist_.size() << "feature value." << std::endl;
        folder_boxlist.push_back(boxlist_);
    }
    
    std::cout << folder_boxlist.at(0)[0].id_ << std::endl;
    
    
    return 0;
}

/** @brief 遍历path下所有文件及文件夹，将文件名存入filenames
 *  @param path 需要遍历的路径
 *  @param filenames 存储所有的文件名
 */
int getFileNames(std::string path,std::vector<std::string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str())))
        return -1;
    while((ptr = readdir(pDir)) != 0){
        if(strcmp(ptr->d_name,".") != 0 && strcmp(ptr->d_name,"..") != 0)
            filenames.push_back(path + "/" + ptr->d_name);
    }
    closedir(pDir);
    return 0;
}

/** @brief 读取jsonfile文件中的所有参数并存入到box_list
 *  @param jsonfile json路径
 *  @param boxlist 用于存储annobox类型数据的vector
 */
int readJson(std::string jsonfile,std::vector<annobox>& boxlist)
{
    Json::Value jsonroot;
    Json::Reader jsonreader;
    jsonroot.clear();
    std::ifstream ifs;
    ifs.open(jsonfile);
    if(!jsonreader.parse(ifs,jsonroot))
    {
        return -1;
    }
    ifs.close();
    std::cout << "begin parse" << jsonroot.size() << std::endl;
    for(unsigned int i=0;i<jsonroot.size();++i)
    {
        annobox box_obj;
        box_obj.id_ = jsonroot[i]["id"].asString();
        box_obj.class_ = jsonroot[i]["class"].asString();
        box_obj.featureValue_.x = jsonroot[i]["featureValue"][1].asDouble();
        box_obj.featureValue_.y = jsonroot[i]["featureValue"][2].asDouble();
        box_obj.featureValue_.z = jsonroot[i]["featureValue"][3].asDouble();
        box_obj.featureValue_.lenth = jsonroot[i]["featureValue"][4].asDouble();
        box_obj.featureValue_.width = jsonroot[i]["featureValue"][5].asDouble();
        box_obj.featureValue_.height = jsonroot[i]["featureValue"][6].asDouble();
        box_obj.featureValue_.roll = jsonroot[i]["featureValue"][7].asDouble();
        box_obj.featureValue_.pitch = jsonroot[i]["featureValue"][8].asDouble();
        box_obj.featureValue_.yaw = jsonroot[i]["featureValue"][9].asDouble();
        boxlist.push_back(box_obj);
    }

    return 0;
}
