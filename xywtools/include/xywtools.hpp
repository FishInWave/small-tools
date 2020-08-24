#ifndef XYW_TOOLS_HPP_
#define XYW_TOOLS_HPP_

#include <string>
#include <vector>
#include "annobox.hpp"


namespace xywtools{

class AnalysJson
{
private:
    /* data */
public:
    AnalysJson(/* args */);
    ~AnalysJson();
    int readJson(std::string,std::vector<jsonanno::annobox>&);

};

AnalysJson::AnalysJson(/* args */)
{
}

AnalysJson::~AnalysJson()
{
}

class GetFilesName
{
private:
    /* data */
public:
    GetFilesName(/* args */);
    ~GetFilesName();
    int getFilesName(std::string,std::vector<std::string>&);
};

GetFilesName::GetFilesName(/* args */)
{
}

GetFilesName::~GetFilesName()
{
}






}







#endif