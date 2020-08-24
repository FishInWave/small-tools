#ifndef  ANNOBOX_HPP
#define ANNOBOX_HPP

#include <string>
namespace jsonanno{
    //该类json文件由博登的detection3D导出。
class annobox
{
public:
    std::string id_;
    std::string class_;
    struct featureValue
    {
        float x;
        float y;
        float z;
        double lenth;
        double width;
        double height;
        float roll;
        float pitch;
        float yaw;
    } featureValue_;
    

public:
    annobox(/* args */);
    ~annobox();
};

annobox::annobox(/* args */)
{
}


annobox::~annobox()
{
}





}
#endif