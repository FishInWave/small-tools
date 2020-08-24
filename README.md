# small-tools
这个仓库主要包含了自己编写的一些小工具，工具的应用环境为LINUX，已在Ubuntu下进行测试。
目前的工具目录：
* 1.pcl_viewer
* 2.binary2ASCII
* 3.analysJson

# 一、功能说明
##　1.pcl_viewer
    可视化用指定的pcd文件，其中点云应当为XYZI类型。
##  2.binary2ASCII
    将指定目录下所有的binary格式pcd文件转换为同名的ASCII格式的pcd文件。
##  3.analysJson
    读取博登的detection3D导出的json。
上述功能的命令行参数单体则输入文件，全体则输入文件路径。
# 二、使用方法：
* 1.先克隆下整个项目：git clone https://github.com/FishInWave/small-tools.git

  
* 2.编译(以pcl_viewer为例)
  ```
    cd 工具目录
    mkdir build && cd build
    cmake ..
    make 
    ./pcl_simple xx.pcd
  ```