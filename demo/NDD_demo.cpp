
/**
  * @file NDD_demo.cpp
  * @author julian 
  * @date 12/21/22
 */
 
#include "NDD.h"
#include <iostream>
#include <pcl/io/pcd_io.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZI> scene, model;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("../data/000209.pcd", scene) != 0)
    {
        throw std::runtime_error("Couldn't read pcd file");
    }
    
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("../data/001684.pcd", model) != 0)
    {
        throw std::runtime_error("Couldn't read pcd file");
    }
    
    ndd::NDDManager ndd;
    
    ndd.makeAndSaveNDDAndKeys(scene);
    ndd.makeAndSaveNDDAndKeys(model);
    auto [loop_id, yaw] = ndd.detectLoopClosureID();
    
    if (loop_id < 0)
    {
        throw std::runtime_error("Couldn't find loop");
    }
    
    std::cout << loop_id << " - " << yaw << "\n";
    
    return 0;
}
