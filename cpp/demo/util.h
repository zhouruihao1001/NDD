#pragma once

/**
  * @file util.h
  * @author julian 
  * @date 12/22/22
 */

// standard lib
#include <set>
#include <string>
#include <istream>
#include <iostream>
#include <fstream>
#include <filesystem>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace demo
{

template<char delimiter>
class WordDelimitedBy : public std::string
{
};

template<char delimiter>
inline std::istream &operator>>(std::istream &is, WordDelimitedBy<delimiter> &output)
{
    std::getline(is, output, delimiter);
    return is;
}

inline void
read_txt(const std::filesystem::path &poses_txt, pcl::PointCloud<pcl::PointXYZ> &points, char delimeter = ',')
{
    points.reserve(500);
    std::string line;
    std::ifstream myfile(poses_txt);
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            std::istringstream iss(line);
            std::vector<std::string> results((std::istream_iterator<WordDelimitedBy<','>>(iss)),
                                             std::istream_iterator<WordDelimitedBy<','>>());
            points.emplace_back(pcl::PointXYZ(std::stof(results[0]), std::stof(results[1]), std::stof(results[2])));
        }
        myfile.close();
    }
    else
    {
        throw std::runtime_error("Can't open " + poses_txt.string());
    }
}

inline void index_keyframes(const std::filesystem::path &keyframe_dir, std::vector<std::filesystem::path> &keyframe_paths, std::string extension = ".pcd")
{
    std::set<std::filesystem::path> sorted_by_name;
    
    for (const auto &entry : std::filesystem::directory_iterator(keyframe_dir))
    {
        if (extension == std::filesystem::path(entry).extension())
        {
            sorted_by_name.insert(entry.path());
        }
    }
    
    std::copy(sorted_by_name.begin(), sorted_by_name.end(), std::back_inserter(keyframe_paths));
}

} // end namespace demo