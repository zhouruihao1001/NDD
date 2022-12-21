#pragma once

/**
  * @file util.h
  * @author julian 
  * @date 12/21/22
 */

#include <cmath>
#include <Eigen/Dense>

inline float deg2rad(float degrees)
{
    return degrees * M_PI / 180.0;
}


inline float xy2theta(const float &_x, const float &_y)
{
    if (_x >= 0 && _y >= 0)
    {
        return (180 / M_PI) * std::atan(_y / _x);
    }
    
    if (_x < 0 && _y >= 0)
    {
        return 180 - ((180 / M_PI) * std::atan(_y / (-_x)));
    }
    
    if (_x < 0 && _y < 0)
    {
        return 180 + ((180 / M_PI) * std::atan(_y / _x));
    }
    
    if (_x >= 0 && _y < 0)
    {
        return 360 - ((180 / M_PI) * std::atan((-_y) / _x));
    }
    
    throw std::runtime_error("xy2theta fail: " + std::to_string(_x) + std::to_string(_y));
} // xy2theta


inline Eigen::MatrixXd circshift(Eigen::MatrixXd &_mat, int _num_shift)
{
    // shift columns to right direction
    assert(_num_shift >= 0);
    
    if (_num_shift == 0)
    {
        Eigen::MatrixXd shifted_mat(_mat);
        return shifted_mat; // Early return
    }
    
    Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero(_mat.rows(), _mat.cols());
    for (int col_idx = 0; col_idx < _mat.cols(); col_idx++)
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }
    
    return shifted_mat;
    
} // circshift


inline std::vector<float> eig2stdvec(Eigen::MatrixXd _eigmat)
{
    std::vector<float> vec(_eigmat.data(), _eigmat.data() + _eigmat.size());
    return vec;
} // eig2stdvec