/** ----------------------------------------------------------------------------
 * @file: uuv_common.cpp
 * @date: March 2, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Common functions and constants used throughout the uuv package.
 * -----------------------------------------------------------------------------
 * */

#include "uuv_common.hpp"

namespace uuv_common
{
    vanttec_uuv::GuidanceWaypoints GenerateCircle(float _radius, float _x_center, float _y_center, float _z_center, float _angle_offset)
    {
        vanttec_uuv::GuidanceWaypoints _waypoints;
        
        float angle = _angle_offset;
        uint8_t counter = 0;
        
        while (angle <= (_angle_offset + M_PI))
        {
            _waypoints.waypoint_list_x.push_back(_radius * std::cos(angle) + _x_center);
            _waypoints.waypoint_list_y.push_back(_radius * std::sin(angle) + _y_center);
            _waypoints.waypoint_list_z.push_back(_z_center);
            angle += M_PI / 6;
            counter++;
        }
        
        angle = _angle_offset - M_PI;

        while (angle <= _angle_offset)
        {
            _waypoints.waypoint_list_x.push_back(_radius * std::cos(angle) + _x_center);
            _waypoints.waypoint_list_y.push_back(_radius * std::sin(angle) + _y_center);
            _waypoints.waypoint_list_z.push_back(_z_center);
            angle += M_PI / 6;
            counter++;
        }
        
        _waypoints.waypoint_list_length = counter;

        return _waypoints;
    }


    Eigen::MatrixXf CalculateRotation(double phi, double theta, double psi)
    {
        Eigen::Matrix3f R;
        R <<    std::cos(psi)*std::cos(theta),      -std::sin(psi)*std::cos(phi) + std::cos(psi)*std::sin(theta)*std::sin(phi),     std::sin(psi)*std::sin(phi) + std::cos(psi)*std::cos(phi)*std::sin(theta),
                std::sin(psi)*std::cos(theta),       std::cos(psi)*std::cos(phi) + std::sin(phi)*std::sin(theta)*std::sin(psi),    -std::cos(psi)*std::sin(phi) + std::sin(theta)*std::sin(psi)*std::cos(phi),
               -std::sin(theta),                     std::cos(theta)*std::sin(phi),                                                 std::cos(theta)*std::cos(phi);

        return R;
    }

    // Eigen::MatrixXf CalculateTransformation(double phi, double theta, double psi)
    // {
    //     Eigen::Matrix3f R;
    //     R << std::cos(theta)*std::cos(psi),                                                 std::cos(theta)*std::sin(psi),                                                  -std::sin(theta),
    //          std::sin(phi)*std::sin(theta)*std::cos(psi) - std::cos(phi)*std::sin(psi),     std::sin(phi)*std::sin(theta)*std::sin(psi) + std::cos(phi)*std::cos(psi),      std::sin(phi)*std::cos(theta),
    //          std::cos(phi)*std::sin(theta)*std::cos(psi) + std::sin(phi)*std::sin(psi),     std::cos(phi)*std::sin(theta)*std::sin(psi) - std::sin(phi)*std::cos(psi),      std::cos(phi)*std::cos(theta);

    //     Eigen::Matrix3f T;
    //     T << 1,     std::sin(phi)*std::tan(theta),  std::cos(phi)*std::tan(theta),
    //          0,     std::cos(phi),                  -std::sin(phi),
    //          0,     std::sin(phi)/std::cos(theta),  std::cos(phi)/std::cos(theta);

    //     Eigen::Matrix3f zero;
    //     zero << 0,0,0,
    //             0,0,0,
    //             0,0,0;

    //     Eigen::MatrixXf J(6,6);
    //     J << R,     zero,
    //          zero,  T;

    //     return J;
    // }
}