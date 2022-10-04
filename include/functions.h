#pragma once

//source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

#define _USE_MATH_DEFINES
#include <cmath>
#include <tf/tf.h>

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {

    //------------------using wikipedia
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    // std::cout << "\n angles with wikipedia are \n" << angles.roll << "\n" << angles.pitch << "\n" << angles.yaw << std::endl;

    //--------------using eigen:

/*     Eigen::Quaterniond eigenquat(q.w, q.x, q.y, q.z);

     auto euler = eigenquat.toRotationMatrix().eulerAngles(0, 1, 2);

     std::cout << "\n euler with quaternions are: \n" << euler << std::endl;

     std::cout << "\n difference is: \n" << angles.roll - euler(0) << "\n" << angles.pitch - euler(1) << "\n" << angles.yaw - euler(2) << std::endl;*/

    // ---------------------using ros:

    tf::Quaternion rosquat;
    rosquat[0] = q.x;
    rosquat[1] = q.y;
    rosquat[2] = q.z;
    rosquat[3] = q.w;
    tf::Matrix3x3 m(rosquat);
    double rollr, pitchr, yawr;
    m.getRPY(rollr, pitchr, yawr);
    // std::cout << "\n angles with ros are \n" << rollr << "\n" << pitchr << "\n" << yawr << std::endl;


    // std::cout << "\n difference is: \n" << rollr - angles.roll << "\n" << pitchr - angles.pitch << "\n" << yawr - angles.yaw << std::endl;





    return angles;
}


