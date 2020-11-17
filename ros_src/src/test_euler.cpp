#include <iostream>
#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

#include <Eigen/Eigen>
#include <cstdlib>
#include <utilities/filter_utilities.h>
#include <utilities/filter_euler_zyx.h>
#include <cassert>

using namespace iav::state_predictor::euler;
using namespace iav::state_predictor;

double EPSILON = std::numeric_limits<double>::epsilon();
bool AreSame(double a, double b)
{
    return std::fabs(a - b) < 1e-7;
}

void print(tf2::Matrix3x3 mat)
{
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            std::cout << mat[i][j] <<" ";
        }
        std::cout <<"\n";
    }
}

template <typename T>
Eigen::Matrix<T,3,3> copy(tf2::Matrix3x3 mat)
{
    Eigen::Matrix<T,3,3> mat2;
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            mat2(i,j) = mat[i][j];
        }
    }

}

template <typename T>
bool assert_near(tf2::Matrix3x3 mat, Eigen::Matrix<T,3,3> mat2)
{
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            if(!AreSame(mat2(i,j) , mat[i][j])) return false;
        }
    }
    return true;

}
template <typename T>
bool assert_near_quaternion(tf2::Quaternion quat, Eigen::Quaternion<T> quat2)
{
    if(!AreSame(quat.x() , quat2.x())) return false;
    if(!AreSame(quat.y() , quat2.y())) return false;
    if(!AreSame(quat.z() , quat2.z())) return false;
    if(!AreSame(quat.w() , quat2.w())) return false;
    return true;

}


int main(int argc, char **argv)
{
    double roll_tmp_tf, pitch_tmp_tf, yaw_tmp_tf;
    double roll_tmp, pitch_tmp, yaw_tmp;
    for (double roll = 0.0; roll <= TAU; roll+=0.2)
    {
        for (double pitch = 0.0; pitch <= TAU; pitch+=0.2)
        {
            for (double yaw = 0.0; yaw <= TAU; yaw+=0.2)
            {
                tf2::Quaternion quat_tf2;
                quat_tf2.setRPY(roll, pitch, yaw);
                tf2::Matrix3x3 orTmp(quat_tf2);
                orTmp.getRPY(roll_tmp_tf, pitch_tmp_tf, yaw_tmp_tf);

                Eigen::Quaterniond quat_eigen;
                quat_eigen = get_quat_rpy(roll, pitch, yaw);
                auto rot_mat = quat_to_rot(quat_eigen);
                // copy(orTmp, rot_mat);
                auto x = get_euler_rpy(rot_mat);
                roll_tmp = x(0); pitch_tmp = x(1); yaw_tmp = x(2);

                // Eigen::Matrix<double,3,3> tmp_rot = copy<double>(orTmp);
                if(!assert_near(orTmp, rot_mat))
                {
                    std::cout <<"rot matrix problem\n";
                    return 0;
                }
                if(!assert_near_quaternion(quat_tf2, quat_eigen))
                {
                    std::cout <<"quat matrix problem\n";
                    return 0;
                }

                if(!( AreSame(roll_tmp_tf,roll_tmp) && AreSame(pitch_tmp_tf,pitch_tmp) && AreSame(yaw_tmp_tf,yaw_tmp)))
                {
                    std::cout << "WRONG\n";
                    std::cout << quat_tf2.x() << " " << quat_tf2.y() << " " << quat_tf2.z() << " " << quat_tf2.w() << "\n";
                    std::cout << quat_eigen.vec().transpose() << " " << quat_eigen.w() << "\n";
                    std::cout << "ROT MAT\n";
                    print(orTmp);
                    std::cout << rot_mat <<"\n";
                    std::cout << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << "\n";
                    std::cout << "roll_dl: " << roll_tmp_tf-roll_tmp +TAU << ", pitch_dl: " << pitch_tmp_tf-pitch_tmp << ", yaw_dl: " << yaw_tmp_tf-yaw_tmp << "\n";
                    std::cout << "roll_tf: " << roll_tmp_tf << ", pitch_tf: " << pitch_tmp_tf << ", yaw_tf: " << yaw_tmp_tf << "\n";
                    std::cout << "roll_eg: " << roll_tmp << ", pitch_eg: " << pitch_tmp << ", yaw_eg: " << yaw_tmp << "\n";
                    return 0;
                };
            }
        }
    // break;
    }


    for (double x = 0.0; x <= 1; x+=0.05)
    {
        for (double y = 0.0; y <= 1; y+=0.05)
        {
            for (double z = 0.0; z <= 1; z+=0.05)
            {
                Eigen::Quaterniond q_eg(1,x,y,z);
                auto rpy_eg = euler::get_euler_rpy(q_eg.normalized());

                tf2::Quaternion q_tf(x, y, z, 1);
                tf2::Matrix3x3 orTmp(q_tf.normalized());
                double roll_tf, pitch_tf, yaw_tf;
                orTmp.getRPY(roll_tf, pitch_tf, yaw_tf);

                tf2::Stamped<tf2::Transform> poseTmp;
                poseTmp.setRotation(q_tf.normalized());
                tf2::Matrix3x3 orTmp2(poseTmp.getRotation());
                orTmp2.getRPY(roll_tf, pitch_tf, yaw_tf);

                if(!assert_near(orTmp, euler::quat_to_rot(q_eg.normalized())))
                {
                    std::cout <<"rot matrix problem\n";
                    return 0;
                }
                if(!assert_near_quaternion(q_tf.normalized(), q_eg.normalized()))
                {
                    std::cout <<"quat matrix problem\n";
                    return 0;
                }
                if(!( AreSame(roll_tf,rpy_eg[0]) && AreSame(pitch_tf,rpy_eg[1]) && AreSame(yaw_tf,rpy_eg[2])))
                {
                    return 0;
                }
            }
        }
    }

    std::cout << "MIKEL\n";

  return 0;
}
