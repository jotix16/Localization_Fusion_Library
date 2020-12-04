 /**
 * @file
 * @brief Defines the EulerAnglesZXY class.
 */

#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include <utilities/filter_utilities.h>
/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace iav{
namespace state_predictor{
namespace euler{

    /**
     * @brief asin: asin math funciton
     * @param[in] x - input
     * @return asin(x)
     */
	template <typename T>
	inline T asin(T x)
	{
		if (x < T(-1))
		x = T(-1);
		if (x > T(1))
		x = T(1);
		return std::asin(x);
	}

    /**
     * @brief euler: Funciton that transforms quaternion to rotation matrix
     * @param[in] q - quaternion to be transformed
     * @return return the rotation matrix
     */
  	template <typename T>
	Eigen::Matrix<T,3,3> quat_to_rot(const Eigen::Quaternion<T> q)
	{
		T d = q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w();
		assert(d != T(0.0));
		T s = T(2.0) / d;
		T xs = q.x() * s,   ys = q.y() * s,   zs = q.z() * s;
		T wx = q.w() * xs,  wy = q.w() * ys,  wz = q.w() * zs;
		T xx = q.x() * xs,  xy = q.x() * ys,  xz = q.x() * zs;
		T yy = q.y() * ys,  yz = q.y() * zs,  zz = q.z() * zs;
		Eigen::Matrix<T,3,3> rot_matr;
		rot_matr << T(1.0) - (yy + zz), xy - wz, xz + wy,
					xy + wz, T(1.0) - (xx + zz), yz - wx,
					xz - wy, yz + wx, T(1.0) - (xx + yy);
		return rot_matr;
	}

    /**
     * @brief euler: Get rpy angles in zyx ordering from the matrix
     * @param[in] m_el - vector for the state we are estimating
     * @param[in] solution_number - pointer to the imu msg of the measurement
     * @return return the translation
     */
  	template <typename T>
	Eigen::Matrix<T,3,1> get_euler_rpy(const Eigen::Matrix<T,3,3> m_el, unsigned int solution_number = 1)
	{
		struct Euler
		{
			T yaw;
			T pitch;
			T roll;
		};

		Euler euler_out;
		Euler euler_out2; //second solution
		//get the pointer to the raw data

		// Check that pitch is not at a singularity
  		// Check that pitch is not at a singularity
		if (std::fabs(m_el(2,0)) >= 1)
		{
			euler_out.yaw = 0;
			euler_out2.yaw = 0;

			// From difference of angles formula
			T delta = std::atan2(m_el(2,1),m_el(2,2));
			if (m_el(2,0) < 0)  //gimbal locked down
			{
				euler_out.pitch = PI / T(2.0);
				euler_out2.pitch = PI / T(2.0);
				euler_out.roll = delta;
				euler_out2.roll = delta;
			}
			else // gimbal locked up
			{
				euler_out.pitch = -PI / T(2.0);
				euler_out2.pitch = -PI / T(2.0);
				euler_out.roll = delta;
				euler_out2.roll = delta;
			}
		}
		else
		{
			euler_out.pitch = - asin(m_el(2,0));
			euler_out2.pitch = PI - euler_out.pitch;

			euler_out.roll = std::atan2(m_el(2,1)/std::cos(euler_out.pitch),
				m_el(2,2)/std::cos(euler_out.pitch));
			euler_out2.roll = std::atan2((2,1)/std::cos(euler_out2.pitch),
				m_el(2,2)/std::cos(euler_out2.pitch));

			euler_out.yaw = std::atan2(m_el(1,0)/std::cos(euler_out.pitch),
				m_el(0,0)/std::cos(euler_out.pitch));
			euler_out2.yaw = std::atan2(m_el(1,0)/std::cos(euler_out2.pitch),
				m_el(0,0)/std::cos(euler_out2.pitch));
		}

		if (solution_number == 1)
		{
      		return {euler_out.roll, euler_out.pitch, euler_out.yaw};
		}
		else
		{
      		return {euler_out2.roll, euler_out2.pitch, euler_out2.yaw};
		}
	}

    /**
     * @brief euler: Get rpy angles in zyx ordering from the quaternion
     * @param[in] q - vector for the state we are estimating
     * @param[in] solution_number - to decide which from two possibilities to use(1 is zyx ordering)
     * @return return the rpy as a vector
     */
  	template <typename T>
	Eigen::Matrix<T,3,1> get_euler_rpy(const Eigen::Quaternion<T> q, unsigned int solution_number = 1)
	{
		return get_euler_rpy(quat_to_rot(q), solution_number);
	}

    /**
     * @brief euler: Set the quaternion using Euler angles
     * @param[in] roll - Angle around Y
     * @param[in] pitch - Angle around X
     * @param[in] yaw - Angle around Y
     * @return return the quaternion
     */
  	template <typename T>
	Eigen::Quaternion<T> get_quat_rpy(const T roll, const T pitch, const T yaw)
	{
		T halfYaw = T(yaw) * T(0.5);
		T halfPitch = T(pitch) * T(0.5);
		T halfRoll = T(roll) * T(0.5);
		T cosYaw = std::cos(halfYaw);
		T sinYaw = std::sin(halfYaw);
		T cosPitch = std::cos(halfPitch);
		T sinPitch = std::sin(halfPitch);
		T cosRoll = std::cos(halfRoll);
		T sinRoll = std::sin(halfRoll);
    	return Eigen::Quaternion<T>(
			cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw, //w
			sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
      		cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
			cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw);//z
	}

    /**
     * @brief euler: Get the matrix represented as a quaternion
     * @param[in] m_el - rotation matrix
     * @return return the quaternion
     */
  	template <typename T>
	Eigen::Quaternion<T> rot_to_quat(const Eigen::Matrix<T,3,3>& m_el)
	{
		T trace = m_el(0,0) + m_el(1,1) + m_el(2,2);
		T temp[4];

		if (trace > T(0.0))
		{
			T s = std::sqrt(trace + T(1.0));
			temp[3]=(s * T(0.5));
			s = T(0.5) / s;

			temp[0]=((m_el(2,1) - m_el(1,2)) * s);
			temp[1]=((m_el(0,2) - m_el(2,0)) * s);
			temp[2]=((m_el(1,0) - m_el(0,1)) * s);
		}
		else
		{
			int i = m_el(0,0) < m_el(1,1) ?
				(m_el(1,1) < m_el(2,2) ? 2 : 1) :
				(m_el(0,0) < m_el(2,2) ? 2 : 0);
			int j = (i + 1) % 3;
			int k = (i + 2) % 3;

			T s = std::sqrt(m_el(i,i) - m_el(j,j) - m_el(k,k) + T(1.0));
			temp[i] = s * T(0.5);
			s = T(0.5) / s;

			temp[3] = (m_el(k,j) - m_el(j,k)) * s;
			temp[j] = (m_el(j,i) + m_el(i,j)) * s;
			temp[k] = (m_el(k,i) + m_el(i,k)) * s;
		}
    	return Eigen::Quaternion<T>(temp[3], temp[0], temp[1], temp[2]);
	}

}  // namespace utilities
}  // namespace state_predictor
}  // namespace iav