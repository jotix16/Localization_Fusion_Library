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


// ------------------------------- TO BE DELETED
  /**
   * @brief Constructs a rotation using components of a quaternion.
   *
   * @param qw Quaternion w-coordinate
   * @param qx Quaternion x-coordinate
   * @param qy Quaternion y-coordinate
   * @param qz Quaternion z-coordinate
   */
  template <typename T>
  Eigen::Matrix<T,3,1> ToEulerAngles(Eigen::Quaternion<T> q)
  {
    T roll_ = std::atan2(T(2.0) * (q.w() * q.y() - q.x() * q.z()),
                         T(2.0) * (q.w()*q.w() + q.z()*q.z()) -
                             T(1.0));
    T pitch_ = std::asin(T(2.0) * (q.w() * q.x() + q.y() * q.z()));
    T yaw_ = std::atan2(T(2.0) * (q.w() * q.z() - q.x() * q.y()),
                        T(2.0) * (q.w()*q.w() + q.y()*q.y()) -
                            T(1.0));
    return {roll_, pitch_, yaw_};
  }

  /**
   * @brief Converts to a quaternion with a non-negative scalar part
   * @return Quaternion encoding this rotation.
   */
  template <typename T>
  Eigen::Quaternion<T> ToQuaternion(T roll, T pitch, T yaw) {
    T coeff = T(0.5);
    T r = roll * coeff;
    T p = pitch * coeff;
    T y = yaw * coeff;

    T sr = std::sin(r);
    T sp = std::sin(p);
    T sy = std::sin(y);

    T cr = std::cos(r);
    T cp = std::cos(p);
    T cy = std::cos(y);

    T qw = cr * cp * cy - sr * sp * sy;
    T qx = cr * sp * cy - sr * cp * sy;
    T qy = cr * sp * sy + sr * cp * cy;
    T qz = cr * cp * sy + sr * sp * cy;
    if (qw < 0.0) {
      return {-qw, -qx, -qy, -qz};
    }
    return {qw, qx, qy, qz};
  }
// -------------------------------


  template <typename T>
  inline T asin(T x)
 {
    if (x < T(-1))
      x = T(-1);
    if (x > T(1))
      x = T(1);
    return std::asin(x);
 }

  template <typename T>
	Eigen::Matrix<T,3,3> quat_to_rot(const Eigen::Quaternion<T>& q)
	{
		// T d = q.squaredNorm();
		T d = q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w();
		std:assert(d != T(0.0));
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

  template <typename T>
	Eigen::Matrix<T,3,1> get_euler_rpy(const Eigen::Matrix<T,3,3>& m_el, unsigned int solution_number = 1)
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

  template <typename T>
	Eigen::Matrix<T,3,1> get_euler_rpy(const Eigen::Quaternion<T> q, unsigned int solution_number = 1)
  {
    return get_euler_rpy(quat_to_rot(q), solution_number);
  }

  /**@brief Set the quaternion using Euler angles
   * @param yaw Angle around Y
   * @param pitch Angle around X
   * @param roll Angle around Z */
  template <typename T>
	Eigen::Quaternion<T> get_quat_rpy(const T& roll, const T& pitch, const T& yaw)
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

	/**@brief Get the matrix represented as a quaternion
	* @param q The quaternion which will be set */
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