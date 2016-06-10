/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Joseph Mirabel */

#ifndef FCL_MATH_TOOLS_H
#define FCL_MATH_TOOLS_H

#include <hpp/fcl/deprecated.hh>
#include <hpp/fcl/config.hh>
#include <hpp/fcl/config-fcl.hh>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <limits>

#define FCL_CCD_INTERVALVECTOR_PLUGIN <hpp/fcl/eigen/plugins/ccd/interval-vector.h>
#define FCL_CCD_MATRIXVECTOR_PLUGIN <hpp/fcl/eigen/plugins/ccd/interval-matrix.h>

namespace fcl
{

template<typename Derived>
static inline typename Derived::Scalar triple(
    const Eigen::MatrixBase<Derived>& x,
    const Eigen::MatrixBase<Derived>& y,
    const Eigen::MatrixBase<Derived>& z)
{
  return x.derived().dot(y.derived().cross(z.derived()));
}


/*
template<typename Derived, typename OtherDerived>
static inline const typename Eigen::BinaryReturnType<const Derived, const OtherDerived>::Min
 min(const Eigen::MatrixBase<Derived>& x, const Eigen::MatrixBase<OtherDerived>& y)
{
  return typename Eigen::BinaryReturnType<const Derived, const OtherDerived>::Min (x.derived(), y.derived());
}

template<typename Derived, typename OtherDerived>
static inline const typename Eigen::BinaryReturnType<const Derived, const OtherDerived>::Max
 max(const Eigen::MatrixBase<Derived>& x, const Eigen::MatrixBase<OtherDerived>& y)
{
  return typename Eigen::BinaryReturnType<const Derived, const OtherDerived>::Max (x.derived(), y.derived());
}

template<typename Derived>
static inline const typename Eigen::UnaryReturnType<const Derived>::Abs
abs(const Eigen::MatrixBase<Derived>& x)
{
  return typename Eigen::UnaryReturnType<const Derived>::Abs (x.derived());
} */

template<typename Derived1, typename Derived2, typename Derived3>
void generateCoordinateSystem(
    const Eigen::MatrixBase<Derived1>& _w,
    const Eigen::MatrixBase<Derived2>& _u,
    const Eigen::MatrixBase<Derived3>& _v)
{
  typedef typename Derived1::Scalar T;

  Eigen::MatrixBase<Derived1>& w = const_cast < Eigen::MatrixBase<Derived1>& > (_w);
  Eigen::MatrixBase<Derived2>& u = const_cast < Eigen::MatrixBase<Derived2>& > (_u);
  Eigen::MatrixBase<Derived3>& v = const_cast < Eigen::MatrixBase<Derived3>& > (_v);

  T inv_length;
  if(std::abs(w[0]) >= std::abs(w[1]))
  {
    inv_length = (T)1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = (T)0;
    u[2] = w[0] * inv_length;
    v[0] = w[1] * u[2];
    v[1] = w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = (T)1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = (T)0;
    u[1] = w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] = w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] = w[0] * u[1];
  }
}

/* ----- Start Matrices ------ */
template<typename Derived, typename OtherDerived>
void hat(const Eigen::MatrixBase<Derived>& mat, const Eigen::MatrixBase<OtherDerived>& vec) HPP_FCL_DEPRECATED;

template<typename Derived, typename OtherDerived>
void hat(const Eigen::MatrixBase<Derived>& mat, const Eigen::MatrixBase<OtherDerived>& vec)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(OtherDerived, 3, 1);
  const_cast< Eigen::MatrixBase<Derived>& >(mat) << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
}

template<typename Derived, typename OtherDerived>
void relativeTransform(const Eigen::MatrixBase<Derived>& R1, const Eigen::MatrixBase<OtherDerived>& t1,
                       const Eigen::MatrixBase<Derived>& R2, const Eigen::MatrixBase<OtherDerived>& t2,
                       const Eigen::MatrixBase<Derived>& R , const Eigen::MatrixBase<OtherDerived>& t)
{
  const_cast< Eigen::MatrixBase<Derived>& >(R) = R1.transpose() * R2;
  const_cast< Eigen::MatrixBase<OtherDerived>& >(t) = R1.transpose() * (t2 - t1);
}

/// @brief compute the eigen vector and eigen vector of a matrix. dout is the eigen values, vout is the eigen vectors
template<typename Derived, typename EigenValue, typename EigenVector>
void eigen(const Eigen::MatrixBase<Derived>& m, const Eigen::MatrixBase<EigenValue>& dout, const Eigen::MatrixBase<EigenVector>& vout)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(EigenValue, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(EigenVector, 3, 3);
  typedef typename Derived::Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 3> Rot_t;
  typedef Eigen::Matrix<Scalar, 3, 1> Vec_t;

  Rot_t R(m.derived());
  int n = 3;
  int j, iq, ip, i;
  Scalar tresh, theta, tau, t, sm, s, h, g, c;
  int nrot;
  Vec_t b;
  Vec_t z;
  Rot_t v (Rot_t::Identity());
  Vec_t d;

  for(ip = 0; ip < n; ++ip)
  {
    b[ip] = d[ip] = R(ip, ip);
    z[ip] = 0;
  }

  nrot = 0;

  for(i = 0; i < 50; ++i)
  {
    sm = 0;
    for(ip = 0; ip < n; ++ip)
      for(iq = ip + 1; iq < n; ++iq)
        sm += std::abs(R(ip, iq));
    if(sm == 0.0)
    {
      const_cast < Eigen::MatrixBase<EigenVector>& > (vout).noalias() = v;
      const_cast < Eigen::MatrixBase<EigenValue>& > (dout).noalias() = d;
      return;
    }

    if(i < 3) tresh = 0.2 * sm / (n * n);
    else tresh = 0.0;

    for(ip = 0; ip < n; ++ip)
    {
      for(iq= ip + 1; iq < n; ++iq)
      {
        g = 100.0 * std::abs(R(ip, iq));
        if(i > 3 &&
           std::abs(d[ip]) + g == std::abs(d[ip]) &&
           std::abs(d[iq]) + g == std::abs(d[iq]))
          R(ip, iq) = 0.0;
        else if(std::abs(R(ip, iq)) > tresh)
        {
          h = d[iq] - d[ip];
          if(std::abs(h) + g == std::abs(h)) t = (R(ip, iq)) / h;
          else
          {
            theta = 0.5 * h / (R(ip, iq));
            t = 1.0 /(std::abs(theta) + std::sqrt(1.0 + theta * theta));
            if(theta < 0.0) t = -t;
          }
          c = 1.0 / std::sqrt(1 + t * t);
          s = t * c;
          tau = s / (1.0 + c);
          h = t * R(ip, iq);
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          R(ip, iq) = 0.0;
          for(j = 0; j < ip; ++j) { g = R(j, ip); h = R(j, iq); R(j, ip) = g - s * (h + g * tau); R(j, iq) = h + s * (g - h * tau); }
          for(j = ip + 1; j < iq; ++j) { g = R(ip, j); h = R(j, iq); R(ip, j) = g - s * (h + g * tau); R(j, iq) = h + s * (g - h * tau); }
          for(j = iq + 1; j < n; ++j) { g = R(ip, j); h = R(iq, j); R(ip, j) = g - s * (h + g * tau); R(iq, j) = h + s * (g - h * tau); }
          for(j = 0; j < n; ++j) { g = v(j,ip); h = v(j,iq); v(j,ip) = g - s * (h + g * tau); v(j,iq) = h + s * (g - h * tau); }
          nrot++;
        }
      }
    }
    for(ip = 0; ip < n; ++ip)
    {
      b[ip] += z[ip];
      d[ip] = b[ip];
      z[ip] = 0.0;
    }
  }

  std::cerr << "eigen: too many iterations in Jacobi transform." << std::endl;

  return;
}

template<typename Derived, typename OtherDerived>
typename Derived::Scalar quadraticForm(const Eigen::MatrixBase<Derived>& R, const Eigen::MatrixBase<OtherDerived>& v)
{
  return v.dot(R * v);
}

template<typename Derived, typename OtherDerived>
bool isEqual(const Eigen::MatrixBase<Derived>& lhs, const Eigen::MatrixBase<OtherDerived>& rhs, const FCL_REAL tol = std::numeric_limits<FCL_REAL>::epsilon()*100)
{
  return ((lhs - rhs).array().abs() < tol).all();
}

template <typename Derived>
inline Derived& setEulerZYX(const Eigen::MatrixBase<Derived>& R, FCL_REAL eulerX, FCL_REAL eulerY, FCL_REAL eulerZ)
{
  const_cast< Eigen::MatrixBase<Derived>& >(R).noalias() = (
      Eigen::AngleAxisd (eulerZ, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd (eulerY, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd (eulerX, Eigen::Vector3d::UnitX())
      ).toRotationMatrix();
  return const_cast< Eigen::MatrixBase<Derived>& >(R).derived();
}

template <typename Derived>
inline Derived& setEulerYPR(const Eigen::MatrixBase<Derived>& R, FCL_REAL yaw, FCL_REAL pitch, FCL_REAL roll)
{
  return setEulerZYX(R, roll, pitch, yaw);
}

}


#endif
