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

/** \author Jia Pan, Florent Lamiraux */

#include <hpp/fcl/BV/OBB.h>
#include "../src/BV/OBB.h"

#include <hpp/fcl/BVH/BVH_utility.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/internal/tools.h>

#include <iostream>
#include <limits>

namespace hpp
{
namespace fcl
{

/// @brief Compute the 8 vertices of a OBB
inline void computeVertices(const OBB& b, Vec3f vertices[8])
{
  Matrix3f extAxes (b.axes * b.extent.asDiagonal());
  vertices[0].noalias() = b.To + extAxes * Vec3f(-1,-1,-1);
  vertices[1].noalias() = b.To + extAxes * Vec3f( 1,-1,-1);
  vertices[2].noalias() = b.To + extAxes * Vec3f( 1, 1,-1);
  vertices[3].noalias() = b.To + extAxes * Vec3f(-1, 1,-1);
  vertices[4].noalias() = b.To + extAxes * Vec3f(-1,-1, 1);
  vertices[5].noalias() = b.To + extAxes * Vec3f( 1,-1, 1);
  vertices[6].noalias() = b.To + extAxes * Vec3f( 1, 1, 1);
  vertices[7].noalias() = b.To + extAxes * Vec3f(-1, 1, 1);
}

/// @brief OBB merge method when the centers of two smaller OBB are far away
inline OBB merge_largedist(const OBB& b1, const OBB& b2)
{
  OBB b;
  Vec3f vertex[16];
  computeVertices(b1, vertex);
  computeVertices(b2, vertex + 8);
  Matrix3f M;
  Vec3f E[3];
  Matrix3f::Scalar s[3] = {0, 0, 0};

  // obb axes
  b.axes.col(0).noalias() = (b1.To - b2.To).normalized();

  Vec3f vertex_proj[16];
  for(int i = 0; i < 16; ++i)
    vertex_proj[i].noalias() = vertex[i] - b.axes.col(0) * vertex[i].dot(b.axes.col(0));

  getCovariance(vertex_proj, NULL, NULL, NULL, 16, M);
  eigen(M, s, E);

  int min, mid, max;
  if (s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if (s[2] < s[min]) { mid = min; min = 2; }
  else if (s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }


  b.axes.col(1) << E[0][max], E[1][max], E[2][max];
  b.axes.col(2) << E[0][mid], E[1][mid], E[2][mid];

  // set obb centers and extensions
  Vec3f center, extent;
  getExtentAndCenter(vertex, NULL, NULL, NULL, 16, b.axes, center, extent);

  b.To.noalias() = center;
  b.extent.noalias() = extent;

  return b;
}


/// @brief OBB merge method when the centers of two smaller OBB are close
inline OBB merge_smalldist(const OBB& b1, const OBB& b2)
{
  OBB b;
  b.To = (b1.To + b2.To) * 0.5;
  Quaternion3f q0 (b1.axes), q1 (b2.axes);
  if(q0.dot(q1) < 0)
    q1.coeffs() *= -1;

  Quaternion3f q ((q0.coeffs() + q1.coeffs()).normalized());
  b.axes = q.toRotationMatrix();


  Vec3f vertex[8], diff;
  FCL_REAL real_max = std::numeric_limits<FCL_REAL>::max();
  Vec3f pmin(real_max, real_max, real_max);
  Vec3f pmax(-real_max, -real_max, -real_max);

  computeVertices(b1, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      FCL_REAL dot = diff.dot(b.axes.col(j));
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  computeVertices(b2, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      FCL_REAL dot = diff.dot(b.axes.col(j));
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  for(int j = 0; j < 3; ++j)
  {
    b.To.noalias() += (b.axes.col(j) * (0.5 * (pmax[j] + pmin[j])));
    b.extent[j] = 0.5 * (pmax[j] - pmin[j]);
  }

  return b;
}

bool obbDisjoint(const Matrix3f& B, const Vec3f& T, const Vec3f& a, const Vec3f& b)
{
  register FCL_REAL t, s;
  const FCL_REAL reps = 1e-6;

  Matrix3f Bf (B.array().abs() + reps);
  // Bf += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((T[0] < 0.0) ? -T[0] : T[0]);

  // if(t > (a[0] + Bf.dotX(b)))
  if(t > (a[0] + Bf.row(0).dot(b)))
    return true;

  // B1 x B2 = B0
  // s =  B.transposeDotX(T);
  s =  B.col(0).dot(T);
  t = ((s < 0.0) ? -s : s);

  // if(t > (b[0] + Bf.transposeDotX(a)))
  if(t > (b[0] + Bf.col(0).dot(a)))
    return true;

  // A2 x A0 = A1
  t = ((T[1] < 0.0) ? -T[1] : T[1]);

  // if(t > (a[1] + Bf.dotY(b)))
  if(t > (a[1] + Bf.row(1).dot(b)))
    return true;

  // A0 x A1 = A2
  t =((T[2] < 0.0) ? -T[2] : T[2]);

  // if(t > (a[2] + Bf.dotZ(b)))
  if(t > (a[2] + Bf.row(2).dot(b)))
    return true;

  // B2 x B0 = B1
  // s = B.transposeDotY(T);
  s = B.col(1).dot(T);
  t = ((s < 0.0) ? -s : s);

  // if(t > (b[1] + Bf.transposeDotY(a)))
  if(t > (b[1] + Bf.col(1).dot(a)))
    return true;

  // B0 x B1 = B2
  // s = B.transposeDotZ(T);
  s = B.col(2).dot(T);
  t = ((s < 0.0) ? -s : s);

  // if(t > (b[2] + Bf.transposeDotZ(a)))
  if(t > (b[2] + Bf.col(2).dot(a)))
    return true;

  // A0 x B0
  s = T[2] * B(1, 0) - T[1] * B(2, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
          b[1] * Bf(0, 2) + b[2] * Bf(0, 1)))
    return true;

  // A0 x B1
  s = T[2] * B(1, 1) - T[1] * B(2, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
          b[0] * Bf(0, 2) + b[2] * Bf(0, 0)))
    return true;

  // A0 x B2
  s = T[2] * B(1, 2) - T[1] * B(2, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
          b[0] * Bf(0, 1) + b[1] * Bf(0, 0)))
    return true;

  // A1 x B0
  s = T[0] * B(2, 0) - T[2] * B(0, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
          b[1] * Bf(1, 2) + b[2] * Bf(1, 1)))
    return true;

  // A1 x B1
  s = T[0] * B(2, 1) - T[2] * B(0, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
          b[0] * Bf(1, 2) + b[2] * Bf(1, 0)))
    return true;

  // A1 x B2
  s = T[0] * B(2, 2) - T[2] * B(0, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
          b[0] * Bf(1, 1) + b[1] * Bf(1, 0)))
    return true;

  // A2 x B0
  s = T[1] * B(0, 0) - T[0] * B(1, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
          b[1] * Bf(2, 2) + b[2] * Bf(2, 1)))
    return true;

  // A2 x B1
  s = T[1] * B(0, 1) - T[0] * B(1, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
          b[0] * Bf(2, 2) + b[2] * Bf(2, 0)))
    return true;

  // A2 x B2
  s = T[1] * B(0, 2) - T[0] * B(1, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
          b[0] * Bf(2, 1) + b[1] * Bf(2, 0)))
    return true;

  return false;

}

/****************** obbDisjointAndLowerBoundDistance **************************/

namespace internal
{
  inline FCL_REAL obbDisjoint_check_A_axis (
      const Vec3f& T, const Vec3f& a, const Vec3f& b, const Matrix3f& Bf)
  {
    // |T| - |B| * b - a
    Vec3f AABB_corner (T.cwiseAbs () - a);
    AABB_corner.noalias() -= Bf.col(0) * b[0];
    AABB_corner.noalias() -= Bf.col(1) * b[1];
    AABB_corner.noalias() -= Bf.col(2) * b[2];
    return AABB_corner.array().max(0).matrix().squaredNorm ();
  }

  inline FCL_REAL obbDisjoint_check_B_axis (
      const Matrix3f& B, const Vec3f& T, const Vec3f& a, const Vec3f& b, const Matrix3f& Bf)
  {
    // Bf = |B|
    // | B^T T| - Bf^T * a - b
    register FCL_REAL s, t = 0;
    s = std::abs(B.col(0).dot(T)) - Bf.col(0).dot(a) - b[0];
    if (s > 0) t += s*s;
    s = std::abs(B.col(1).dot(T)) - Bf.col(1).dot(a) - b[1];
    if (s > 0) t += s*s;
    s = std::abs(B.col(2).dot(T)) - Bf.col(2).dot(a) - b[2];
    if (s > 0) t += s*s;
    return t;
  }

  template <int ib, int jb = (ib+1)%3, int kb = (ib+2)%3 >
  struct obbDisjoint_check_Ai_cross_Bi
  {
    static inline bool run (int ia, int ja, int ka,
        const Matrix3f& B, const Vec3f& T, const Vec3f& a, const Vec3f& b,
        const Matrix3f& Bf,
        const FCL_REAL& breakDistance2, FCL_REAL& squaredLowerBoundDistance)
    {
      FCL_REAL sinus2 = 1 - Bf (ia,ib) * Bf (ia,ib);
      if (sinus2 < 1e-6) return false;

      const FCL_REAL s = T[ka] * B(ja, ib) - T[ja] * B(ka, ib);

      const FCL_REAL diff = fabs(s) - (a[ja] * Bf(ka, ib) + a[ka] * Bf(ja, ib) +
                                       b[jb] * Bf(ia, kb) + b[kb] * Bf(ia, jb));
      // We need to divide by the norm || Aia x Bib ||
      // As ||Aia|| = ||Bib|| = 1, (Aia | Bib)^2  = cosine^2
      if (diff > 0) {
        squaredLowerBoundDistance = diff * diff / sinus2;
        if (squaredLowerBoundDistance > breakDistance2) {
          return true;
        }
      }
      return false;
    }
  };
}

bool obbDisjointAndLowerBoundDistance (const Matrix3f& B, const Vec3f& T,
                                       const Vec3f& a, const Vec3f& b,
                                       const CollisionRequest& request,
                                       FCL_REAL& squaredLowerBoundDistance)
{
  const FCL_REAL breakDistance (request.break_distance + request.security_margin);
  const FCL_REAL breakDistance2 = breakDistance * breakDistance;

  Matrix3f Bf (B.cwiseAbs());

  // Corner of b axis aligned bounding box the closest to the origin
  squaredLowerBoundDistance = internal::obbDisjoint_check_A_axis (T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2)
    return true;

  squaredLowerBoundDistance = internal::obbDisjoint_check_B_axis (B, T, a, b, Bf);
  if (squaredLowerBoundDistance > breakDistance2)
    return true;

  // Ai x Bj
  int ja = 1, ka = 2;
  for (int ia = 0; ia < 3; ++ia) {
    if (internal::obbDisjoint_check_Ai_cross_Bi<0>::run (ia, ja, ka,
          B, T, a, b, Bf, breakDistance2, squaredLowerBoundDistance)) return true;
    if (internal::obbDisjoint_check_Ai_cross_Bi<1>::run (ia, ja, ka,
          B, T, a, b, Bf, breakDistance2, squaredLowerBoundDistance)) return true;
    if (internal::obbDisjoint_check_Ai_cross_Bi<2>::run (ia, ja, ka,
          B, T, a, b, Bf, breakDistance2, squaredLowerBoundDistance)) return true;
    ja = ka; ka = ia;
  }

  return false;
}

/****************** obbDisjointAndLowerBoundDistance **************************/

#ifdef HPP_FCL_VECTORIZE_OBB_DISJOINT
namespace internal {
template <int N>
EIGEN_STRONG_INLINE void aligned_obbDisjoint_check_A_axis (
    const typename AlignedMatrix3f<N>::AlignedVec3f& T,
    const typename AlignedMatrix3f<N>::AlignedVec3f& a,
    const typename AlignedMatrix3f<N>::AlignedVec3f& b,
    const AlignedMatrix3f<N>& Bf,
    Eigen::Array<FCL_REAL, N, 1>& squaredLowerBoundDistance)
{
  // |B| * b - |T| + a
  typename AlignedMatrix3f<N>::AlignedVec3f AABB_corner;
  Bf.mult(b, AABB_corner);
  AABB_corner += a - T.cwiseAbs();
  squaredLowerBoundDistance =
         AABB_corner.array().col(0).min(0).abs2()
    +    AABB_corner.array().col(1).min(0).abs2()
    +    AABB_corner.array().col(2).min(0).abs2();
}

template <int N>
EIGEN_STRONG_INLINE void aligned_obbDisjoint_check_B_axis (
    const AlignedMatrix3f<N>& B,
    const typename AlignedMatrix3f<N>::AlignedVec3f& T,
    const typename AlignedMatrix3f<N>::AlignedVec3f& a,
    const typename AlignedMatrix3f<N>::AlignedVec3f& b,
    const AlignedMatrix3f<N>& Bf,
    Eigen::Array<FCL_REAL, N, 1>& squaredLowerBoundDistance)
{
  // Bf = |B|
  // res = - | B^T T| + Bf^T * a + b
  typename AlignedMatrix3f<N>::AlignedArr3f res;

  for (int k = 0; k < 3; ++k) {
    res.col(k) = // Bf^T a
      ( Bf(0,k).array() * a.array().col(0)
      + Bf(1,k).array() * a.array().col(1)
      + Bf(2,k).array() * a.array().col(2));
    // | B^T T |
    res.col(k) -=
      ( B(0,k).array() * T.array().col(0)
      + B(1,k).array() * T.array().col(1)
      + B(2,k).array() * T.array().col(2)).abs();
    res.col(k) += b.array().col(k);
    res.col(k) = res.col(k).min(0).abs2();
  }
  squaredLowerBoundDistance = squaredLowerBoundDistance.max(res.rowwise().sum());
}

template <int N>
EIGEN_STRONG_INLINE void aligned_obbDisjoint_check_Ai_cross_Bi (int ia, int ja, int ka,
    int ib, int jb, int kb,
    const AlignedMatrix3f<N>& B,
    const typename AlignedMatrix3f<N>::AlignedVec3f& T,
    const typename AlignedMatrix3f<N>::AlignedVec3f& a,
    const typename AlignedMatrix3f<N>::AlignedVec3f& b,
    const AlignedMatrix3f<N>& Bf,
    Eigen::Array<FCL_REAL, N, 1>& squaredLowerBoundDistance)
{
  const Eigen::Array<FCL_REAL, N, 1> s (
      T.col(ka).array() * B(ja, ib).array()
      - T.col(ja).array() * B(ka, ib).array());

  Eigen::Array<FCL_REAL, N, 1> diff (s.abs());
  diff -= a.col(ja).array() * Bf(ka, ib).array();
  diff -= a.col(ka).array() * Bf(ja, ib).array();
  diff -= b.col(jb).array() * Bf(ia, kb).array();
  diff -= b.col(kb).array() * Bf(ia, jb).array();
  diff = diff.max(0).abs2();
  // We need to divide by the norm || Aia x Bib ||
  // As ||Aia|| = ||Bib|| = 1, (Aia | Bib)^2  = cosine^2
  const Eigen::Array<FCL_REAL, N, 1> sinus2 (1 - Bf(ia,ib).array().abs2());
  diff = squaredLowerBoundDistance.max(diff / sinus2),

  squaredLowerBoundDistance = (sinus2 > 1e-6).select(
      diff, squaredLowerBoundDistance);
}

} // namespace internal

template <int N, bool EarlyStop>
bool obbDisjointAndLowerBoundDistance (
    const AlignedMatrix3f<N>& B,
    const typename AlignedMatrix3f<N>::AlignedVec3f& T,
    const typename AlignedMatrix3f<N>::AlignedVec3f& a,
    const typename AlignedMatrix3f<N>::AlignedVec3f& b,
    const CollisionRequest& request,
    Eigen::Array<FCL_REAL, N, 1>& squaredLowerBoundDistance,
    Eigen::Array<bool, N, 1>& ok)
{
  const FCL_REAL breakDistance (request.break_distance + request.security_margin);
  const FCL_REAL breakDistance2 = breakDistance * breakDistance;

  AlignedMatrix3f<N> Bf;
  Eigen::Array<FCL_REAL, N, 1> d;
  Bf.storage = B.storage.cwiseAbs();

  // Corner of b axis aligned bounding box the closest to the origin
  internal::aligned_obbDisjoint_check_A_axis (T, a, b, Bf, d);
  if (EarlyStop) {
    ok = (d > breakDistance2);
    if (ok.all()) {
      squaredLowerBoundDistance = d;
      return true;
    }
  }

  internal::aligned_obbDisjoint_check_B_axis (B, T, a, b, Bf, d);
  if (EarlyStop) {
    ok = (d > breakDistance2);
    if (ok.all()) {
      squaredLowerBoundDistance = d;
      return true;
    }
  }

  // Ai x Bj
  int ja = 1, ka = 2;
  for (int ia = 0; ia < 3; ++ia) {
    int jb = 1, kb = 2;
    for (int ib = 0; ib < 3; ++ib) {
      internal::aligned_obbDisjoint_check_Ai_cross_Bi (ia, ja, ka, ib, jb, kb,
          B, T, a, b, Bf, d);
      if (EarlyStop) {
        ok = (d > breakDistance2);
        if (ok.all()) {
          squaredLowerBoundDistance = d;
          return true;
        }
      }
      jb = kb; kb = ib;
    }
    ja = ka; ka = ia;
  }

  squaredLowerBoundDistance = d;
  if (!EarlyStop) {
    ok = (d > breakDistance2);
    if (ok.all()) return true;
  }
  return false;
}

#define OBB_DISJOINT_SPECIALIZATION(N,EarlyStop)                               \
template bool obbDisjointAndLowerBoundDistance<N, EarlyStop> (                 \
    const AlignedMatrix3f<N>& B,                                               \
    const typename AlignedMatrix3f<N>::AlignedVec3f& T,                        \
    const typename AlignedMatrix3f<N>::AlignedVec3f& a,                        \
    const typename AlignedMatrix3f<N>::AlignedVec3f& b,                        \
    const CollisionRequest& request,                                           \
    Eigen::Array<FCL_REAL, N, 1>& squaredLowerBoundDistance,                   \
    Eigen::Array<bool, N, 1>& ok)

OBB_DISJOINT_SPECIALIZATION(HPP_FCL_VECTORIZE_OBB_DISJOINT, true);
OBB_DISJOINT_SPECIALIZATION(HPP_FCL_VECTORIZE_OBB_DISJOINT, false);

#undef OBB_DISJOINT_SPECIALIZATION
#endif

/*************************** OBB functions ************************************/

bool OBB::overlap(const OBB& other) const
{
  /// compute what transform [R,T] that takes us from cs1 to cs2.
  /// [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  /// First compute the rotation part, then translation part
  Vec3f T (axes.transpose() * (other.To - To));
  Matrix3f R (axes.transpose() * other.axes);

  return !obbDisjoint(R, T, extent, other.extent);
}

bool OBB::overlap(const OBB& other, const CollisionRequest& request,
    FCL_REAL& sqrDistLowerBound) const
{
    // compute what transform [R,T] that takes us from cs1 to cs2.
  Vec3f T (axes.transpose() * (other.To - To));
  Matrix3f R (axes.transpose() * other.axes);

  return !obbDisjointAndLowerBoundDistance
    (R, T, extent, other.extent, request, sqrDistLowerBound);
}

bool OBB::contain(const Vec3f& p) const
{
  Vec3f local_p (p - To);
  FCL_REAL proj = local_p.dot(axes.col(0));
  if((proj > extent[0]) || (proj < -extent[0]))
    return false;

  proj = local_p.dot(axes.col(1));
  if((proj > extent[1]) || (proj < -extent[1]))
    return false;

  proj = local_p.dot(axes.col(2));
  if((proj > extent[2]) || (proj < -extent[2]))
    return false;

  return true;
}

OBB& OBB::operator += (const Vec3f& p)
{
  OBB bvp;
  bvp.To = p;
  bvp.axes.noalias() = axes;
  bvp.extent.setZero();

  *this += bvp;
  return *this;
}

OBB OBB::operator + (const OBB& other) const
{
  Vec3f center_diff = To - other.To;
  FCL_REAL max_extent = std::max(std::max(extent[0], extent[1]), extent[2]);
  FCL_REAL max_extent2 = std::max(std::max(other.extent[0], other.extent[1]), other.extent[2]);
  if(center_diff.norm() > 2 * (max_extent + max_extent2))
  {
    return merge_largedist(*this, other);
  }
  else
  {
    return merge_smalldist(*this, other);
  }
}


FCL_REAL OBB::distance(const OBB& /*other*/, Vec3f* /*P*/, Vec3f* /*Q*/) const
{
  std::cerr << "OBB distance not implemented!" << std::endl;
  return 0.0;
}


bool overlap(const Matrix3f& R0, const Vec3f& T0, const OBB& b1, const OBB& b2)
{
  Vec3f Ttemp (R0 * b2.To + T0 - b1.To);
  Vec3f T (b1.axes.transpose() * Ttemp);
  Matrix3f R (b1.axes.transpose() * R0 * b2.axes);

  return !obbDisjoint(R, T, b1.extent, b2.extent);
}

bool overlap(const Matrix3f& R0, const Vec3f& T0, const OBB& b1, const OBB& b2,
	     const CollisionRequest& request, FCL_REAL& sqrDistLowerBound)
{
  Vec3f Ttemp (R0 * b2.To + T0 - b1.To);
  Vec3f T (b1.axes.transpose() * Ttemp);
  Matrix3f R (b1.axes.transpose() * R0 * b2.axes);

  return !obbDisjointAndLowerBoundDistance (R, T, b1.extent, b2.extent,
					    request, sqrDistLowerBound);
}

OBB translate(const OBB& bv, const Vec3f& t)
{
  OBB res(bv);
  res.To += t;
  return res;
}

}

} // namespace hpp
