/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, CNRS
 *  Author: Florent Lamiraux
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
 *   * Neither the name of CNRS nor the names of its
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

#ifndef HPP_FCL_SRC_OBB_H
# define HPP_FCL_SRC_OBB_H

#include "../src/aligned-math.h"

#if defined EIGEN_VECTORIZE_AVX512
#define HPP_FCL_VECTORIZE_OBB_DISJOINT 8
#elif defined EIGEN_VECTORIZE_AVX2
#define HPP_FCL_VECTORIZE_OBB_DISJOINT 4
#endif

namespace hpp
{
namespace fcl
{

  /// This function tests whether bounding boxes should be broken down.
  ///
  /// \param B, T orientation and position of 2nd OBB in frame of 1st OBB,
  /// \param a, b extent of 1st and 2nd OBB.
  bool obbDisjointAndLowerBoundDistance (const Matrix3f& B, const Vec3f& T,
					 const Vec3f& a, const Vec3f& b,
                                         const CollisionRequest& request,
					 FCL_REAL& squaredLowerBoundDistance);

#ifdef HPP_FCL_VECTORIZE_OBB_DISJOINT
  /// Vectorized version of obbDisjointAndLowerBoundDistance
  /// \tparam N the number of OBB pairs to be considered.
  /// \tparam EarlyStop turns on the checks on the squaredLowerBoundDistance
  ///                   to know whether the algo can stop. In practice, enabling
  ///                   this seems to be slower.
  template <int N, bool EarlyStop = false>
  bool obbDisjointAndLowerBoundDistance (
      const AlignedMatrix3f<N>& B,
      const typename AlignedMatrix3f<N>::AlignedVec3f& T,
      const typename AlignedMatrix3f<N>::AlignedVec3f& a,
      const typename AlignedMatrix3f<N>::AlignedVec3f& b,
      const CollisionRequest& request,
      Eigen::Array<FCL_REAL, N, 1>& squaredLowerBoundDistance,
      Eigen::Array<bool, N, 1>& ok);
#endif

  bool obbDisjoint(const Matrix3f& B, const Vec3f& T, const Vec3f& a,
		   const Vec3f& b);
} // namespace fcl

} // namespace hpp

#endif // HPP_FCL_SRC_OBB_H
