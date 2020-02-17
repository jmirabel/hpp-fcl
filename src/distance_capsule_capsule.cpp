/*
 * Software License Agreement (BSD License)
 *  Copyright (c) 2015-2019, CNRS - LAAS INRIA
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

#include <cmath>
#include <limits>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <../src/distance_func_matrix.h>

// Note that partial specialization of template functions is not allowed.
// Therefore, two implementations with the default narrow phase solvers are
// provided. If another narrow phase solver were to be used, the default
// template implementation would be called, unless the function is also
// specialized for this new type.
//
// One solution would be to make narrow phase solvers derive from an abstract
// class and specialize the template for this abstract class.
namespace hpp
{
namespace fcl {
  class GJKSolver;


  // Compute the distance between C1 and C2 by computing the distance
  // between the two segments supporting the capsules.
  // Match algorithm of Real-Time Collision Detection, Christer Ericson - Closest Point of Two Line Segments
  template <>
  FCL_REAL ShapeShapeDistance <Capsule, Capsule> (const CollisionGeometry* o1, const Transform3f& tf1,
   const CollisionGeometry* o2, const Transform3f& tf2,
   const GJKSolver*, const DistanceRequest& request,
   DistanceResult& result)
  {
    details::capsuleCapsuleDistance (
        static_cast <const Capsule&> (*o1), tf1,
        static_cast <const Capsule&> (*o2), tf2,
        result.distance,
        result.nearest_points[0],
        result.nearest_points[1],
        result.normal)
  }

} // namespace fcl

} // namespace hpp
