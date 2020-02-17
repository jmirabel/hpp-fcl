/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2018-2019, Centre National de la Recherche Scientifique
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

#ifndef HPP_FCL_NARROWPHASE_H
#define HPP_FCL_NARROWPHASE_H

#include <hpp/fcl/narrowphase/gjk.h>
#include <hpp/fcl/collision_data.h>

namespace hpp
{
namespace fcl
{




  /// @brief collision and distance solver based on GJK algorithm implemented in fcl (rewritten the code from the GJK in bullet)
  struct GJKSolver
  {
    /// @brief intersection checking between two shapes
    template<typename S1, typename S2>
      bool shapeIntersect(const S1& s1, const Transform3f& tf1,
                          const S2& s2, const Transform3f& tf2,
                          Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const
    {
      Vec3f guess(1, 0, 0);
      if(enable_cached_guess) guess = cached_guess;
    
      details::MinkowskiDiff shape;
      shape.set (&s1, &s2, tf1, tf2);
  
      details::GJK gjk((unsigned int )gjk_max_iterations, gjk_tolerance);
      gjk.setDistanceEarlyBreak(security_margin + break_distance);
      details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
      if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();
    
      switch(gjk_status)
        {
        case details::GJK::Inside:
          {
            details::EPA epa(epa_max_face_num, epa_max_vertex_num, epa_max_iterations, epa_tolerance);
            details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
            if(epa_status != details::EPA::Failed)
              {
                Vec3f w0, w1;
                details::GJK::getClosestPoints (epa.result, w0, w1);
                if(penetration_depth) *penetration_depth = -epa.depth;
                if(normal) *normal = tf2.getRotation() * epa.normal;
                if(contact_points) *contact_points = tf1.transform(w0 - epa.normal*(epa.depth *0.5));
                return true;
              }
            else return false;
          }
          break;
        default:
          ;
        }

      return false;
    }

    //// @brief intersection checking between one shape and a triangle with transformation
    /// @return true if the shape are colliding.
    template<typename S>
    bool shapeTriangleInteraction
    (const S& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
     const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
     Vec3f& p1, Vec3f& p2, Vec3f& normal) const
    {
      bool col;
      // Express everything in frame 1
      const Transform3f tf_1M2 (tf1.inverseTimes(tf2));
      TriangleP tri(
          tf_1M2.transform (P1),
          tf_1M2.transform (P2),
          tf_1M2.transform (P3));

      Vec3f guess(1, 0, 0);
      if(enable_cached_guess) guess = cached_guess;

      details::MinkowskiDiff shape;
      shape.set (&s, &tri);
  
      details::GJK gjk((unsigned int )gjk_max_iterations, gjk_tolerance);
      gjk.setDistanceEarlyBreak(security_margin + break_distance);
      details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
      if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

      switch(gjk_status)
        {
        case details::GJK::Inside:
          {
            col = true;
            details::EPA epa(epa_max_face_num, epa_max_vertex_num, epa_max_iterations, epa_tolerance);
            details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
            assert (epa_status != details::EPA::Failed); (void) epa_status;
            Vec3f w0, w1;
            details::GJK::getClosestPoints (epa.result, w0, w1);
            distance = -epa.depth;
            normal = -epa.normal;
            p1 = p2 = tf1.transform(w0 - epa.normal*(epa.depth *0.5));
            assert (distance <= 1e-6);
            break;
          }
        case details::GJK::Valid:
        case details::GJK::Failed:
          {
            col = false;

            details::GJK::getClosestPoints (*gjk.getSimplex(), p1, p2);
            // TODO On degenerated case, the closest point may be wrong
            // (i.e. an object face normal is colinear to gjk.ray
            // assert (distance == (w0 - w1).norm());
            distance = gjk.distance;

            p1 = tf1.transform (p1);
            p2 = tf1.transform (p2);
            assert (distance > 0);
          }
          break;
        default:
          assert (false && "should not reach type part.");
          return true;
        }
      return col;
    }

    /// @brief distance computation between two shapes
    template<typename S1, typename S2>
      bool shapeDistance(const S1& s1, const Transform3f& tf1,
                         const S2& s2, const Transform3f& tf2,
                         FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
                         Vec3f& normal) const
    {
#ifndef NDEBUG
      FCL_REAL eps (sqrt(std::numeric_limits<FCL_REAL>::epsilon()));
#endif
      bool compute_normal (true);
      Vec3f guess(1, 0, 0);
      if(enable_cached_guess) guess = cached_guess;

      details::MinkowskiDiff shape;
      shape.set (&s1, &s2, tf1, tf2);

      details::GJK gjk((unsigned int) gjk_max_iterations, gjk_tolerance);
      details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
      if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

      if(gjk_status == details::GJK::Failed)
      {
        // TODO: understand why GJK fails between cylinder and box
        assert (distance * distance < sqrt (eps));
        Vec3f w0, w1;
        details::GJK::getClosestPoints (*gjk.getSimplex(), w0, w1);
        distance = 0;
        p1 = p2 = tf1.transform (.5* (w0 + w1));
        normal = Vec3f (0,0,0);
        return false;
      }
      else if(gjk_status == details::GJK::Valid)
        {
          details::GJK::getClosestPoints (*gjk.getSimplex(), p1, p2);
          // TODO On degenerated case, the closest point may be wrong
          // (i.e. an object face normal is colinear to gjk.ray
          // assert (distance == (w0 - w1).norm());
          distance = gjk.distance;

          p1 = tf1.transform (p1);
          p2 = tf1.transform (p2);
          return true;
        }
      else
        {
          assert (gjk_status == details::GJK::Inside);
          if (compute_normal)
            {
              details::EPA epa(epa_max_face_num, epa_max_vertex_num,
                               epa_max_iterations, epa_tolerance);
              details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
              if(epa_status != details::EPA::Failed)
                {
                  Vec3f w0, w1;
                  details::GJK::getClosestPoints (epa.result, w0, w1);
                  assert (epa.depth >= -eps);
                  distance = std::min (0., -epa.depth);
                  normal = tf2.getRotation() * epa.normal;
                  p1 = p2 = tf1.transform(w0 - epa.normal*(epa.depth *0.5));
                }
            }
          else
            {
              details::GJK::getClosestPoints (*gjk.getSimplex(), p1, p2);
              distance = 0;

              p1 = tf1.transform (p1);
              p2 = tf1.transform (p2);
            }
          return false;
        }
    }

    /// @brief default setting for GJK algorithm
    GJKSolver() { init(); }

    GJKSolver(const CollisionRequest& request)
    {
      init();

      security_margin = request.security_margin;
      break_distance = request.break_distance;

      if (request.enable_cached_gjk_guess) {
        enable_cached_guess = true;
        cached_guess = request.cached_gjk_guess;
      }
    }

    GJKSolver(const DistanceRequest&)
    {
      init();
    }

    inline void init()
    {
      security_margin = 0;
      break_distance = 0.001;
      gjk_max_iterations = 128;
      gjk_tolerance = 1e-6;
      epa_max_face_num = 128;
      epa_max_vertex_num = 64;
      epa_max_iterations = 255;
      epa_tolerance = 1e-6;
      enable_cached_guess = false;
      cached_guess = Vec3f(1, 0, 0);
    }

    void enableCachedGuess(bool if_enable) const
    {
      enable_cached_guess = if_enable;
    }

    void setCachedGuess(const Vec3f& guess) const
    {
      cached_guess = guess;
    }

    Vec3f getCachedGuess() const
    {
      return cached_guess;
    }

    /// \copydoc CollisionRequest::security_margin
    FCL_REAL security_margin;

    /// \copydoc CollisionRequest::break_distance
    FCL_REAL break_distance;

    /// @brief maximum number of simplex face used in EPA algorithm
    unsigned int epa_max_face_num;

    /// @brief maximum number of simplex vertex used in EPA algorithm
    unsigned int epa_max_vertex_num;

    /// @brief maximum number of iterations used for EPA iterations
    unsigned int epa_max_iterations;

    /// @brief the threshold used in EPA to stop iteration
    FCL_REAL epa_tolerance;

    /// @brief the threshold used in GJK to stop iteration
    FCL_REAL gjk_tolerance;

    /// @brief maximum number of iterations used for GJK iterations
    FCL_REAL gjk_max_iterations;

    /// @brief Whether smart guess can be provided
    mutable bool enable_cached_guess;

    /// @brief smart guess
    mutable Vec3f cached_guess;
  };

#define _GJK_SOLVER_DECLARE_SPECIALIZATION(S1,S2)                              \
  template<> bool GJKSolver::shapeIntersect<S1, S2>(                           \
      const S1& s1, const Transform3f& tf1,                                    \
      const S2& s2, const Transform3f& tf2,                                    \
      Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const

#define GJK_SOLVER_DECLARE_SPECIALIZATION(S1)                                  \
  _GJK_SOLVER_DECLARE_SPECIALIZATION(S1,S1)

#define GJK_SOLVER_DECLARE_SPECIALIZATIONS(S1,S2)                              \
  _GJK_SOLVER_DECLARE_SPECIALIZATION(S1,S2);                                   \
  _GJK_SOLVER_DECLARE_SPECIALIZATION(S2,S1)

  /// @brief Fast implementation for sphere-capsule collision
  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Sphere, Capsule);

  /// @brief Fast implementation for sphere-sphere collision
  GJK_SOLVER_DECLARE_SPECIALIZATION(Sphere);

  /// @brief Fast implementation for box-box collision
  GJK_SOLVER_DECLARE_SPECIALIZATION(Box);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Sphere, Halfspace);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Box, Halfspace);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Capsule, Halfspace);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Cylinder, Halfspace);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Cone, Halfspace);

  GJK_SOLVER_DECLARE_SPECIALIZATION(Halfspace);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Plane, Halfspace);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Sphere, Plane);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Box, Plane);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Capsule, Plane);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Cylinder, Plane);

  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Cone, Plane);

  GJK_SOLVER_DECLARE_SPECIALIZATION(Plane);

#undef _GJK_SOLVER_DECLARE_SPECIALIZATION
#undef GJK_SOLVER_DECLARE_SPECIALIZATION
#undef GJK_SOLVER_DECLARE_SPECIALIZATIONS

#define GJK_SOLVER_DECLARE_SPECIALIZATION(S1)                                  \
  template<> bool GJKSolver::shapeTriangleInteraction<S1>(                     \
      const S1& s1, const Transform3f& tf1,                                    \
      const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& M2,\
      FCL_REAL& distance, Vec3f& p1, Vec3f& p2, Vec3f& normal) const

  /// @brief Fast implementation for sphere-triangle collision
  GJK_SOLVER_DECLARE_SPECIALIZATION(Sphere);

  GJK_SOLVER_DECLARE_SPECIALIZATION(Halfspace);

  GJK_SOLVER_DECLARE_SPECIALIZATION(Plane);

#undef GJK_SOLVER_DECLARE_SPECIALIZATION

#define _GJK_SOLVER_DECLARE_SPECIALIZATION(S1,S2)                              \
  template<> bool GJKSolver::shapeDistance<S1, S2>(                            \
      const S1& s1, const Transform3f& tf1,                                    \
      const S2& s2, const Transform3f& tf2,                                    \
     FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const

#define GJK_SOLVER_DECLARE_SPECIALIZATION(S1)                                  \
  _GJK_SOLVER_DECLARE_SPECIALIZATION(S1,S1)

#define GJK_SOLVER_DECLARE_SPECIALIZATIONS(S1,S2)                              \
  _GJK_SOLVER_DECLARE_SPECIALIZATION(S1,S2);                                   \
  _GJK_SOLVER_DECLARE_SPECIALIZATION(S2,S1)

  /// @brief Fast implementation for sphere-capsule distance
  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Sphere, Capsule);

  /// @brief Fast implementation for sphere-cylinder distance
  GJK_SOLVER_DECLARE_SPECIALIZATIONS(Sphere, Cylinder);

  /// @brief Fast implementation for sphere-sphere distance
  GJK_SOLVER_DECLARE_SPECIALIZATION(Sphere);

  /// @brief Computation of the distance result for capsule capsule.
  /// Closest points are based on two line-segments.
  GJK_SOLVER_DECLARE_SPECIALIZATION(Capsule);

  /// Distance computation between two triangles
  ///
  /// Do not run EPA algorithm to compute penetration depth, use a dedicated
  /// method.
  GJK_SOLVER_DECLARE_SPECIALIZATION(TriangleP);

#undef _GJK_SOLVER_DECLARE_SPECIALIZATION
#undef GJK_SOLVER_DECLARE_SPECIALIZATION
#undef GJK_SOLVER_DECLARE_SPECIALIZATIONS

}

} // namespace hpp

#endif
