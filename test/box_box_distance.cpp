/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, CNRS-LAAS.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Florent Lamiraux <florent@laas.fr> */

#define BOOST_TEST_MODULE FCL_BOX_BOX
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#define CHECK_CLOSE_TO_0(x, eps) BOOST_CHECK_CLOSE ((x + 1.0), (1.0), (eps))

#include <cmath>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/mesh_loader/loader.h>

#include "utility.h"
#include "fcl_resources/config.h"

#ifdef HPP_FCL_WITH_SCH_CORE
# include <hpp/fcl/sch/object.h>
#endif

typedef boost::shared_ptr <hpp::fcl::CollisionGeometry> CollisionGeometryPtr_t;

using hpp::fcl::Transform3f;
using hpp::fcl::Vec3f;
using hpp::fcl::Quaternion3f;
using hpp::fcl::CollisionObject;
using hpp::fcl::DistanceResult;
using hpp::fcl::DistanceRequest;
using hpp::fcl::GST_INDEP;

void distance_box_box_1 (CollisionGeometryPtr_t s1, CollisionGeometryPtr_t s2)
{
  Transform3f tf1;
  Transform3f tf2 (Vec3f(25, 20, 5));

  CollisionObject o1 (s1, tf1);
  CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest (true, 0, 0, GST_INDEP);
  DistanceResult distanceResult;

  hpp::fcl::distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
	    << " R1 = " << tf1.getRotation () << std::endl
	    << " T2 = " << tf2.getTranslation() << std::endl
	    << " R2 = " << tf2.getRotation () << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;
  double dx = 25 - 3 - 1;
  double dy = 20 - 5 - 1;
  double dz = 5 - 1 - 1;

  const Vec3f& p1 = distanceResult.nearest_points [0];
  const Vec3f& p2 = distanceResult.nearest_points [1];
  BOOST_CHECK_CLOSE(distanceResult.min_distance, 
		    sqrt (dx*dx + dy*dy + dz*dz), 1e-4);

  BOOST_CHECK_CLOSE (p1 [0], 3, 1e-6);
  BOOST_CHECK_CLOSE (p1 [1], 5, 1e-6);
  BOOST_CHECK_CLOSE (p1 [2], 1, 1e-6);
  BOOST_CHECK_CLOSE (p2 [0], 24, 1e-6);
  BOOST_CHECK_CLOSE (p2 [1], 19, 1e-6);
  BOOST_CHECK_CLOSE (p2 [2], 4, 1e-6);
}

BOOST_AUTO_TEST_CASE(distance_box_box_1_fcl)
{
  CollisionGeometryPtr_t s1 (new hpp::fcl::Box (6, 10, 2));
  CollisionGeometryPtr_t s2 (new hpp::fcl::Box (2, 2, 2));
  distance_box_box_1 (s1, s2);
}

#ifdef HPP_FCL_WITH_SCH_CORE
BOOST_AUTO_TEST_CASE(distance_box_box_1_sch)
{
  CollisionGeometryPtr_t s1 (new hpp::fcl::sch::S_Box (6, 10, 2));
  CollisionGeometryPtr_t s2 (new hpp::fcl::sch::S_Box (2, 2, 2));
  distance_box_box_1 (s1, s2);
}
#endif

void distance_box_box_2 (CollisionGeometryPtr_t s1, CollisionGeometryPtr_t s2)
{
  static double pi = M_PI;
  Transform3f tf1;
  Transform3f tf2 (hpp::fcl::makeQuat (cos (pi/8), sin(pi/8)/sqrt(3),
				 sin(pi/8)/sqrt(3), sin(pi/8)/sqrt(3)),
		   Vec3f(0, 0, 10));

  CollisionObject o1 (s1, tf1);
  CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest (true, 0, 0, GST_INDEP);
  DistanceResult distanceResult;

  hpp::fcl::distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
	    << " R1 = " << tf1.getRotation () << std::endl
	    << " T2 = " << tf2.getTranslation() << std::endl
	    << " R2 = " << tf2.getRotation () << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;

  const Vec3f& p1 = distanceResult.nearest_points [0];
  const Vec3f& p2 = distanceResult.nearest_points [1];
  double distance = -1.62123444 + 10 - 1;
  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 1e-4);

  BOOST_CHECK_CLOSE (p1 [0], 0.60947571, 1e-4);
  BOOST_CHECK_CLOSE (p1 [1], 0.01175873, 1e-4);
  BOOST_CHECK_CLOSE (p1 [2], 1, 1e-6);
  BOOST_CHECK_CLOSE (p2 [0], 0.60947571, 1e-4);
  BOOST_CHECK_CLOSE (p2 [1], 0.01175873, 1e-4);
  BOOST_CHECK_CLOSE (p2 [2], -1.62123444 + 10, 1e-4);
}

BOOST_AUTO_TEST_CASE(distance_box_box_2_fcl)
{
  CollisionGeometryPtr_t s1 (new hpp::fcl::Box (6, 10, 2));
  CollisionGeometryPtr_t s2 (new hpp::fcl::Box (2, 2, 2));
  distance_box_box_2 (s1, s2);
}

#ifdef HPP_FCL_WITH_SCH_CORE
BOOST_AUTO_TEST_CASE(distance_box_box_2_sch)
{
  CollisionGeometryPtr_t s1 (new hpp::fcl::sch::S_Box (6, 10, 2));
  CollisionGeometryPtr_t s2 (new hpp::fcl::sch::S_Box (2, 2, 2));
  distance_box_box_2 (s1, s2);
}
#endif

void distance_box_box_3 (CollisionGeometryPtr_t s1, CollisionGeometryPtr_t s2)
{
  static double pi = M_PI;
  Transform3f tf1 (hpp::fcl::makeQuat (cos (pi/8), 0, 0, sin (pi/8)),
		   Vec3f (-2, 1, .5));
  Transform3f tf2 (hpp::fcl::makeQuat (cos (pi/8), 0, sin(pi/8),0),
		   Vec3f (2, .5, .5));

  CollisionObject o1 (s1, tf1);
  CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest (true, 0, 0, GST_INDEP);
  DistanceResult distanceResult;

  hpp::fcl::distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
	    << " R1 = " << tf1.getRotation () << std::endl
	    << " T2 = " << tf2.getTranslation() << std::endl
	    << " R2 = " << tf2.getRotation () << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;

  const Vec3f& p1 = distanceResult.nearest_points [0];
  const Vec3f& p2 = distanceResult.nearest_points [1];
  double distance = 4 - sqrt (2);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 1e-4);

  const Vec3f p1Ref (sqrt(2)/2 - 2, 1, .5);
  const Vec3f p2Ref (2 - sqrt(2)/2, 1, .5);
  BOOST_CHECK_CLOSE (p1 [0], p1Ref [0], 1e-4);
  BOOST_CHECK_CLOSE (p1 [1], p1Ref [1], 1e-4);
  BOOST_CHECK_CLOSE (p1 [2], p1Ref [2], 1e-4);
  BOOST_CHECK_CLOSE (p2 [0], p2Ref [0], 1e-4);
  BOOST_CHECK_CLOSE (p2 [1], p2Ref [1], 1e-4);
  BOOST_CHECK_CLOSE (p2 [2], p2Ref [2], 1e-4);

  // Apply the same global transform to both objects and recompute
  Transform3f tf3 (hpp::fcl::makeQuat (0.435952844074,-0.718287018243,
				 0.310622451066, 0.444435113443),
		   Vec3f (4, 5, 6));
  tf1 = tf3*tf1;
  tf2 = tf3*tf2;
  o1 = CollisionObject (s1, tf1);
  o2 = CollisionObject (s2, tf2);

  distanceResult.clear ();
  hpp::fcl::distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
	    << " R1 = " << tf1.getRotation () << std::endl
	    << " T2 = " << tf2.getTranslation() << std::endl
	    << " R2 = " << tf2.getRotation () << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;

  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 1e-4);

  const Vec3f p1Moved = tf3.transform (p1Ref);
  const Vec3f p2Moved = tf3.transform (p2Ref);
  BOOST_CHECK_CLOSE (p1 [0], p1Moved [0], 1e-4);
  BOOST_CHECK_CLOSE (p1 [1], p1Moved [1], 1e-4);
  BOOST_CHECK_CLOSE (p1 [2], p1Moved [2], 1e-4);
  BOOST_CHECK_CLOSE (p2 [0], p2Moved [0], 1e-4);
  BOOST_CHECK_CLOSE (p2 [1], p2Moved [1], 1e-4);
  BOOST_CHECK_CLOSE (p2 [2], p2Moved [2], 1e-4);
  
}

BOOST_AUTO_TEST_CASE(distance_box_box_3_fcl)
{
  CollisionGeometryPtr_t s1 (new hpp::fcl::Box (1, 1, 1));
  CollisionGeometryPtr_t s2 (new hpp::fcl::Box (1, 1, 1));
  distance_box_box_3 (s1, s2);
}

#ifdef HPP_FCL_WITH_SCH_CORE
BOOST_AUTO_TEST_CASE(distance_box_box_3_sch)
{
  CollisionGeometryPtr_t s1 (new hpp::fcl::sch::S_Box (1, 1, 1));
  CollisionGeometryPtr_t s2 (new hpp::fcl::sch::S_Box (1, 1, 1));
  distance_box_box_3 (s1, s2);
}
#endif

#ifdef HPP_FCL_WITH_SCH_CORE
// Check that a box as a shape and a box as a mesh gives the same results.
void distance_box_box_4 (CollisionGeometryPtr_t s1, CollisionGeometryPtr_t s2,
    CollisionGeometryPtr_t s3)
{
  Quaternion3f q; q.coeffs().setRandom();

  for (int i = 0; i < 100; ++i) {
    Transform3f tf1 (q.normalized(), Vec3f::Random ()); q.coeffs().setRandom();
    Transform3f tf2 (q.normalized(), Vec3f::Random ()); q.coeffs().setRandom();

    CollisionObject o1 (s1, tf1);
    CollisionObject o2 (s2, tf2);
    CollisionObject o3 (s3, tf2);

    // Enable computation of nearest points
    DistanceRequest distanceRequest (true, 0, 0, GST_INDEP);
    DistanceResult distanceResult1, distanceResult2;

    hpp::fcl::distance (&o1, &o2, distanceRequest, distanceResult1);
    hpp::fcl::distance (&o1, &o3, distanceRequest, distanceResult2);

    BOOST_TEST_MESSAGE("Applied transformations on two boxes"
              << "\nT1 = " << tf1.getTranslation().transpose()
              << "\nR1 = " << tf1.getRotation ()
              << "\nT2 = " << tf2.getTranslation().transpose()
              << "\nR2 = " << tf2.getRotation()
              << "\nClosest points 1:"
              << "\np1 = " << distanceResult1.nearest_points [0].transpose()
              << "\np2 = " << distanceResult1.nearest_points [1].transpose()
              << "\ndistance = " << distanceResult1.min_distance
              << "\nClosest points 2:"
              << "\np1 = " << distanceResult2.nearest_points [0].transpose()
              << "\np2 = " << distanceResult2.nearest_points [1].transpose()
              << "\ndistance = " << distanceResult2.min_distance
              << '\n');

    const Vec3f& a1 = distanceResult1.nearest_points [0];
    const Vec3f& b1 = distanceResult1.nearest_points [1];
    const Vec3f& a2 = distanceResult2.nearest_points [0];
    const Vec3f& b2 = distanceResult2.nearest_points [1];
    BOOST_CHECK_CLOSE(distanceResult1.min_distance, distanceResult2.min_distance, 1e-4);
    BOOST_CHECK_CLOSE (a1 [0], a2 [0], 1e-4);
    BOOST_CHECK_CLOSE (a1 [1], a2 [1], 1e-4);
    BOOST_CHECK_CLOSE (a1 [2], a2 [2], 1e-4);
    BOOST_CHECK_CLOSE (b1 [0], b2 [0], 1e-4);
    BOOST_CHECK_CLOSE (b1 [1], b2 [1], 1e-4);
    BOOST_CHECK_CLOSE (b1 [2], b2 [2], 1e-4);
  }
}

BOOST_AUTO_TEST_CASE(distance_box_box_4_sch)
{
  hpp::fcl::MeshLoader loader;
  CollisionGeometryPtr_t s1 (loader.makeBox (Vec3f(1, 1, 1)));
  CollisionGeometryPtr_t s2 (loader.makeBox (Vec3f(1, 1, 1)));
  CollisionGeometryPtr_t s3 (loader.loadConvex (TEST_RESOURCES_DIR"/box.stl", Vec3f(1,1,1)));

  BOOST_CHECK (boost::dynamic_pointer_cast<hpp::fcl::sch::S_Box>(s1));
  distance_box_box_4 (s1, s2, s3);
}
#endif
