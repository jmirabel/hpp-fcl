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

/** \author Jia Pan */

#include <hpp/fcl/distance.h>
#include <hpp/fcl/distance_func_matrix.h>
#include <hpp/fcl/narrowphase/narrowphase.h>

#ifdef HPP_FCL_WITH_SCH_CORE
# include <sch/CD/CD_Pair.h>
# include <sch/S_Object/S_Object.h>
#endif

#include <iostream>

namespace hpp
{
namespace fcl
{

template<typename GJKSolver>
DistanceFunctionMatrix<GJKSolver>& getDistanceFunctionLookTable()
{
  static DistanceFunctionMatrix<GJKSolver> table;
  return table;
}

#ifdef HPP_FCL_WITH_SCH_CORE

FCL_REAL sch_distance(
                    const CollisionGeometry* _o1, const Transform3f& tf1,
                    const CollisionGeometry* _o2, const Transform3f& tf2,
                    const DistanceRequest& request,
                    DistanceResult& result)
{
  typedef ::sch::S_Object S_Object;
  S_Object* o1 = dynamic_cast< S_Object*> (const_cast<CollisionGeometry*>(_o1));
  S_Object* o2 = dynamic_cast< S_Object*> (const_cast<CollisionGeometry*>(_o2));

  const Vec3f& t1 (tf1.getTranslation()),
               t2 (tf2.getTranslation());
  const Quaternion3f& q1 (tf1.getQuatRotation()),
                      q2 (tf2.getQuatRotation());

  o1->setPosition (t1[0], t1[1], t1[2]);
  o1->setOrientation (q1.x(), q1.y(), q1.z(), q1.w());

  o2->setPosition (t2[0], t2[1], t2[2]);
  o2->setOrientation (q2.x(), q2.y(), q2.z(), q2.w());

  ::sch::CD_Pair pair (o1, o2);
  pair.setRelativePrecision(1e-4);
  // The bsd version of SCH does not include the EPA algorithm (because it is GPL)
  FCL_REAL squared_dist = pair.getDistance();
  result.o1 = _o1;
  result.o2 = _o2;
  if (squared_dist > 0) {
    result.min_distance = std::sqrt(squared_dist);
  } else {
    result.min_distance = - std::sqrt(- squared_dist);
  }
  if (request.enable_nearest_points) {
#ifdef SCH_BUILD_BSD
    if (squared_dist == 0) {
      //std::cerr << "penetration points are not computed with SCH with GPL license." << std::endl;
      result.nearest_points[0].setConstant (std::numeric_limits<FCL_REAL>::quiet_nan());
      result.nearest_points[1].setConstant (std::numeric_limits<FCL_REAL>::quiet_nan());
      result.normal.setConstant (std::numeric_limits<FCL_REAL>::quiet_nan());
    } else
#endif
    {
      ::sch::Point3 p1, p2, u;
      pair.getClosestPoints (p1, p2);
      u = p2 - p1;

      result.nearest_points[0] << p1[0], p1[1], p1[2];
      result.nearest_points[1] << p2[0], p2[1], p2[2];
      result.normal << u[0], u[1], u[2];
    }
  }
  return result.min_distance;
}
#endif


template<typename NarrowPhaseSolver>
FCL_REAL distance(const CollisionObject* o1, const CollisionObject* o2, const NarrowPhaseSolver* nsolver,
                  const DistanceRequest& request, DistanceResult& result)
{
  return distance<NarrowPhaseSolver>(o1->collisionGeometry().get(), o1->getTransform(), o2->collisionGeometry().get(), o2->getTransform(), nsolver,
                                     request, result);
}

template<typename NarrowPhaseSolver>
FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1, 
                  const CollisionGeometry* o2, const Transform3f& tf2,
                  const NarrowPhaseSolver* nsolver_,
                  const DistanceRequest& request, DistanceResult& result)
{
  const NarrowPhaseSolver* nsolver = nsolver_;
  if(!nsolver_) 
    nsolver = new NarrowPhaseSolver();

  const DistanceFunctionMatrix<NarrowPhaseSolver>& looktable = getDistanceFunctionLookTable<NarrowPhaseSolver>();

  int object_type1 = o1->getObjectType() | OT_MASK;
  int object_type2 = o2->getObjectType() | OT_MASK;
  int node_type1 = o1->getNodeType();
  int node_type2 = o2->getNodeType();

  FCL_REAL res = std::numeric_limits<FCL_REAL>::max();

#ifdef HPP_FCL_WITH_SCH_CORE
  bool has_sch1 = o1->getObjectType() & OT_SCH;
  bool has_sch2 = o2->getObjectType() & OT_SCH;
  if (has_sch1 && has_sch2
      && ( object_type1 != OT_GEOM
        || object_type2 != OT_GEOM
        || (node_type1 == GEOM_BOX && node_type2 != GEOM_SPHERE)
        || (node_type1 != GEOM_SPHERE && node_type2 == GEOM_BOX)
        )
      )
  {
    res = sch_distance (o1, tf1, o2, tf2, request, result);
  }
  else
#endif
  if(object_type1 == OT_GEOM && object_type2 == OT_BVH)
  {
    if(!looktable.distance_matrix[node_type2][node_type1])
    {
      std::cerr << "Warning: distance function between node type " << node_type1 << " and node type " << node_type2 << " is not supported" << std::endl;
    }
    else
    {
      res = looktable.distance_matrix[node_type2][node_type1](o2, tf2, o1, tf1, nsolver, request, result);
      // If closest points are requested, switch object 1 and 2
      if (request.enable_nearest_points) {
	const CollisionGeometry *tmpo = result.o1;
	result.o1 = result.o2;
	result.o2 = tmpo;
	Vec3f tmpn (result.nearest_points [0]);
	result.nearest_points [0] = result.nearest_points [1];
	result.nearest_points [1] = tmpn;
      }
    }
  }
  else
  {
    if(!looktable.distance_matrix[node_type1][node_type2])
    {
      std::cerr << "Warning: distance function between node type " << node_type1 << " and node type " << node_type2 << " is not supported" << std::endl;
    }
    else
    {
      res = looktable.distance_matrix[node_type1][node_type2](o1, tf1, o2, tf2, nsolver, request, result);    
    }
  }

  if(!nsolver_)
    delete nsolver;

  return res;
}


FCL_REAL distance(const CollisionObject* o1, const CollisionObject* o2, const DistanceRequest& request, DistanceResult& result)
{
  switch(request.gjk_solver_type)
  {
  case GST_INDEP:
    {
      GJKSolver_indep solver;
      return distance<GJKSolver_indep>(o1, o2, &solver, request, result);
    }
  default:
    return -1; // error
  }
}

FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1,
                  const CollisionGeometry* o2, const Transform3f& tf2,
                  const DistanceRequest& request, DistanceResult& result)
{
  switch(request.gjk_solver_type)
  {
  case GST_INDEP:
    {
      GJKSolver_indep solver;
      return distance<GJKSolver_indep>(o1, tf1, o2, tf2, &solver, request, result);
    }
  default:
    return -1;
  }
}


}

} // namespace hpp
