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


#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_func_matrix.h>
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
CollisionFunctionMatrix<GJKSolver>& getCollisionFunctionLookTable()
{
  static CollisionFunctionMatrix<GJKSolver> table;
  return table;
}

template<typename NarrowPhaseSolver>
std::size_t collide(const CollisionObject* o1, const CollisionObject* o2,
                    const NarrowPhaseSolver* nsolver,
                    const CollisionRequest& request,
                    CollisionResult& result)
{
  return collide(o1->collisionGeometry().get(), o1->getTransform(), o2->collisionGeometry().get(), o2->getTransform(),
                 nsolver, request, result);
}

// reorder collision results in the order the call has been made.
void invertResults(CollisionResult& result)
{
    const CollisionGeometry* otmp;
    int btmp;
    for(std::vector<Contact>::iterator it = result.contacts.begin();
        it != result.contacts.end(); ++it)
    {
        otmp = it->o1;
        it->o1 = it->o2;
        it->o2 = otmp;
        btmp = it->b1;
        it->b1 = it->b2;
        it->b2 = btmp;
    }
}

#ifdef HPP_FCL_WITH_SCH_CORE

std::size_t sch_collide(
                    const CollisionGeometry* _o1, const Transform3f& tf1,
                    const CollisionGeometry* _o2, const Transform3f& tf2,
                    const CollisionRequest& request,
                    CollisionResult& result)
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
  bool collision = pair.isInCollision();
  if (collision && request.enable_contact) {
    Contact contact (_o1, _o2, Contact::NONE, Contact::NONE);
    // The bsd version does not include the EPA algorithm (because it is GPL)
    // so there is no contact point
#ifndef SCH_BUILD_BSD
    ::sch::Point3 p1, p2, u;
    contact.penetration_depth = - std::sqrt (-pair.getClosestPoints (p1, p2));
    u = p2 - p1;
    contact.pos << p1[0], p1[1], p1[2];
    contact.normal << u[0], u[1], u[2];
#endif
    result.addContact (contact);
    result.distance_lower_bound = 0;
  } else {
    if (request.enable_distance_lower_bound)
      result.distance_lower_bound = pair.getDistanceWithoutPenetrationDepth();
  }
  return collision ? 1 : 0;
}
#endif

template<typename NarrowPhaseSolver>
std::size_t collide(const CollisionGeometry* o1, const Transform3f& tf1,
                    const CollisionGeometry* o2, const Transform3f& tf2,
                    const NarrowPhaseSolver* nsolver_,
                    const CollisionRequest& request,
                    CollisionResult& result)
{
  const NarrowPhaseSolver* nsolver = nsolver_;
  if(!nsolver_)
    nsolver = new NarrowPhaseSolver();  

  const CollisionFunctionMatrix<NarrowPhaseSolver>& looktable = getCollisionFunctionLookTable<NarrowPhaseSolver>();
  result.distance_lower_bound = -1;
  std::size_t res; 
  if(request.num_max_contacts == 0)
  {
    std::cerr << "Warning: should stop early as num_max_contact is " << request.num_max_contacts << " !" << std::endl;
    res = 0;
  }
  else
  {
    int object_type1 = o1->getObjectType() | OT_MASK;
    int object_type2 = o2->getObjectType() | OT_MASK;
    int node_type1 = o1->getNodeType();
    int node_type2 = o2->getNodeType();
  
#ifdef HPP_FCL_WITH_SCH_CORE
    bool has_sch1 = o1->getObjectType() & OT_SCH;
    bool has_sch2 = o2->getObjectType() & OT_SCH;
    if (has_sch1 && has_sch2 && (object_type1 != OT_GEOM || object_type2 != OT_GEOM))
    {
      res = sch_collide (o1, tf1, o2, tf2, request, result);
    }
    else
#endif
      if(object_type1 == OT_GEOM && object_type2 == OT_BVH)
    {  
      if(!looktable.collision_matrix[node_type2][node_type1])
      {
        std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
        res = 0;
      }
      else
      {
        res = looktable.collision_matrix[node_type2][node_type1](o2, tf2, o1, tf1, nsolver, request, result);
        invertResults(result);
      }
    }
    else
    {
      if(!looktable.collision_matrix[node_type1][node_type2])
      {
        std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
        res = 0;
      }
      else
        res = looktable.collision_matrix[node_type1][node_type2](o1, tf1, o2, tf2, nsolver, request, result);
    }
  }

  if(!nsolver_)
    delete nsolver;
  
  return res;
}

std::size_t collide(const CollisionObject* o1, const CollisionObject* o2,
                    const CollisionRequest& request, CollisionResult& result)
{
  switch(request.gjk_solver_type)
  {
  case GST_INDEP:
    {
      GJKSolver_indep solver;
      return collide<GJKSolver_indep>(o1, o2, &solver, request, result);
    }
  default:
    return -1; // error
  }
}

std::size_t collide(const CollisionGeometry* o1, const Transform3f& tf1,
                    const CollisionGeometry* o2, const Transform3f& tf2,
                    const CollisionRequest& request, CollisionResult& result)
{
  switch(request.gjk_solver_type)
  {
  case GST_INDEP:
    {
      GJKSolver_indep solver;
      return collide<GJKSolver_indep>(o1, tf1, o2, tf2, &solver, request, result);
    }
  default:
    std::cerr << "Warning! Invalid GJK solver" << std::endl;
    return -1; // error
  }
}

}


} // namespace hpp
