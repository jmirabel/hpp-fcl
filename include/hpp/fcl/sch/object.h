/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, LAAS-CNRS
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

#ifndef HPP_FCL_SCH_OBJECT_H
#define HPP_FCL_SCH_OBJECT_H

#include <sch/S_Object/S_Box.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Capsule.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes.h>

namespace hpp
{
namespace fcl
{
namespace sch
{
typedef boost::shared_ptr< ::sch::S_Object> S_ObjectPtr_t;

template<typename BV>
class S_Polyhedron : public BVHModel <BV>, public ::sch::S_Polyhedron
{
public:
  /// @brief get the type of the object
  OBJECT_TYPE getObjectType() const { return OT_BVH | OT_FCL | OT_SCH; }
};

class S_Box : public Box, public ::sch::S_Box
{
public:
  S_Box(FCL_REAL x, FCL_REAL y, FCL_REAL z)
    : Box(x, y, z)
    , ::sch::S_Box (x, y, z)
  {}

  S_Box(const Vec3f& side)
    : Box(side)
    , ::sch::S_Box (side[0], side[1], side[2])
  {}

  /// @brief get the type of the object
  OBJECT_TYPE getObjectType() const { return (OBJECT_TYPE) (OT_GEOM | OT_FCL | OT_SCH); }
};

class S_Sphere : public Sphere, public ::sch::S_Sphere
{
public:
  S_Sphere(FCL_REAL radius)
    : Sphere(radius)
    , ::sch::S_Sphere (radius)
  {}

  /// @brief get the type of the object
  OBJECT_TYPE getObjectType() const { return (OBJECT_TYPE) (OT_GEOM | OT_FCL | OT_SCH); }
};

class S_Capsule : public Capsule, public ::sch::S_Capsule
{
public:
  S_Capsule(FCL_REAL radius, FCL_REAL lz)
    : Capsule(radius, lz)
    , ::sch::S_Capsule (::sch::Point3(0,0,-lz/2), ::sch::Point3(0,0,lz/2), radius)
  {}

  /// @brief get the type of the object
  OBJECT_TYPE getObjectType() const { return (OBJECT_TYPE) (OT_GEOM | OT_FCL | OT_SCH); }
};

} // namespace sch
} // namespace fcl
} // namespace hpp

#endif // HPP_FCL_SCH_OBJECT_H
