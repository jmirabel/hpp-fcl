/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2016, CNRS - LAAS
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

#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/mesh_loader/assimp.h>

#include <hpp/fcl/BV/BV.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#ifdef HPP_FCL_WITH_SCH_CORE
# include <hpp/fcl/sch/object.h>
# include <hpp/fcl/sch/assimp.h>
#endif

#ifdef HPP_FCL_WITH_SCH_CORE
  typedef hpp::fcl::sch::S_Box     Box_t;
  typedef hpp::fcl::sch::S_Capsule Capsule_t;
  typedef hpp::fcl::sch::S_Sphere  Sphere_t;
#else
  typedef hpp::fcl::Box     Box_t;
  typedef hpp::fcl::Capsule Capsule_t;
  typedef hpp::fcl::Sphere  Sphere_t;
#endif

namespace hpp
{
namespace fcl {
  bool CachedMeshLoader::Key::operator< (const CachedMeshLoader::Key& b) const
  {
    const CachedMeshLoader::Key& a = *this;
    if (convex == true && b.convex == false) return true;
    if (convex == false && b.convex == true) return false;
    for (int i = 0; i < 3; ++i) {
      if (a.scale[i] < b.scale[i]) return true;
      else if (a.scale[i] > b.scale[i]) return false;
    }
    return std::less<std::string>() (a.filename, b.filename);
  }

  template <typename BV>
  CollisionGeometryPtr_t _load (const std::string& filename, const Vec3f& scale)
  {
    boost::shared_ptr < BVHModel<BV> > polyhedron (new BVHModel<BV>);
    loadPolyhedronFromResource (filename, scale, polyhedron);
    return polyhedron;
  }

  template <typename BV>
  CollisionGeometryPtr_t _loadConvex (const std::string& filename, const Vec3f& scale)
  {
#ifdef HPP_FCL_WITH_SCH_CORE
    boost::shared_ptr < sch::S_Polyhedron<BV> > polyhedron (new sch::S_Polyhedron<BV>);
#else
    boost::shared_ptr < BVHModel<BV> > polyhedron (new BVHModel<BV>);
#endif
    loadPolyhedronFromResource (filename, scale, polyhedron);
    return polyhedron;
  }

  CollisionGeometryPtr_t MeshLoader::load (const std::string& filename,
      const Vec3f& scale)
  {
    switch (bvType_) {
      case BV_AABB  : return _load <AABB  > (filename, scale);
      case BV_OBB   : return _load <OBB   > (filename, scale);
      case BV_RSS   : return _load <RSS   > (filename, scale);
      case BV_kIOS  : return _load <kIOS  > (filename, scale);
      case BV_OBBRSS: return _load <OBBRSS> (filename, scale);
      case BV_KDOP16: return _load <KDOP<16> > (filename, scale);
      case BV_KDOP18: return _load <KDOP<18> > (filename, scale);
      case BV_KDOP24: return _load <KDOP<24> > (filename, scale);
      default:
        throw std::invalid_argument("Unhandled bouding volume type.");
    }
  }

  CollisionGeometryPtr_t MeshLoader::loadConvex (const std::string& filename,
      const Vec3f& scale)
  {
    switch (bvType_) {
      case BV_AABB  : return _loadConvex <AABB  > (filename, scale);
      case BV_OBB   : return _loadConvex <OBB   > (filename, scale);
      case BV_RSS   : return _loadConvex <RSS   > (filename, scale);
      case BV_kIOS  : return _loadConvex <kIOS  > (filename, scale);
      case BV_OBBRSS: return _loadConvex <OBBRSS> (filename, scale);
      case BV_KDOP16: return _loadConvex <KDOP<16> > (filename, scale);
      case BV_KDOP18: return _loadConvex <KDOP<18> > (filename, scale);
      case BV_KDOP24: return _loadConvex <KDOP<24> > (filename, scale);
      default:
        throw std::invalid_argument("Unhandled bouding volume type.");
    }
  }

  CollisionGeometryPtr_t MeshLoader::makeBox (const Vec3f& side)
  {
    return CollisionGeometryPtr_t (new Box_t (side));
  }

  CollisionGeometryPtr_t MeshLoader::makeCapsule (const FCL_REAL& radius, const FCL_REAL& lz)
  {
    return CollisionGeometryPtr_t (new Capsule_t (radius, lz));
  }

  CollisionGeometryPtr_t MeshLoader::makeCone (const FCL_REAL& radius, const FCL_REAL& lz)
  {
    return CollisionGeometryPtr_t (new Cone (radius, lz));
  }

  CollisionGeometryPtr_t MeshLoader::makeCylinder (const FCL_REAL& radius, const FCL_REAL& lz)
  {
    return CollisionGeometryPtr_t (new Cylinder (radius, lz));
  }

  CollisionGeometryPtr_t MeshLoader::makeHalfspace (const Vec3f& normal, const FCL_REAL& d)
  {
    return CollisionGeometryPtr_t (new Halfspace (normal, d));
  }

  CollisionGeometryPtr_t MeshLoader::makePlane (const Vec3f& normal, const FCL_REAL& d)
  {
    return CollisionGeometryPtr_t (new Plane (normal, d));
  }

  CollisionGeometryPtr_t MeshLoader::makeSphere (const FCL_REAL& radius)
  {
    return CollisionGeometryPtr_t (new Sphere_t (radius));
  }

  CollisionGeometryPtr_t MeshLoader::makeTriangleP (const Vec3f& a, const Vec3f& b, const Vec3f& c)
  {
    return CollisionGeometryPtr_t (new TriangleP (a, b, c));
  }

  CollisionGeometryPtr_t CachedMeshLoader::load (const std::string& filename,
      const Vec3f& scale)
  {
    Key key (filename, scale, false);
    Cache_t::const_iterator _cached = cache_.find (key);
    if (_cached == cache_.end()) {
      CollisionGeometryPtr_t geom = MeshLoader::load (filename, scale);
      cache_.insert (std::make_pair(key, geom));
      return geom;
    } else {
      return _cached->second;
    }
  }

  CollisionGeometryPtr_t CachedMeshLoader::loadConvex (const std::string& filename,
      const Vec3f& scale)
  {
    Key key (filename, scale, true);
    Cache_t::const_iterator _cached = cache_.find (key);
    if (_cached == cache_.end()) {
      CollisionGeometryPtr_t geom = MeshLoader::loadConvex (filename, scale);
      cache_.insert (std::make_pair(key, geom));
      return geom;
    } else {
      return _cached->second;
    }
  }
}

} // namespace hpp
