/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, CNRS - LAAS
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

/* \author Joseph Mirabel */

#ifndef HPP_FCL_SCH_ASSIMP_H
#define HPP_FCL_SCH_ASSIMP_H

#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/sch/object.h>

namespace hpp
{
namespace fcl
{
  
namespace internal
{
/**
 * @brief      Convert an assimp scene to a ::sch::S_Polyhedron
 *
 * @param[in]  scale       Scale to apply when reading the ressource
 * @param[in]  scene       Pointer to the assimp scene
 * @param[out] polyhedron  The mesh that must be built
 */
void polyhedronFromAssimpScene(const fcl::Vec3f & scale,
                               const aiScene* scene,
                               ::sch::S_Polyhedron* polyhedron);
} // namespace internal

/**
 * @brief      Template specialization to build both BVHModel and SCH S_Polyhedron.
 *
 * @param[in]  resource_path  Path to the ressource mesh file to be read
 * @param[in]  scale          Scale to apply when reading the ressource
 * @param[out] polyhedron     The resulted polyhedron
 */
template<class BoundingVolume>
inline void loadPolyhedronFromResource (const std::string & resource_path,
                                        const fcl::Vec3f & scale,
                                        const boost::shared_ptr < sch::S_Polyhedron<BoundingVolume> > & polyhedron)
{
  internal::Loader scene;
  scene.load (resource_path);

  internal::meshFromAssimpScene (scale, scene.scene, polyhedron);
  internal::polyhedronFromAssimpScene (scale, scene.scene, polyhedron.get());
}

} // namespace fcl

} // namespace hpp

#endif // HPP_FCL_SCH_ASSIMP_H
