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

#include <hpp/fcl/sch/assimp.h>

#ifdef HPP_FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES
  #include <assimp/DefaultLogger.hpp>
  #include <assimp/IOStream.hpp>
  #include <assimp/IOSystem.hpp>
  #include <assimp/Importer.hpp>
  #include <assimp/postprocess.h>
  #include <assimp/scene.h>
#else
  #include <assimp/DefaultLogger.h>
  #include <assimp/assimp.hpp>
  #include <assimp/IOStream.h>
  #include <assimp/IOSystem.h>
  #include <assimp/aiPostProcess.h>
  #include <assimp/aiScene.h>
#endif

namespace hpp
{
namespace fcl
{
  
namespace internal
{

void polyhedronFromAssimpScene(const fcl::Vec3f & scale,
                               const aiScene* scene,
                               ::sch::S_Polyhedron* polyhedron)
{
  ::sch::Polyhedron_algorithms& algo = *polyhedron->getPolyhedronAlgorithm();
  algo.clear();

  // Check that there is no children. Trees are not supported yet.
  aiNode *node = scene->mRootNode;
  if (!node) return;
  assert (node->mParent == NULL);
  aiMatrix4x4 transform = node->mTransformation;

  size_t nr_vertexes = 0;
  size_t nr_faces = 0;
  for (uint32_t i = 0; i < node->mNumMeshes; i++)
  {
    const aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
    nr_vertexes += input_mesh->mNumVertices;
    nr_faces += input_mesh->mNumFaces;
  }
  algo.vertexes_.reserve(nr_vertexes);
  algo.triangles_.reserve(nr_faces);

  for (uint32_t i = 0; i < node->mNumMeshes; i++)
  {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
    
    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
    {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;

      ::sch::S_PolyhedronVertex *v = new ::sch::S_PolyhedronVertex();
      v->setCoordinates(p.x * scale[0], p.y * scale[1], p.z * scale[2]);
      v->setNumber(unsigned (algo.vertexes_.size()));
      algo.vertexes_.push_back(v);
    }
  }

  for (uint32_t i = 0; i < node->mNumMeshes; i++)
  {
    const aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
    
    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
    {
      const aiFace& face = input_mesh->mFaces[j];

      if (face.mNumIndices != 3) {
        std::stringstream ss;
#ifdef HPP_FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES
        ss << "Mesh " << input_mesh->mName.C_Str() << " has a face with "
           << face.mNumIndices << " vertices. This is not supported\n";
        ss << "Node name is: " << node->mName.C_Str() << "\n";
#endif
        ss << "Mesh index: " << i << "\n";
        ss << "Face index: " << j << "\n";
        throw std::invalid_argument (ss.str());
      }

      ::sch::PolyhedronTriangle t;

      t.a = face.mIndices[0];
      t.b = face.mIndices[1];
      t.c = face.mIndices[2];

      algo.vertexes_[t.a]->addNeighbor(algo.vertexes_[t.b]);
      algo.vertexes_[t.a]->addNeighbor(algo.vertexes_[t.c]);

      algo.vertexes_[t.b]->addNeighbor(algo.vertexes_[t.a]);
      algo.vertexes_[t.b]->addNeighbor(algo.vertexes_[t.c]);

      algo.vertexes_[t.c]->addNeighbor(algo.vertexes_[t.a]);
      algo.vertexes_[t.c]->addNeighbor(algo.vertexes_[t.b]);

      ::sch::Vector3 ab (algo.vertexes_[t.b]->getCoordinates() - algo.vertexes_[t.a]->getCoordinates()),
                     ac (algo.vertexes_[t.c]->getCoordinates() - algo.vertexes_[t.a]->getCoordinates());
      t.normal = ab ^ ac;
      FCL_REAL norm = t.normal.normsquared();
      if (norm < 1e-8) {
        std::stringstream ss;
        ss << "Mesh "
#ifdef HPP_FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES
          << input_mesh->mName.C_Str()
#endif
          << " has a flat triangle (||AB^AC||^2 = " << norm << ")."
          " This is not supported\n";
#ifdef HPP_FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES
        ss << "Node name is: " << node->mName.C_Str() << "\n";
#endif
        ss << "Mesh index: " << i << "\n";
        ss << "Face index: " << j << "\n";
        throw std::invalid_argument (ss.str());
      }

      t.normal *= 1 / std::sqrt (norm);

      algo.triangles_.push_back(t);
    }
  }

  for (unsigned int i=0; i<algo.vertexes_.size(); i++)
  {
    algo.vertexes_[i]->updateFastArrays();
  }

  algo.deleteVertexesWithoutNeighbors();
}

} // namespace internal
} // namespace fcl
} // namespace hpp
