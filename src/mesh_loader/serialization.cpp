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

/** \author Joseph Mirabel */

#include <fstream>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <hpp/fcl/BV/BV_node.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/mesh_loader/serialization.h>

BOOST_SERIALIZATION_SPLIT_FREE(hpp::fcl::BVHModel<hpp::fcl::AABB     >)
BOOST_SERIALIZATION_SPLIT_FREE(hpp::fcl::BVHModel<hpp::fcl::OBB      >)
BOOST_SERIALIZATION_SPLIT_FREE(hpp::fcl::BVHModel<hpp::fcl::RSS      >)
// BOOST_SERIALIZATION_SPLIT_FREE(hpp::fcl::BVHModel<hpp::fcl::kIOS     >)
BOOST_SERIALIZATION_SPLIT_FREE(hpp::fcl::BVHModel<hpp::fcl::OBBRSS   >)
BOOST_SERIALIZATION_SPLIT_FREE(hpp::fcl::BVHModel<hpp::fcl::KDOP<16> >)
BOOST_SERIALIZATION_SPLIT_FREE(hpp::fcl::BVHModel<hpp::fcl::KDOP<18> >)
BOOST_SERIALIZATION_SPLIT_FREE(hpp::fcl::BVHModel<hpp::fcl::KDOP<24> >)

namespace boost
{
namespace serialization
{

using namespace hpp::fcl;

// ---- Matrices and triangle -------------------------------------------------
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(Archive & ar, 
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t, 
    const unsigned int file_version) 
{
  (void) file_version;
  Eigen::Index rows = t.rows(), cols = t.cols();
  if (_Rows == Eigen::Dynamic) ar & rows;
  if (_Cols == Eigen::Dynamic) ar & cols;
  if( rows * cols != t.size() )
    t.resize( rows, cols );

  for(Eigen::Index i=0; i<t.size(); i++)
    ar & t.data()[i];
}

template<class Archive>
inline void serialize(Archive & ar, Triangle& tri, const unsigned int file_version) 
{
  (void) file_version;
  ar & tri[0];
  ar & tri[1];
  ar & tri[2];
}

// ---- Bounding volumes ------------------------------------------------------
template<class Archive>
inline void serialize(Archive & ar, OBB& o, const unsigned int file_version) 
{
  (void) file_version;
  ar & o.axes;
  ar & o.To;
  ar & o.extent;
}
template<class Archive>
inline void serialize(Archive & ar, AABB& o, const unsigned int file_version) 
{
  (void) file_version;
  ar & o.min_;
  ar & o.max_;
}
template<class Archive>
inline void serialize(Archive & ar, RSS& o, const unsigned int file_version) 
{
  (void) file_version;
  ar & o.axes;
  ar & o.Tr;
  ar & o.l;
  ar & o.r;
}
template<class Archive>
inline void serialize(Archive & ar, OBBRSS& o, const unsigned int file_version) 
{
  (void) file_version;
  ar & o.obb;
  ar & o.rss;
}
template<class Archive, size_t N>
inline void serialize(Archive & ar, KDOP<N>& o, const unsigned int file_version) 
{
  (void) file_version;
  for (std::size_t i = 0; i < N; ++i)
    ar & o.dist(i);
}
/*
TODO struct kIOS_Sphere is private.
template<class Archive, size_t N>
inline void serialize(Archive & ar, kIOS<N>& o, const unsigned int file_version) 
{
  ar & o.spheres;
  ar & o.num_spheres;
  ar & o.obb;
}
*/

template<class Archive, typename BV>
inline void serialize(Archive & ar, BVNode<BV>& bv, const unsigned int file_version) 
{
  (void) file_version;
  ar & bv.first_child;
  ar & bv.first_primitive;
  ar & bv.num_primitives;
  ar & bv.bv;
}

// ---- Bounding volumes hierarchy --------------------------------------------
template<class Archive>
inline void serialize(Archive & ar, CollisionGeometry& cg, const unsigned int file_version) 
{
  (void) file_version;
  ar & cg.aabb_center;
  ar & cg.aabb_radius;
  ar & cg.aabb_local;
  //ar & cg.user_data;
  ar & cg.cost_density;
  ar & cg.threshold_occupied;
  ar & cg.threshold_free;
}

template <typename Archive, typename BV>
void save (Archive& ar, const BVHModel<BV>& bvh, const unsigned int file_version)
{
  if (   bvh.build_state != BVH_BUILD_STATE_EMPTY
      && bvh.build_state != BVH_BUILD_STATE_PROCESSED) {
    throw std::logic_error ("Cannot serialize a BVHModel that's not completely built.");
  }
  (void) file_version;
  ar & boost::serialization::base_object<CollisionGeometry>(bvh);
  ar & bvh.build_state;
  ar & bvh.num_vertices;
  ar & bvh.num_tris;
  int num_bvs = bvh.getNumBVs();
  ar & num_bvs;

  for (int i = 0; i < bvh.num_vertices; ++i)
    ar & bvh.vertices[i];
  for (int i = 0; i < bvh.num_tris; ++i)
    ar & bvh.tri_indices[i];
  for (int i = 0; i < bvh.getNumBVs(); ++i)
    ar & bvh.getBV(i);
}

template <typename Archive, typename BV>
void load (Archive& ar, BVHModel<BV>& bvh, const unsigned int file_version)
{
  (void) file_version;
  ar & boost::serialization::base_object<CollisionGeometry>(bvh);
  ar & bvh.build_state;

  int num_vertices;
  Vec3f* vertices;
  int num_tris;
  Triangle* tri_indices;
  int num_bvs;
  BVNode<BV>* bvs;

  ar & num_vertices;
  ar & num_tris;
  ar & num_bvs;

  vertices          = new Vec3f[num_vertices];
  tri_indices       = new Triangle[num_tris];
  bvs               = new BVNode<BV>[num_bvs];
  for (int i = 0; i < num_vertices; ++i)
    ar & vertices[i];
  for (int i = 0; i < num_tris; ++i)
    ar & tri_indices[i];
  for (int i = 0; i < num_bvs; ++i)
    ar & bvs[i];
  bvh.setupBVH (
      num_vertices, vertices,
      num_tris    , tri_indices,
      num_bvs     , bvs);
}

}
}

namespace hpp
{
namespace fcl
{

/// Write a BVHModel to file
/// The file will include vertices, triangles and bounding volumes.
template <typename BV>
void serialize (const BVHModel<BV>& bv, const char* filename, bool binary)
{
  std::ofstream ofs (filename);
  if (!ofs.is_open())
    throw std::logic_error (std::string ("Could not open file ") + filename);
  if (binary) {
    boost::archive::binary_oarchive oa (ofs);
    oa << bv;
  } else {
    boost::archive::text_oarchive oa (ofs);
    oa << bv;
  }
}

/// Read a BVHModel from file
template <typename BV>
void deserialize (const char* filename, BVHModel<BV>& bv, bool binary)
{
  std::ifstream ifs (filename);
  if (!ifs.is_open())
    throw std::logic_error (std::string ("Could not open file ") + filename);
  if (binary) {
    boost::archive::binary_iarchive ia (ifs);
    ia >> bv;
  } else {
    boost::archive::text_iarchive ia (ifs);
    ia >> bv;
  }
}

template void   serialize < AABB     > (const BVHModel< AABB     >& bv, const char* filename, bool binary);
template void   serialize < OBB      > (const BVHModel< OBB      >& bv, const char* filename, bool binary);
template void   serialize < RSS      > (const BVHModel< RSS      >& bv, const char* filename, bool binary);
// template void   serialize < kIOS     > (const BVHModel< kIOS     >& bv, const char* filename, bool binary);
template void   serialize < OBBRSS   > (const BVHModel< OBBRSS   >& bv, const char* filename, bool binary);
template void   serialize < KDOP<16> > (const BVHModel< KDOP<16> >& bv, const char* filename, bool binary);
template void   serialize < KDOP<18> > (const BVHModel< KDOP<18> >& bv, const char* filename, bool binary);
template void   serialize < KDOP<24> > (const BVHModel< KDOP<24> >& bv, const char* filename, bool binary);

template void deserialize < AABB     > (const char* filename, BVHModel< AABB     >& bv, bool binary);
template void deserialize < OBB      > (const char* filename, BVHModel< OBB      >& bv, bool binary);
template void deserialize < RSS      > (const char* filename, BVHModel< RSS      >& bv, bool binary);
// template void deserialize < kIOS     > (const char* filename, BVHModel< kIOS     >& bv, bool binary);
template void deserialize < OBBRSS   > (const char* filename, BVHModel< OBBRSS   >& bv, bool binary);
template void deserialize < KDOP<16> > (const char* filename, BVHModel< KDOP<16> >& bv, bool binary);
template void deserialize < KDOP<18> > (const char* filename, BVHModel< KDOP<18> >& bv, bool binary);
template void deserialize < KDOP<24> > (const char* filename, BVHModel< KDOP<24> >& bv, bool binary);

}
}
