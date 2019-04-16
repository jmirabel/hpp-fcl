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

#define BOOST_TEST_MODULE HPP_FCL_SERIALIZATION
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <fstream>
#include <ctime>
#include <sys/stat.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <hpp/fcl/mesh_loader/assimp.h>

#include <hpp/fcl/BV/BV_node.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/mesh_loader/serialization.h>

#include "fcl_resources/config.h"

namespace hpp
{
namespace fcl
{

bool operator== (const Triangle& a, const Triangle& b)
{
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

bool operator== (const OBB& a, const OBB& b)
{
  return a.axes   == b.axes
    &&   a.To     == b.To
    &&   a.extent == b.extent;
}

bool operator== (const RSS& a, const RSS& b)
{
  return a.axes == b.axes
    &&   a.Tr   == b.Tr
    &&   a.l[0] == b.l[0]
    &&   a.l[1] == b.l[1]
    &&   a.r    == b.r;
}

bool operator== (const OBBRSS& a, const OBBRSS& b)
{
  return a.obb == b.obb && a.rss == b.rss;
}

template <typename BV>
bool operator== (const BVNode<BV>& a, const BVNode<BV>& b)
{
  return a.first_child     == b.first_child
    &&   a.first_primitive == b.first_primitive
    &&   a.num_primitives  == b.num_primitives
    &&   a.bv == b.bv;
}

}
}

using namespace hpp::fcl;

/*
BOOST_SERIALIZATION_SPLIT_FREE(BVHModel<OBB>)

namespace boost {
  namespace serialization {

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

    // ---- Bounding volumes --------------------------------------------------
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
      ar & o.dist_;
    }

    template<class Archive, typename BV>
    inline void serialize(Archive & ar, BVNode<BV>& bv, const unsigned int file_version) 
    {
      (void) file_version;
      ar & bv.first_child;
      ar & bv.first_primitive;
      ar & bv.num_primitives;
      ar & bv.bv;
    }

    // ---- Bounding volumes hierarchy ----------------------------------------
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

template <typename Matrix>
void checkEigen ()
{
  Matrix v (Matrix::Random());
  {
    std::ofstream ofs ("/tmp/foo");
    boost::archive::text_oarchive oa (ofs);
    oa << v;
  }
  Matrix vv;
  {
    std::ifstream ifs ("/tmp/foo");
    boost::archive::text_iarchive ia (ifs);
    ia >> vv;
  }
  BOOST_CHECK (v == vv);
}

template <typename BV>
void checkBV (BV& bv)
{
  BVNode<BV> a, b;
  a.bv = bv;
  {
    std::ofstream ofs ("/tmp/foo");
    boost::archive::binary_oarchive oa (ofs);
    oa << a;
  }

  {
    std::ifstream ifs ("/tmp/foo");
    boost::archive::binary_iarchive ia (ifs);
    ia >> b;
  }

  BOOST_CHECK (a == b);
}

*/

template <typename BV>
void checkBVH ()
{
  std::string filename (TEST_RESOURCES_DIR "/env.obj");
  const char* bvhfile = TEST_RESOURCES_DIR "/env.obj.fcl";
  const bool binary = true;

  typedef BVHModel<BV> BVH_t;
  boost::shared_ptr< BVH_t > modelA (new BVH_t());
  loadPolyhedronFromResource(filename, Vec3f(1,1,1), modelA);
  serialize (*modelA, bvhfile, binary);

  boost::shared_ptr< BVH_t > modelB (new BVH_t());
  deserialize (bvhfile, *modelB, binary);

  BOOST_CHECK_EQUAL (modelA->num_vertices, modelB->num_vertices);
  BOOST_CHECK_EQUAL (modelA->num_tris    , modelB->num_tris    );
  BOOST_CHECK_EQUAL (modelA->getNumBVs() , modelB->getNumBVs() );

  for (int i = 0; i < modelA->num_vertices; ++i)
    BOOST_CHECK_EQUAL (modelA->vertices[i], modelB->vertices[i]);
  for (int i = 0; i < modelA->num_tris; ++i)
    BOOST_CHECK (modelA->tri_indices[i] == modelB->tri_indices[i]);
  for (int i = 0; i < modelA->getNumBVs(); ++i)
    BOOST_CHECK (modelA->getBV(i) == modelB->getBV(i));
}

bool exists (const char* f)
{
  struct stat buf;
  return (stat(f, &buf) == 0);
}

template <typename BV>
void runFoo ()
{
  //std::string filename ("/local/jmirabel/devel/hpp/encrypted/share/airbus_description/meshes/ATR72/XTCT-ATR72_AllCATPart_scaled.stl");
  //const char* bvhfile = "/tmp/obj.fcl";
  std::string filename (TEST_RESOURCES_DIR "/env.obj");
  const char* bvhfile = "/tmp/env.obj.fcl";
  const bool binary = true;

  double vertex_equal_thr2 = 1e-12;

  typedef BVHModel<BV> BVH_t;
  boost::shared_ptr< BVH_t > model (new BVH_t());

  clock_t t0, t1;
  double d01;

  bool bvhexists = exists (bvhfile);

  t0 = clock();
  if (bvhexists) deserialize (bvhfile, *model, binary);
  else           loadPolyhedronFromResource(filename, Vec3f(1,1,1), model);
  t1 = clock();
  d01 = double(t1-t0) / CLOCKS_PER_SEC;
  std::cout << "Time to load " << d01 << std::endl;

  if (!bvhexists) {
    t0 = clock();
    serialize (*model, bvhfile, binary);
    t1 = clock();
    d01 = double(t1-t0) / CLOCKS_PER_SEC;
    std::cout << "Time to save " << d01 << std::endl;
  }

  // Look for duplicates
  // TODO use the BVH to make this N*log(N) rather that N*N

  boost::shared_ptr< BVH_t > reduced (new BVH_t());
  reduced->beginModel (model->num_vertices, model->num_tris);

  std::vector <std::size_t> indices (model->num_vertices);
  std::vector <std::size_t> indicesReduced (model->num_vertices, model->num_vertices);

  for (std::size_t i = 0; i < indices.size(); ++i) indices[i] = i;
  for (std::size_t i = 0; i < indices.size(); ++i) {
    if (indices[i] > i)
      throw std::logic_error ("This should not happen");
    // i is already a duplicated vertices
    if (indices[i] != i) continue;

    // Add vertex
    indicesReduced[i] = reduced->num_vertices;
    reduced->addVertex (model->vertices[i]);

    // Check for duplicates with following vertices.
    for (std::size_t j = i+1; j < indices.size(); ++j) {
      if ((model->vertices[i]-model->vertices[j]).squaredNorm() < vertex_equal_thr2) {
        indices[j] = i;
        //std::cout << "Equal vertex " << i << ' ' << j << '\n';
      }
    }
  }

  for (std::size_t i = 0; i < (std::size_t)model->num_tris; ++i) {
    const Triangle& tri (model->tri_indices[i]);
    std::size_t ip[3];
    for (std::size_t j = 0; j < 3; ++j) {
      if (indices[tri[j]] > tri[j])
        throw std::logic_error ("This should not happen");
      ip[j] = indicesReduced[indices[tri[j]]];
      if (ip[j] >= reduced->num_vertices)
        throw std::logic_error ("This should not happen");
    }

    // Check if triangle already exists
    bool triangleIsDuplicated = false;
    for (std::size_t j = 0; j < (std::size_t)reduced->num_tris; ++j) {
      const Triangle& rtri = reduced->tri_indices[j];
      triangleIsDuplicated =
        (rtri[0] == ip[0] && rtri[1] == ip[1] && rtri[2] == ip[2]);
      if (triangleIsDuplicated) break;
    }

    if (!triangleIsDuplicated)
      reduced->addTriangle (ip[0], ip[1], ip[2]);
  }

  std::cout << "          Cleaned / Dirty\n"
            << "vertices : " << reduced->num_vertices << " / " << model->num_vertices << '\n'
            << "triangles: " << reduced->num_tris     << " / " << model->num_tris << '\n';

  for (std::size_t j = 0; j < (std::size_t)reduced->num_tris; ++j) {
    const Triangle& tri = reduced->tri_indices[j];
    std::cout << tri[0] << ' ' << tri[1] << ' ' << tri[2] << '\n';
  }
}

BOOST_AUTO_TEST_SUITE(serialization)

/*
BOOST_AUTO_TEST_CASE(eigen)
{
  checkEigen <Vec3f> ();
  checkEigen <Matrix3f> ();
}

BOOST_AUTO_TEST_CASE(triangle)
{
  Triangle a (1, 1, 2);
  {
    std::ofstream ofs ("/tmp/foo");
    boost::archive::binary_oarchive oa (ofs);
    oa << a;
  }
  Triangle b;
  {
    std::ifstream ifs ("/tmp/foo");
    boost::archive::binary_iarchive ia (ifs);
    ia >> b;
  }
  BOOST_CHECK (a == b);
}

BOOST_AUTO_TEST_CASE(boundingVolumes)
{
  OBB obb;
  obb.axes = Eigen::AngleAxisd(1., Eigen::Vector3d::Random().normalized()).matrix();
  obb.To.setRandom();
  obb.extent.setRandom();
  checkBV (obb);
}
*/

BOOST_AUTO_TEST_CASE(bvhmodel)
{
  checkBVH<OBB   >();
  checkBVH<OBBRSS>();
}

BOOST_AUTO_TEST_CASE(foo)
{
  runFoo <OBB> ();
}

BOOST_AUTO_TEST_SUITE_END()
