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
#include <ctime>
#include <sys/stat.h>

#include <boost/program_options.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <hpp/fcl/mesh_loader/assimp.h>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/mesh_loader/serialization.h>

using namespace hpp::fcl;
namespace po = boost::program_options;

struct Options
{
  std::string inputFile;
  std::string outputFile;
  bool removeDuplicatedVertices;
  bool verbose;
  double vertexEqualityThr2;
  bool binary;

  Options ()
    : inputFile ()
    , outputFile ()
    , removeDuplicatedVertices (true)
    , verbose (true)
    , vertexEqualityThr2 (1e-12)
    , binary (true)
  {}
};

template <typename BV>
void run (const Options& opts)
{
  typedef BVHModel<BV> BVH_t;
  boost::shared_ptr< BVH_t > model (new BVH_t());

  loadPolyhedronFromResource (opts.inputFile, Vec3f(1,1,1), model);

  // Look for duplicates
  // TODO use the BVH to make this N*log(N) rather that N*N

  if (opts.removeDuplicatedVertices) {
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
      for (std::size_t j = i+1; j < indices.size(); ++j)
        if ((model->vertices[i]-model->vertices[j]).squaredNorm() < opts.vertexEqualityThr2)
          indices[j] = i;
    }

    for (std::size_t i = 0; i < (std::size_t)model->num_tris; ++i) {
      const Triangle& tri (model->tri_indices[i]);
      std::size_t ip[3];
      for (int j = 0; j < 3; ++j) {
        if (indices[tri[j]] > tri[j])
          throw std::logic_error ("This should not happen");
        ip[j] = indicesReduced[indices[tri[j]]];
        if (ip[j] >= (std::size_t)reduced->num_vertices)
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

    if (opts.verbose)
      std::cout << "          Cleaned / Dirty\n"
        << "vertices : " << reduced->num_vertices << " / " << model->num_vertices << '\n'
        << "triangles: " << reduced->num_tris     << " / " << model->num_tris << '\n';

    // TODO add a measure of cache friendliness.
    // - a triangle whose indices are far from each other is bad.
    // - more vertices and triangles is bad.
    // for (std::size_t j = 0; j < (std::size_t)reduced->num_tris; ++j) {
    // const Triangle& tri = reduced->tri_indices[j];
    // std::cout << tri[0] << ' ' << tri[1] << ' ' << tri[2] << '\n';
    // }

    reduced->endModel();
    model = reduced;
  }

  serialize (*model, opts.outputFile.c_str(), opts.binary);
}

int main (int argc, const char* argv[])
{
  // Parse arguments
  Options opts;
  int bvType = BV_OBBRSS;

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message and exit.")
    ("list-bv-types", "list the BV types and exit.")
    ("input,i"                   , po::value<std::string>(&opts.inputFile)->required(), "set the input file")
    ("output,o"                  , po::value<std::string>(&opts.outputFile)->required(), "set the output file")
    ("bv-type"                   , po::value<int        >(&bvType), "set the BV type. See list-bv-types.")
    ("verbose,v"                 , po::value<bool       >(&opts.verbose), "set verbosity.")
    ("remove-duplicated-vertices", po::value<bool       >(&opts.removeDuplicatedVertices), "remove the duplicated vertices.")
    ("vertex-equality-thr2"      , po::value<double     >(&opts.vertexEqualityThr2), "equality threshold (on the squared norm of the distance between two vertices).")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }
  if (vm.count("list-bv-types")) {
    std::cout << "Possible values are: \n"
      << BV_AABB   << ": BV_AABB  \n"
      << BV_OBB    << ": BV_OBB   \n"
      << BV_RSS    << ": BV_RSS   \n"
      // << BV_kIOS   << ": BV_kIOS  \n"
      << BV_OBBRSS << ": BV_OBBRSS\n"
      << BV_KDOP16 << ": BV_KDOP16\n"
      << BV_KDOP18 << ": BV_KDOP18\n"
      << BV_KDOP24 << ": BV_KDOP24\n"
      ;
    return 1;
  }

  if (opts.verbose) {
    std::cout << "Current options are:\n"
      << opts.inputFile << '\n'
      << opts.outputFile << '\n'
      << bvType << '\n'
      << opts.removeDuplicatedVertices << '\n'
      ;
  }

  switch (bvType) {
    case BV_AABB  : run<AABB     > (opts); break;
    case BV_OBB   : run<OBB      > (opts); break;
    case BV_RSS   : run<RSS      > (opts); break;
    // case BV_kIOS  : run<kIOS     > (opts); break;
    case BV_OBBRSS: run<OBBRSS   > (opts); break;
    case BV_KDOP16: run<KDOP<16> > (opts); break;
    case BV_KDOP18: run<KDOP<18> > (opts); break;
    case BV_KDOP24: run<KDOP<24> > (opts); break;
    default: std::cerr << "Invalid bounding volume type. See --list-bv-types.\n"; break;
  }

  return 0;
}
