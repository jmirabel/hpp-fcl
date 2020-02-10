/*
 * Software License Agreement (BSD License)
 */

/** \author Joseph Mirabel */

#define BOOST_TEST_MODULE HPP_FCL_BOUNDING_VOLUMES
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <chrono>
#include <fstream>

#include <hpp/fcl/data_types.h>
#include <hpp/fcl/BV/OBB.h>

#include "../src/aligned-math.h"
#include "../src/BV/OBB.h"

// Some include to compute the true distance.
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/narrowphase/narrowphase.h>

using namespace hpp::fcl;

Eigen::IOFormat benchFormat = Eigen::IOFormat (Eigen::StreamPrecision,
    Eigen::DontAlignCols, " ", " ", "", "", "", "" );
std::ofstream* bench = NULL;


BOOST_AUTO_TEST_SUITE( obb )

FCL_REAL computeDistance (const Matrix3f& B, const Vec3f& T,
                          const Vec3f& a, const Vec3f& b)
{
  Box boxa, boxb;
  boxa.halfSide = a;
  boxb.halfSide = b;

  Transform3f tf (B, T);

  GJKSolver gjk;
  Vec3f pa, pb, n;
  FCL_REAL dist = std::numeric_limits<FCL_REAL>::quiet_NaN();
  gjk.shapeDistance(boxa, Transform3f(), boxb, tf, dist, pa, pb, n);
  return dist;
}

BOOST_AUTO_TEST_CASE(disjoint_consistency)
{
  Vec3f T;
  Vec3f a;
  Vec3f b;
  Matrix3f B;
  CollisionRequest request;

  for (int i = 0; i < 1000; ++i) {
    // Compute random OBB
    B = Quaternion3f::UnitRandom().matrix();
    T.setRandom();

    a.setRandom(); a = a.cwiseAbs();
    b.setRandom(); b = b.cwiseAbs();

    // Check for collision
    FCL_REAL distLB;
    bool res = obbDisjointAndLowerBoundDistance (B, T, a, b, request, distLB);

    // Compute distance
    FCL_REAL dist = computeDistance(B, T, a, b);

    // Check the results
    BOOST_CHECK_EQUAL(dist >= request.break_distance, res);
    BOOST_CHECK_MESSAGE(dist < request.break_distance || dist * dist >= distLB - 1e-10,
        "distance and distance lower bound between OBB does not coincide.\n"
        << "lower bound: " << std::sqrt(distLB) << '\n'
        << "distance: " << dist);
  }
}

#ifdef HPP_FCL_VECTORIZE_OBB_DISJOINT

template <int N>
void check_aligned_disjoint()
{
  typename AlignedMatrix3f<N>::AlignedVec3f T;
  typename AlignedMatrix3f<N>::AlignedVec3f a;
  typename AlignedMatrix3f<N>::AlignedVec3f b;
  AlignedMatrix3f<N> B;
  CollisionRequest request;

  for (int i = 0; i < 1000; ++i) {
    // Compute random OBB
    for (int j = 0; j < N; ++j)
      B.matrix(j) = Quaternion3f::UnitRandom().matrix();
    T.setRandom();

    a.setRandom(); a = a.cwiseAbs();
    b.setRandom(); b = b.cwiseAbs();

    Eigen::Array<FCL_REAL, N, 1> aligned_distLB2, current_distLB2;
    Eigen::Array<bool, N, 1> oks;

    obbDisjointAndLowerBoundDistance<N, false> (B, T, a, b, request, aligned_distLB2, oks);

    for (int j = 0; j < N; ++j) {
      FCL_REAL distLB;
      Matrix3f BB (B.matrix(j));
      Vec3f TT (T.row(j)), aa (a.row(j)), bb (b.row(j));

      bool res = obbDisjointAndLowerBoundDistance (BB, TT, aa, bb, request, distLB);
      current_distLB2[j] = res ? distLB : 0;

      FCL_REAL dist = computeDistance(BB, TT, aa, bb);

      BOOST_CHECK_EQUAL(oks[j], res);
      BOOST_CHECK_MESSAGE(dist < 0 || dist * dist >= aligned_distLB2[j] - 1e-10,
          "distance between OBB pair " << j << " does not coincide.\n"
          << "aligned: " << std::sqrt(aligned_distLB2[j]) << '\n'
          << "current: " << std::sqrt(distLB) << '\n'
          << "distance: " << dist);
      BOOST_CHECK_MESSAGE(aligned_distLB2[j] >= distLB - 1e-10,
          "distance lower bound between OBB pair " << j << " does not coincide.\n"
          << "aligned: " << std::sqrt(aligned_distLB2[j]) << '\n'
          << "current: " << std::sqrt(distLB) << '\n');
    }

  }
}

BOOST_AUTO_TEST_CASE(aligned_disjoint_consistency)
{
  check_aligned_disjoint<HPP_FCL_VECTORIZE_OBB_DISJOINT>();
}

template <int N>
void benchmark_aligned_disjoint()
{
  typename AlignedMatrix3f<N>::AlignedVec3f T;
  typename AlignedMatrix3f<N>::AlignedVec3f a;
  typename AlignedMatrix3f<N>::AlignedVec3f b;
  AlignedMatrix3f<N> B;
  CollisionRequest request;

  typedef std::chrono::high_resolution_clock clock_type;
  typedef clock_type::duration duration_type;
  typedef clock_type::time_point time_type;

  for (int i = 0; i < 1000; ++i) {
    // Compute random OBB
    for (int j = 0; j < N; ++j)
      B.matrix(j) = Quaternion3f::UnitRandom().matrix();
    T.setRandom();

    a.setRandom(); a = a.cwiseAbs();
    b.setRandom(); b = b.cwiseAbs();

    Eigen::Array<FCL_REAL, N, 1> aligned_distLB2, current_distLB2;
    Eigen::Array<bool, N, 1> oks;

    time_type t0 = clock_type::now();
    obbDisjointAndLowerBoundDistance<N, false> (B, T, a, b,
        request, aligned_distLB2, oks);
    time_type t1 = clock_type::now();
    duration_type t_aligned = t1 - t0,
                  t_current (duration_type::zero());

    for (int j = 0; j < N; ++j) {
      FCL_REAL distLB;
      Matrix3f BB (B.matrix(j));
      Vec3f TT (T.row(j)), aa (a.row(j)), bb (b.row(j));

      t0 = clock_type::now();
      bool res = obbDisjointAndLowerBoundDistance (BB, TT, aa, bb, request, distLB);
      t1 = clock_type::now();
      t_current += (t1 - t0);
      current_distLB2[j] = res ? distLB : 0;
    }

    if (bench)
      *bench << t_aligned.count()
        << ' ' << t_current.count()
        << ' ' << aligned_distLB2.format(benchFormat)
        << ' ' << current_distLB2.format(benchFormat)
        << ' ' << (current_distLB2 == 0).select (
            (aligned_distLB2 == 0).select (
              aligned_distLB2.Constant(std::numeric_limits<FCL_REAL>::quiet_NaN()),
              -1.),
            aligned_distLB2 / current_distLB2
            ).format(benchFormat)
        << std::endl;
  }
}

BOOST_AUTO_TEST_CASE(aligned_disjoint_benchmark)
{
  std::ofstream ofs ("benchmark_result");
  bench = &ofs;
  benchmark_aligned_disjoint<HPP_FCL_VECTORIZE_OBB_DISJOINT>();
  bench = NULL;
}

#endif

BOOST_AUTO_TEST_SUITE_END()
