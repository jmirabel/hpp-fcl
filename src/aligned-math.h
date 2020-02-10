/** \author Joseph Mirabel */

#ifndef HPP_FCL_SRC_ALIGNED_MATH_H
#define HPP_FCL_SRC_ALIGNED_MATH_H

#include <hpp/fcl/data_types.h>

namespace hpp
{
namespace fcl
{

/// @brief Class that computates several operations on Matrix3f in parallel.
/// \tparam N should be set according to the compilation option. Typically,
///         N should be the number of FCL_REAL that fits into a register of
///         vectorize operations.
template <int N>
struct AlignedMatrix3f {
  typedef Eigen::Matrix<FCL_REAL, N, 3> AlignedVec3f;
  typedef Eigen::Array<FCL_REAL, N, 3> AlignedArr3f;

  typedef Eigen::Map<Matrix3f      , 0, Eigen::Stride<3*N,N> > InnerMap;
  typedef Eigen::Map<const Matrix3f, 0, Eigen::Stride<3*N,N> > InnerMapConst;

  typedef Eigen::Matrix<FCL_REAL, N, 9> Storage_t;
  Storage_t storage;

  InnerMap      matrix(Eigen::Index i)       { return InnerMap     (&storage(i,0)); }
  InnerMapConst matrix(Eigen::Index i) const { return InnerMapConst(&storage(i,0)); }

  template <Eigen::Index _N> InnerMap      matrix(Eigen::Index i = _N)       { return InnerMap     (&storage(i,0)); }
  template <Eigen::Index _N> InnerMapConst matrix(Eigen::Index i = _N) const { return InnerMapConst(&storage(i,0)); }

  typename Storage_t::ColXpr      operator() (Eigen::Index r, Eigen::Index j)       { return storage.col(r + 3*j); }
  typename Storage_t::ConstColXpr operator() (Eigen::Index r, Eigen::Index j) const { return storage.col(r + 3*j); }

  void mult (const Eigen::Matrix<FCL_REAL, N, 3>& vin,
      Eigen::Matrix<FCL_REAL, N, 3>& vout) const
  {
    for (int k = 0; k < 3; ++k)
      vout.col(k) =
          storage.col(  k).array() * vin.array().col(0)
        + storage.col(3+k).array() * vin.array().col(1)
        + storage.col(6+k).array() * vin.array().col(2);
  }

  void mult (const Eigen::Matrix<FCL_REAL, 3, 1>& vin,
      Eigen::Matrix<FCL_REAL, N, 3>& vout) const
  {
    for (int k = 0; k < 3; ++k)
      vout.col(k) =
          storage.col(  k) * vin(0)
        + storage.col(3+k) * vin(1)
        + storage.col(6+k) * vin(2);

  }

  void transposeMult (const Eigen::Matrix<FCL_REAL, N, 3>& vin,
      Eigen::Matrix<FCL_REAL, N, 3>& vout) const
  {
    for (int k = 0; k < 3; ++k) {
      vout.col(k) =
          storage.col(3*k+0).array() * vin.array().col(0)
        + storage.col(3*k+1).array() * vin.array().col(1)
        + storage.col(3*k+2).array() * vin.array().col(2);
    }
  }

  void transposeMultAdd (const Eigen::Matrix<FCL_REAL, N, 3>& vin,
      Eigen::Matrix<FCL_REAL, N, 3>& vout) const
  {
    for (int k = 0; k < 3; ++k) {
      vout.col(k).array() +=
          storage.col(3*k+0).array() * vin.array().col(0)
        + storage.col(3*k+1).array() * vin.array().col(1)
        + storage.col(3*k+2).array() * vin.array().col(2);
    }
  }

  void transposeMultSubstract (const Eigen::Matrix<FCL_REAL, N, 3>& vin,
      Eigen::Matrix<FCL_REAL, N, 3>& vout) const
  {
    for (int k = 0; k < 3; ++k) {
      vout.col(k).array() -=
          storage.col(3*k+0).array() * vin.array().col(0)
        + storage.col(3*k+1).array() * vin.array().col(1)
        + storage.col(3*k+2).array() * vin.array().col(2);
    }
  }

  void transposeMult (const Eigen::Matrix<FCL_REAL, 3, 1>& vin,
      Eigen::Matrix<FCL_REAL, N, 3>& vout) const
  {
    for (int k = 0; k < 3; ++k)
      vout.col(k) =
          storage.col(3*k+0) * vin(0)
        + storage.col(3*k+1) * vin(1)
        + storage.col(3*k+2) * vin(2);

  }
};

} // namespace fcl

} // namespace hpp


#endif
