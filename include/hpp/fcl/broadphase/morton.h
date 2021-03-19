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

#ifndef HPP_FCL_MORTON_H
#define HPP_FCL_MORTON_H

#include <boost/dynamic_bitset.hpp>
#include <boost/cstdint.hpp>

#include <hpp/fcl/data_types.h>
#include <hpp/fcl/BV/AABB.h>

namespace hpp {
namespace fcl {

typedef boost::uint64_t uint64_t;
typedef boost::int64_t int64_t;
typedef boost::uint32_t uint32_t;
typedef boost::int32_t int32_t;

/// @cond IGNORE
namespace details
{

static inline uint32_t quantize(FCL_REAL x, uint32_t n)
{
  return std::max(std::min((uint32_t)(x * (FCL_REAL)n), uint32_t(n-1)), uint32_t(0));
}

/// @brief compute 30 bit morton code
static inline uint32_t morton_code(uint32_t x, uint32_t y, uint32_t z)
{
  x = (x | (x << 16)) & 0x030000FF; 
  x = (x | (x <<  8)) & 0x0300F00F; 
  x = (x | (x <<  4)) & 0x030C30C3; 
  x = (x | (x <<  2)) & 0x09249249; 

  y = (y | (y << 16)) & 0x030000FF; 
  y = (y | (y <<  8)) & 0x0300F00F; 
  y = (y | (y <<  4)) & 0x030C30C3; 
  y = (y | (y <<  2)) & 0x09249249; 

  z = (z | (z << 16)) & 0x030000FF; 
  z = (z | (z <<  8)) & 0x0300F00F; 
  z = (z | (z <<  4)) & 0x030C30C3; 
  z = (z | (z <<  2)) & 0x09249249; 

  return x | (y << 1) | (z << 2);
}

/// @brief compute 60 bit morton code
static inline uint64_t morton_code60(uint32_t x, uint32_t y, uint32_t z)
{
  uint32_t lo_x = x & 1023u;
  uint32_t lo_y = y & 1023u;
  uint32_t lo_z = z & 1023u;
  uint32_t hi_x = x >> 10u;
  uint32_t hi_y = y >> 10u;
  uint32_t hi_z = z >> 10u;

  return (uint64_t(morton_code(hi_x, hi_y, hi_z)) << 30) | uint64_t(morton_code(lo_x, lo_y, lo_z));
}

}
/// @endcond


/// @brief Functor to compute the morton code for a given AABB
template<typename T>
struct morton_functor {};


/// @brief Functor to compute 30 bit morton code for a given AABB
template<>
struct morton_functor<uint32_t>
{
  morton_functor(const AABB& bbox) : base(bbox.min_), 
                                     inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
                                         1.0 / (bbox.max_[1] - bbox.min_[1]),
                                         1.0 / (bbox.max_[2] - bbox.min_[2]))
  {}

  uint32_t operator() (const Vec3f& point) const
  {
    uint32_t x = details::quantize((point[0] - base[0]) * inv[0], 1024u);
    uint32_t y = details::quantize((point[1] - base[1]) * inv[1], 1024u);
    uint32_t z = details::quantize((point[2] - base[2]) * inv[2], 1024u);
    
    return details::morton_code(x, y, z);
  }

  const Vec3f base;
  const Vec3f inv;

  size_t bits() const { return 30; }
};


/// @brief Functor to compute 60 bit morton code for a given AABB
template<>
struct morton_functor<uint64_t>
{
  morton_functor(const AABB& bbox) : base(bbox.min_),
                                     inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
                                         1.0 / (bbox.max_[1] - bbox.min_[1]),
                                         1.0 / (bbox.max_[2] - bbox.min_[2]))
  {}

  uint64_t operator() (const Vec3f& point) const
  {
    uint32_t x = details::quantize((point[0] - base[0]) * inv[0], 1u << 20);
    uint32_t y = details::quantize((point[1] - base[1]) * inv[1], 1u << 20);
    uint32_t z = details::quantize((point[2] - base[2]) * inv[2], 1u << 20);

    return details::morton_code60(x, y, z);
  }

  const Vec3f base;
  const Vec3f inv;

  size_t bits() const { return 60; }
};

/// @brief Functor to compute n bit morton code for a given AABB
template<>
struct morton_functor<boost::dynamic_bitset<> >
{
  morton_functor(const AABB& bbox, int bit_num_) : base(bbox.min_),
                                                      inv(bbox.diagonal().cwiseInverse()),
                                                      bit_num(bit_num_)
  {}

  boost::dynamic_bitset<> operator() (const Vec3f& point) const
  {
    FCL_REAL x = (point[0] - base[0]) * inv[0];
    FCL_REAL y = (point[1] - base[1]) * inv[1];
    FCL_REAL z = (point[2] - base[2]) * inv[2];
    int start_bit = bit_num * 3 - 1;
    boost::dynamic_bitset<> bits(bit_num * 3);

    x *= 2;
    y *= 2;
    z *= 2;

    for(int i = 0; i < bit_num; ++i)
    {
      bits[start_bit--] = ((z < 1) ? 0 : 1);
      bits[start_bit--] = ((y < 1) ? 0 : 1);
      bits[start_bit--] = ((x < 1) ? 0 : 1);
      x = ((x >= 1) ? 2*(x-1) : 2*x);
      y = ((y >= 1) ? 2*(y-1) : 2*y);
      z = ((z >= 1) ? 2*(z-1) : 2*z);
    }

    return bits;
  }

  const Vec3f base;
  const Vec3f inv;
  const int bit_num;

  int bits() const { return bit_num * 3; }
};

} // namespace fcl
} // namespace hpp

#endif
