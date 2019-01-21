// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYUTIL
#define OCCUPANCYUTIL

#include "OhmConfig.h"

#include "Key.h"

#include <iostream>

#include <glm/glm.hpp>


namespace ohm
{
  // From GLM 0.9.9.1, numbering starting including a patch number.
  // So 0.9.8 was version 98, 0.9.9 was 99 and 0.9.9.1 was 991
  // This assumes version 1.0 will be 1000
#if GLM_VERSION < 99
  using GlmQualifier = glm::precision;
#else   // GLM_VERSION
  using GlmQualifier = glm::qualifier;
#endif  // GLM_VERSION

  template <typename T, GlmQualifier Q>
  inline T volumeOf(const glm::tvec3<T, Q> &expanse)
  {
    return expanse.x * expanse.y * expanse.z;
  }
}  // namespace ohm


inline std::ostream &operator<<(std::ostream &out, const ohm::Key &key)
{
  out << "[ R(" << key.regionKey().x << ',' << key.regionKey().y << ',' << key.regionKey().z << ") V("
      << int(key.localKey().x) << ',' << int(key.localKey().y) << ',' << int(key.localKey().z) << ") ]";
  return out;
}

template <typename T, ohm::GlmQualifier Q>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec2<T, Q> &v)
{
  out << '(' << v.x << ',' << v.y << ')';
  return out;
}

template <typename T, ohm::GlmQualifier Q>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec3<T, Q> &v)
{
  out << '(' << v.x << ',' << v.y << ',' << v.z << ')';
  return out;
}

template <typename T, ohm::GlmQualifier Q>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec4<T, Q> &v)
{
  out << '(' << v.w << ',' << v.x << ',' << v.y << ',' << v.z << ')';
  return out;
}

template <ohm::GlmQualifier Q>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec3<char, Q> &v)
{
  out << '(' << (int)v.x << ',' << (int)v.y << ',' << (int)v.z << ')';
  return out;
}

template <ohm::GlmQualifier Q>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec3<unsigned char, Q> &v)
{
  out << '(' << (int)v.x << ',' << (int)v.y << ',' << (int)v.z << ')';
  return out;
}

template <ohm::GlmQualifier Q>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec4<char, Q> &v)
{
  out << '(' << (int)v.w << ',' << (int)v.x << ',' << (int)v.y << ',' << (int)v.z << ')';
  return out;
}

template <ohm::GlmQualifier Q>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec4<unsigned char, Q> &v)
{
  out << '(' << (int)v.w << ',' << (int)v.x << ',' << (int)v.y << ',' << (int)v.z << ')';
  return out;
}

#endif  // OCCUPANCYUTIL
