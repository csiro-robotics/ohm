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
  template <typename T, glm::precision P>
  inline T volumeOf(const glm::tvec3<T, P> &expanse)
  {
    return expanse.x * expanse.y * expanse.z;
  }
}


inline std::ostream &operator<<(std::ostream &out, const ohm::OccupancyKey &key)
{
  out << "[ R(" << key.regionKey().x << ',' << key.regionKey().y << ',' << key.regionKey().z
      << ") V(" << int(key.localKey().x) << ',' << int(key.localKey().y) << ',' << int(key.localKey().z)
      << ") ]";
  return out;
}

template <typename T, glm::precision P>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec2<T, P> &v)
{
  out << '(' << v.x << ',' << v.y << ')';
  return out;
}

template <typename T, glm::precision P>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec3<T, P> &v)
{
  out << '(' << v.x << ',' << v.y << ',' << v.z << ')';
  return out;
}

template <typename T, glm::precision P>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec4<T, P> &v)
{
  out << '(' << v.w << ',' << v.x << ',' << v.y << ',' << v.z << ')';
  return out;
}

template <glm::precision P>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec3<char, P> &v)
{
  out << '(' << (int)v.x << ',' << (int)v.y << ',' << (int)v.z << ')';
  return out;
}

template <glm::precision P>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec3<unsigned char, P> &v)
{
  out << '(' << (int)v.x << ',' << (int)v.y << ',' << (int)v.z << ')';
  return out;
}

template <glm::precision P>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec4<char, P> &v)
{
  out << '(' << (int)v.w << ',' << (int)v.x << ',' << (int)v.y << ',' << (int)v.z << ')';
  return out;
}

template <glm::precision P>
inline std::ostream &operator<<(std::ostream &out, const glm::tvec4<unsigned char, P> &v)
{
  out << '(' << (int)v.w << ',' << (int)v.x << ',' << (int)v.y << ',' << (int)v.z << ')';
  return out;
}

#endif // OCCUPANCYUTIL
