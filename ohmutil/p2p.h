//
// author Kazys Stepanas
//
#ifndef OHMUTIL_P2P_H
#define OHMUTIL_P2P_H

#include <glm/glm.hpp>

#ifdef TES_ENABLE
#include "3esvector3.h"

#include <cstddef>

inline tes::Vector3f p2p(const glm::vec3 &p)
{
  return tes::Vector3f(p.x, p.y, p.z);
}

inline glm::vec3 p2p(const tes::Vector3f &p)
{
  return glm::vec3(p.x, p.y, p.z);
}

inline const tes::Vector3f *p2pArray(const glm::vec3 *points)
{
  static_assert(sizeof(tes::Vector3f) == sizeof(glm::vec3), "tes::Vector3f size does not match glm::vec3 size.");
  return reinterpret_cast<const tes::Vector3f *>(points);
}

inline const glm::vec3 *p2pArray(const tes::Vector3f *points)
{
  static_assert(sizeof(tes::Vector3f) == sizeof(glm::vec3), "tes::Vector3f size does not match glm::vec3 size.");
  return reinterpret_cast<const glm::vec3 *>(points);
}

#endif // TES_ENABLE

#endif  // OHMUTIL_P2P_H
