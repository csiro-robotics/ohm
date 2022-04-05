// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_KEYSTREAM_H_
#define OHM_KEYSTREAM_H_

#include "OhmConfig.h"

#include "Key.h"

#include <ostream>

inline std::ostream &operator<<(std::ostream &o, const ohm::Key &key)
{
  o << "[(" << key.regionKey().x << ',' << key.regionKey().y << ',' << key.regionKey().z << "):("
    << int(key.localKey().x) << ',' << int(key.localKey().y) << ',' << int(key.localKey().z) << ")]";
  return o;
}

#endif  // OHM_KEYSTREAM_H_
