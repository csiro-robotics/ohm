// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUCACHEPOSTSYNCHANDLER_H
#define GPUCACHEPOSTSYNCHANDLER_H

#include <OhmConfig.h>

#include <glm/fwd.hpp>

#include <functional>

namespace ohm
{
  struct MapChunk;

  using GpuCachePostSyncHandler = std::function<void(MapChunk *, const glm::u8vec3 &)>;
}  // namespace ohm

#endif  // GPUCACHEPOSTSYNCHANDLER_H
