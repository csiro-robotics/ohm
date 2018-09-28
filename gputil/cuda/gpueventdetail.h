// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUEVENTDETAIL_H_
#define GPUEVENTDETAIL_H_

#include "gpuconfig.h"

#include "cuda/ref.h"

#include <cuda_runtime.h>

#include <utility>

namespace gputil
{
  void destroyEvent(cudaEvent_t event);

  struct EventDetail : public Ref<cudaEvent_t>
  {
    inline EventDetail(cudaEvent_t obj, unsigned initialRefCount, const ReleaseFunc &release)
      : Ref<cudaEvent_t>(obj, initialRefCount, release)
    {
    }

    inline EventDetail(Ref &&other)
      : Ref<cudaEvent_t>(std::move(other))
    {
    }

    inline EventDetail(const Ref &other) = delete;

  protected:
    inline ~EventDetail() {}
  };
}

#endif // GPUEVENTDETAIL_H_
