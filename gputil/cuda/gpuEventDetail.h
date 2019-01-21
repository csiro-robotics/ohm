// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUEVENTDETAIL_H
#define GPUEVENTDETAIL_H

#include "gpuConfig.h"

#include "cuda/ref.h"

#include <cuda_runtime.h>

#include <utility>

namespace gputil
{
  void destroyEvent(cudaEvent_t event);

  struct EventDetail : public Ref<cudaEvent_t>
  {
    inline EventDetail(cudaEvent_t obj, unsigned initial_ref_count, const ReleaseFunc &release)
      : Ref<cudaEvent_t>(obj, initial_ref_count, release)
    {}

    inline EventDetail(Ref &&other)
      : Ref<cudaEvent_t>(std::move(other))
    {}

    inline EventDetail(const Ref &other) = delete;

  protected:
    inline ~EventDetail() {}
  };
}  // namespace gputil

#endif  // GPUEVENTDETAIL_H
