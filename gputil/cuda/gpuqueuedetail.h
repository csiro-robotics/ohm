// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUQUEUEDETAIL_H_
#define GPUQUEUEDETAIL_H_

#include "gpuconfig.h"

#include "cuda/ref.h"

#include <cuda_runtime.h>

#include <utility>

namespace gputil
{
  struct QueueDetail : public Ref<cudaStream_t>
  {
    inline QueueDetail(cudaStream_t obj, unsigned initialRefCount, const ReleaseFunc &release)
      : Ref<cudaStream_t>(obj, initialRefCount, release)
    {

    }

    inline QueueDetail(Ref &&other)
      : Ref<cudaStream_t>(std::move(other))
    {
    }

    inline QueueDetail(const Ref &other) = delete;

  protected:
    inline ~QueueDetail() {}
  };
}

#endif // GPUQUEUEDETAIL_H_
