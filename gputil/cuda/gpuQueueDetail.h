// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUQUEUEDETAIL_H
#define GPUQUEUEDETAIL_H

#include "gpuConfig.h"

#include "gputil/cuda/ref.h"

#include <cuda_runtime.h>

#include <utility>

namespace gputil
{
struct QueueDetail : public Ref<cudaStream_t>
{
  inline QueueDetail(cudaStream_t obj, unsigned initial_ref_count, const ReleaseFunc &release)
    : Ref<cudaStream_t>(obj, initial_ref_count, release)
  {}

  inline QueueDetail(Ref &&other)
    : Ref<cudaStream_t>(std::move(other))
  {}

  inline QueueDetail(const Ref &other) = delete;

protected:
  inline ~QueueDetail() = default;
};
}  // namespace gputil

#endif  // GPUQUEUEDETAIL_H
