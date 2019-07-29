// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuTransformSamples.h"

#include "private/GpuTransformSamplesDetail.h"

#include "OhmGpu.h"

#include "private/GpuProgramRef.h"

#include <gputil/gpuEvent.h>
#include <gputil/gpuEventList.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuPinnedBuffer.h>
#include <gputil/gpuPlatform.h>
#include <gputil/gpuProgram.h>

#include <glm/ext.hpp>

#include <algorithm>
#include <mutex>

#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
#include "TransformSamplesResource.h"
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(transformTimestampedPoints);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

using namespace ohm;

namespace
{
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref("TransformSamples", GpuProgramRef::kSourceString, TransformSamplesCode,
                            TransformSamplesCode_length);
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref("TransformSamples", GpuProgramRef::kSourceFile, "TransformSamples.cl");
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

  inline bool goodSample(const glm::dvec3 &sample, double max_range)
  {
    if (glm::any(glm::isnan(sample)))
    {
      return false;
    }

    if (glm::dot(sample, sample) > max_range)
    {
      return false;
    }

    return true;
  }
}  // namespace


GpuTransformSamples::GpuTransformSamples(gputil::Device &gpu)
  : imp_(new GpuTransformSamplesDetail)
{
  imp_->gpu = gpu;
  imp_->transform_positions_buffer = gputil::Buffer(gpu, sizeof(gputil::float3) * 8, gputil::kBfReadHost);
  imp_->transform_rotations_buffer = gputil::Buffer(gpu, sizeof(gputil::float4) * 8, gputil::kBfReadHost);
  imp_->transform_times_buffer = gputil::Buffer(gpu, sizeof(float) * 8, gputil::kBfReadHost);
  if (program_ref.addReference(gpu))
  {
    imp_->kernel = GPUTIL_MAKE_KERNEL(program_ref.program(), transformTimestampedPoints);
    imp_->kernel.calculateOptimalWorkGroupSize();
  }
}


GpuTransformSamples::GpuTransformSamples(GpuTransformSamples &&other)
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


GpuTransformSamples::~GpuTransformSamples()
{
  if (imp_ && imp_->kernel.isValid())
  {
    imp_->kernel = gputil::Kernel();
    program_ref.releaseReference();
  }
  delete imp_;
}


unsigned GpuTransformSamples::transform(const double *transform_times, const glm::dvec3 *transform_translations,
                                        const glm::dquat *transform_rotations, unsigned transform_count,
                                        const double *sample_times, const glm::dvec3 *local_samples,
                                        unsigned point_count, gputil::Queue &gpu_queue, gputil::Buffer &output_buffer,
                                        gputil::Event &completion_event, double max_range)
{
  if (point_count == 0 || transform_count == 0)
  {
    return 0u;
  }

  // Wait on outstanding operations to complete.
  gputil::Event::wait(imp_->upload_events, GpuTransformSamplesDetail::kUploadEventCount);

  // Reserve GPU memory for the rays. We reserve space for the provides samples paired with transformed origin point.
  // This results in an interleaved spacing with:
  //    reserved_ray_origin
  //    local_sample_point
  //    reserved_ray_origin
  //    local_sample_point
  //    reserved_ray_origin
  //    local_sample_point
  //    ...
  //
  // To preserve memory, we use the reserve_ray_origin.x value to store the associated timestamps for the samples.
  // To address lack of double precision support, we'll make the times relative to the first transform timestamp.
  output_buffer.resize(sizeof(gputil::float3) * 2 * point_count);
  gputil::PinnedBuffer ray_buffer(output_buffer, gputil::kPinWrite);
  glm::vec3 sample_time, sample;

  unsigned upload_count = 0u;
  const double base_time = transform_times[0];
  const float max_time = float(transform_times[transform_count - 1] - base_time);
  for (unsigned i = 0; i < point_count; ++i)
  {
    sample_time = glm::vec3(float(sample_times[i] - base_time));
    sample = glm::vec3(local_samples[i]);
    if (!goodSample(sample, max_range))
    {
      continue;
    }
    ray_buffer.write(glm::value_ptr(sample_time), sizeof(sample_time), (upload_count + 0) * sizeof(gputil::float3));
    ray_buffer.write(glm::value_ptr(sample), sizeof(sample), (upload_count + 1) * sizeof(gputil::float3));
    upload_count += 2;
  }

  if (upload_count == 0)
  {
    return 0u;
  }

  // Asynchronous unpin. Kernels will wait on the associated event.
  ray_buffer.unpin(&gpu_queue, nullptr, &imp_->upload_events[0]);

  // Upload transforms.
  // FIXME: resolve device
  imp_->transform_positions_buffer.resize(sizeof(gputil::float3) * transform_count);
  imp_->transform_rotations_buffer.resize(sizeof(gputil::float4) * transform_count);
  imp_->transform_times_buffer.resize(sizeof(float) * transform_count);
  gputil::PinnedBuffer positions_buffer(imp_->transform_positions_buffer, gputil::kPinWrite);
  gputil::PinnedBuffer rotations_buffer(imp_->transform_rotations_buffer, gputil::kPinWrite);
  gputil::PinnedBuffer times_buffer(imp_->transform_times_buffer, gputil::kPinWrite);

  glm::vec3 position;
  glm::quat rotation;
  float single_precision_timestamp;
  for (unsigned i = 0; i < transform_count; ++i)
  {
    position = glm::vec3(transform_translations[i]);
    rotation = glm::quat(transform_rotations[i]);
    positions_buffer.write(glm::value_ptr(position), sizeof(position), i * sizeof(gputil::float3));
    rotations_buffer.write(glm::value_ptr(rotation), sizeof(rotation), i * sizeof(gputil::float4));
    single_precision_timestamp = std::max(0.0f, std::min(float(transform_times[i] - base_time), max_time));
    times_buffer.write(&single_precision_timestamp, sizeof(single_precision_timestamp),
                       i * sizeof(single_precision_timestamp));
  }

  // Asynchronous unpin.
  positions_buffer.unpin(&gpu_queue, nullptr, &imp_->upload_events[1]);
  rotations_buffer.unpin(&gpu_queue, nullptr, &imp_->upload_events[2]);
  times_buffer.unpin(&gpu_queue, nullptr, &imp_->upload_events[3]);

  // Set a max global size.
  const unsigned sample_count = upload_count / 2;
  const unsigned max_global_threads = 8 * 1024u;
  const unsigned min_batch_size = 8;
  const unsigned batch_size = std::max((sample_count + max_global_threads - 1) / max_global_threads, min_batch_size);

  gputil::Dim3 global_size((sample_count + batch_size - 1) / batch_size);
  gputil::Dim3 local_size(std::min(imp_->kernel.optimalWorkGroupSize(), global_size.x));
  gputil::EventList wait(imp_->upload_events, GpuTransformSamplesDetail::kUploadEventCount);
  imp_->kernel(global_size, local_size, wait, completion_event, &gpu_queue,
               gputil::BufferArg<gputil::float3 *>(output_buffer), sample_count,
               gputil::BufferArg<float *>(imp_->transform_times_buffer),
               gputil::BufferArg<gputil::float3 *>(imp_->transform_positions_buffer),
               gputil::BufferArg<gputil::float4 *>(imp_->transform_rotations_buffer), transform_count, batch_size);

  gpu_queue.flush();

  return upload_count;
}
