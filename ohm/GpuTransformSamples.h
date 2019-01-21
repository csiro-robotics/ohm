// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPU_TRANSFORM_SAMPLES_H
#define GPU_TRANSFORM_SAMPLES_H

#include "OhmConfig.h"

#include <glm/fwd.hpp>

#include <limits>

namespace gputil
{
  class Buffer;
  class Device;
  class Event;
  class Queue;
}  // namespace gputil

namespace ohm
{
  struct GpuTransformSamplesDetail;

  /// A utility class for transforming local samples into a global frame using the Gpu.
  ///
  /// See @c transform() for details.
  class GpuTransformSamples
  {
  public:
    GpuTransformSamples(gputil::Device &gpu);
    GpuTransformSamples(GpuTransformSamples &&other);

    ~GpuTransformSamples();

    /// Transform rays in the indicated buffer from local space representation into a global frame.
    ///
    /// This local sample points specified in @p local_samples with corresponding @p sample_times and transforms them
    /// into a global frame using the GPU. The transforms are specified by @p transform_times, @p transform_translations
    /// and @p transform_rotations - scaling is not supported. These transforms specify the origin local to global
    /// transformations. The @p transform_times range must encompass all @p sample_times. The number of transforms
    /// should generally be small and each GPU thread executed a binary search for its appropriate sample.
    ///
    /// On successful execution, @p @p output_buffer is resize to hold @p point_count * 2 @c gputil::float3 entries.
    /// These entries are line segment pairs in the output space. The first item of each pair is the sensor origin
    /// for the sample, and the second is the transformed sample point. The @p completion_event is may be used to
    /// mark completion of the transformation as the GPU execution is asynchronous.
    ///
    /// Input samples are filtered for infinite and NaN points, which are not uploaded to the GPU. This means the
    /// items in @p output_buffer may not directly correspond to the @p local_samples. The number of result pairs is
    /// given by the return value.
    ///
    /// @param transform_times Array of timestamps for the local to global transforms.
    /// @param transform_translations Array of translation components of the local to global transforms.
    /// @param transform_rotations Array of quaternion rotations of the local to global transforms.
    /// @param transform_count number of entries in @p transform_times, @p transform_translations and
    ///   @p transform_rotations.
    /// @param sample_times Array of timestamps for @p local_samples.
    /// @param local_samples The sample points to transform in local sensor space.
    /// @param point_count Number of items in @p local_samples and @p sample_times.
    /// @param gpu_queue The queue in which to execute the GPU operations.
    /// @param output_buffer GPU buffer to calculate the results in. Will be resized if too small.
    /// @param completion_event Event which may be used to monitor GPU completion of the translations.
    /// @param max_range Maximum allowed distance length of a valid ray (sensor to sample distance).
    ///   Longer rays are rejected.
    /// @return The number of valid samples queued for translation on GPU.
    unsigned transform(const double *transform_times, const glm::dvec3 *transform_translations,
                       const glm::dquat *transform_rotations, unsigned transform_count, const double *sample_times,
                       const glm::dvec3 *local_samples, unsigned point_count, gputil::Queue &gpu_queue,
                       gputil::Buffer &output_buffer, gputil::Event &completion_event,
                       double max_range = std::numeric_limits<double>::infinity());

  private:
    GpuTransformSamplesDetail *imp_;
  };

}  // namespace ohm

#endif  // GPU_TRANSFORM_SAMPLES_H
