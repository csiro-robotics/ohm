// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/GpuTransformSamples.h>
#include <ohm/OhmGpu.h>

#include <ohmtools/OhmCloud.h>
#include <ohmutil/OhmUtil.h>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuDevice.h>
#include <gputil/gpuEvent.h>
#include <gputil/gpuPinnedBuffer.h>

#include <glm/ext.hpp>
#include <glm/glm.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include "gputil/gpuPlatform.h"

using namespace ohm;

namespace
{
  using Clock = std::chrono::high_resolution_clock;
}

namespace gpumap
{
  void gpuTransformSamples(const std::vector<glm::dvec3> &samples_global, bool compare_performance)
  {
    ASSERT_GT(samples_global.size(), 0u);

    // Generate a base reference time using a unix style timestamp for now.
    // This tests likely data input types.
    auto unix_timestamp = std::chrono::milliseconds(std::time(nullptr));
    const double base_time = std::chrono::duration_cast<std::chrono::duration<double>>(unix_timestamp).count();
    const double time_increment = 1e-3;

    // Work out a time range based on the number of samples and time increment.
    // Make the interval larger than required to cover the samples.
    const double time_max = base_time + time_increment * (samples_global.size() + 10);
    const unsigned transforms_count = 10;

    // Generate a moving local frame.
    std::vector<double> timestamps(transforms_count);
    std::vector<glm::dvec3> translations(transforms_count);
    std::vector<glm::dquat> rotations(transforms_count);

    const glm::dvec3 start_point(-0.42);
    const glm::dvec3 end_point(10.0f);

    const glm::dquat start_rotation(1, 0, 0, 0);
    const glm::dquat end_rotation = glm::rotate(glm::dquat(1, 0, 0, 0), glm::pi<double>(), glm::dvec3(0, 0, 1));

    double timestamp = base_time;
    // Ensure time range covers the samples with padding at either end.
    const double end_time = base_time + samples_global.size() * time_increment + 1.5 * time_increment;
    double interpolation_step = 1.0 / (transforms_count - 1);
    for (unsigned i = 0; i < transforms_count; ++i)
    {
      timestamps[i] = base_time + (end_time - base_time) * (i / double(transforms_count - 1));
      translations[i] = start_point + (i * interpolation_step) * (end_point - start_point);
      rotations[i] = start_rotation * glm::slerp(start_rotation, end_rotation, i * interpolation_step);
    }

    std::vector<double> sample_times(samples_global.size());
    timestamp = base_time + 0.67 * time_increment;  // Reset timestamp for samples.
    for (size_t i = 0; i < samples_global.size(); ++i)
    {
      ASSERT_GE(timestamp, timestamps.front());
      ASSERT_LE(timestamp, timestamps.back());
      sample_times[i] = timestamp;
      timestamp += time_increment;
    }

    // Transform samples into a moving, local frame.
    std::vector<glm::dvec3> samples_local(samples_global.size());
    glm::dvec3 p;
    glm::dvec3 translation;
    glm::dquat rotation;
    double lerp;
    unsigned tidx = 0;

    for (size_t i = 0; i < samples_global.size(); ++i)
    {
      // Find the appropriate transform indices.
      while (timestamps[tidx + 1] < sample_times[i])
      {
        ++tidx;
        ASSERT_LT(tidx + 1, timestamps.size()) << "out of bounds";
      }

      lerp = (sample_times[i] - timestamps[tidx]) / (timestamps[tidx + 1] - timestamps[tidx]);

      translation = translations[tidx] + lerp * (translations[tidx + 1] - translations[tidx]);
      rotation = rotations[tidx] * glm::slerp(rotations[tidx], rotations[tidx + 1], lerp);

      // Apply inverse transform for global => local
      p = glm::inverse(rotation) * (samples_global[i] - translation);
      samples_local[i] = p;
      // Validate the transformation back to global.
      // printf("Gen: %f(%f)  T(%f %f %f) R(%f %f %f %f)\n", sample_times[i] - base_time, lerp, translation.x,
      //       translation.y, translation.z, rotation.w, rotation.x, rotation.y, rotation.z);
      p = rotation * samples_local[i] + translation;
      glm::dvec3 diff = samples_global[i] - p;
      ASSERT_NEAR(glm::length(diff), 0.0, 1e-7);
    }

    // Perform reverse transform for performance testing and to make sure the transforms are right.
    // Note we use double precision transforms here, the GPU is single precision.
    Clock::duration cpu_time;
    if (compare_performance)
    {
      std::vector<glm::dvec3> samples_global_cpu(samples_local.size());

      const auto cpu_start = Clock::now();
      unsigned tidx = 0;
      for (size_t i = 0; i < samples_local.size(); ++i)
      {
        timestamp = sample_times[i];
        p = samples_local[i];

        // Find the appropriate transform indices.
        while (timestamps[tidx + 1] < timestamp)
        {
          ++tidx;
          ASSERT_LT(tidx + 1, timestamps.size()) << "out of bounds";
        }

        lerp = (sample_times[i] - timestamps[tidx]) / (timestamps[tidx + 1] - timestamps[tidx]);

        translation = translations[tidx] + lerp * (translations[tidx + 1] - translations[tidx]);
        rotation = rotations[tidx] * glm::slerp(rotations[tidx], rotations[tidx + 1], lerp);

        samples_global_cpu[i] = rotation * p + translation;
      }
      cpu_time = Clock::now() - cpu_start;
      std::cout << "CPU Execution time: " << cpu_time << std::endl;

      // Validate.
      glm::dvec3 sample, expect;
      glm::dvec3 diff;
      for (size_t i = 0; i < samples_global.size(); ++i)
      {
        expect = samples_global[i];
        sample = samples_global_cpu[i];
        diff = expect - sample;
        EXPECT_NEAR(glm::length(diff), 0.0, 1e-7);
      }
    }

    // Do the GPU transformation.
    gputil::Device gpu = ohm::gpuDevice();
    GpuTransformSamples transformation(gpu);

    gputil::Buffer output_buffer(gpu, 0, gputil::kBfReadWriteHost);
    gputil::Event completion_event;
    const auto gpu_start = Clock::now();
    unsigned ray_count = transformation.transform(
      timestamps.data(), translations.data(), rotations.data(), transforms_count, sample_times.data(),
      samples_local.data(), unsigned(samples_local.size()), gpu.defaultQueue(), output_buffer, completion_event);

    ASSERT_GT(ray_count, 0u);
    ray_count /= 2;
    ASSERT_EQ(ray_count, samples_global.size());

    completion_event.wait();

    const auto gpu_time = Clock::now() - gpu_start;
    std::cout << "GPU Execution time: " << gpu_time << std::endl;

    // Read back the results.
    ASSERT_GE(output_buffer.size(), sizeof(gputil::float3) * 2 * ray_count);
    // Results are rays: sensor origin, global sample
    // Note have to copy out single precision results as that is what the GPU uses.
    std::vector<glm::vec3> rays(2 * ray_count);
    gputil::PinnedBuffer rays_buffer(output_buffer, gputil::kPinRead);
    rays_buffer.readElements<gputil::float3>(rays.data(), 2 * ray_count);
    rays_buffer.unpin();

    // Validate results.
    glm::dvec3 sample, expect;
    glm::dvec3 diff;
    for (size_t i = 0; i < samples_global.size(); ++i)
    {
      expect = samples_global[i];
      sample = rays[i * 2 + 1];
      diff = expect - sample;
      EXPECT_NEAR(glm::length(diff), 0.0, 1e-4);
    }

    // Not currently fast enough. Revisit once I have a clear idea on how many input points to expect.
//#ifdef NDEBUG
//    if (compare_performance)
//    {
//      EXPECT_LE(gpu_time, cpu_time) << "CPU execution less than GPU.";
//    }
//#endif // NDEBUG
  }


  TEST(Gpu, TransformSample)
  {
    // Just transform a single sample for this test.
    gpuTransformSamples({ glm::dvec3(10.0f, 20.0f, 15.0f) }, false);
  }


  TEST(Gpu, TransformSamples)
  {
    // Just transform a single sample for this test.
    std::vector<glm::dvec3> samples_global;
    std::mt19937 rand_engine;
    std::uniform_real_distribution<double> rand(-10.0f, 25.0f);
    const unsigned sample_count = 16 * 1024u;

    samples_global.reserve(sample_count);
    while (samples_global.size() < sample_count)
    {
      samples_global.emplace_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
    }

    gpuTransformSamples(samples_global, true);
  }
}  // namespace gpumap
