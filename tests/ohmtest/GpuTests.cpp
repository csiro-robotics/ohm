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

namespace gpumap
{
  void gpuTransformSamples(const std::vector<glm::dvec3> &samples_global)
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
    double interpolation_step = 1.0 / (transforms_count - 1);
    for (unsigned i = 0; i < transforms_count; ++i)
    {
      timestamps[i] = timestamp;
      translations[i] = start_point + (i * interpolation_step) * (end_point - start_point);
      rotations[i] = start_rotation * glm::slerp(start_rotation, end_rotation, i * interpolation_step);
      timestamp += time_increment;
    }

    std::vector<double> sample_times(samples_global.size());
    timestamp = base_time + time_increment;  // Reset timestamp for samples.
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
      // Find the appropriate time indices.
      while (timestamps[tidx] < sample_times[i])
      {
        ++tidx;
        ASSERT_LT(tidx + 1, timestamps.size()) << "out of bounds";
      }

      lerp = (sample_times[i] - timestamps[tidx]) / (timestamps[tidx + 1] - timestamps[tidx]);

      translation = translations[tidx] + lerp + (translations[tidx + 1] - translations[tidx]);
      rotation = rotations[tidx] * glm::slerp(rotations[tidx], rotations[tidx + 1], lerp);

      p = rotation * samples_global[i] + translation;
      samples_local[i] = p;
    }

    // Do the GPU transformation.
    gputil::Device gpu = ohm::gpuDevice();
    GpuTransformSamples transformation(gpu);

    gputil::Buffer output_buffer(gpu, 0, gputil::kBfReadWriteHost);
    gputil::Event completion_event;
    int err = transformation.transform(timestamps.data(), translations.data(), rotations.data(), transforms_count,
                                       sample_times.data(), samples_local.data(), unsigned(samples_local.size()),
                                       gpu.defaultQueue(), output_buffer, completion_event);

    ASSERT_EQ(err, 0);

    completion_event.wait();

    // Read back the results.
    ASSERT_GE(output_buffer.size(), sizeof(gputil::float3) * 2 * samples_local.size());
    // Results are rays: sensor origin, global sample
    std::vector<glm::dvec3> rays(2 * samples_local.size());
    gputil::PinnedBuffer rays_buffer(output_buffer, gputil::kPinRead);
    rays_buffer.readElements<gputil::float3>(rays.data(), 2 * samples_local.size());
    rays_buffer.unpin();

    // Validate results.
    glm::dvec3 sample, expect;
    glm::dvec3 diff;
    for (size_t i = 0; i < samples_global.size(); ++i)
    {
      expect = samples_global[i];
      sample = rays[i * 2 + 1];
      diff = expect - sample;
      EXPECT_NEAR(glm::length(diff), 0.0, 1e-6);
    }
  }


  TEST(Gpu, TransformSample)
  {
    // Just transform a single sample for this test.
    gpuTransformSamples({ glm::dvec3(10.0f, 20.0f, 15.0f) });
  }


  TEST(Gpu, TransformSamples)
  {
    // Just transform a single sample for this test.
    std::vector<glm::dvec3> samples_global;
    std::mt19937 rand_engine;
    std::uniform_real_distribution<double> rand(-10.0f, 25.0f);
    const unsigned sample_count = 4 * 1024u;

    samples_global.reserve(sample_count);
    while (samples_global.size() < sample_count)
    {
      samples_global.emplace_back(glm::dvec3(rand(rand_engine), rand(rand_engine), rand(rand_engine)));
    }


    gpuTransformSamples(samples_global);
  }
}  // namespace gpumap
