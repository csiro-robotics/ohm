// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas, Jason Williams
#include <ohm/DefaultLayer.h>
#include <ohm/Key.h>
#include <ohm/KeyHash.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/NdtMap.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Trace.h>
#include <ohm/VoxelData.h>

#include <ohmgpu/GpuCache.h>
#include <ohmgpu/GpuNdtMap.h>

#include <gtest/gtest.h>
#include "ohmtestcommon/CovarianceTestUtil.h"
#include "ohmtestcommon/OhmTestUtil.h"

#include <chrono>
#include <random>
#include <unordered_map>

#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

// Must come after glm includes due to usage on GPU.
#include <ohm/CovarianceVoxel.h>

#include <3esservermacros.h>

#include <Eigen/Dense>

using namespace ohm;
using namespace ohmtestutil;

namespace ndttests
{
void testNdtHits(const std::vector<glm::dvec3> &samples, double resolution)
{
  std::unordered_map<ohm::Key, CovTestVoxel, ohm::KeyHash> reference_voxels;

  ohm::OccupancyMap map(resolution, ohm::MapFlag::kVoxelMean);
  ohm::GpuNdtMap ndt(&map, true);

  // Simulate a sensor at the origin. Not used.
  const glm::dvec3 sensor(0.0);

  // Queue in rays.
  std::vector<glm::dvec3> rays(samples.size() * 2);
  for (size_t i = 0; i < samples.size(); ++i)
  {
    rays[i * 2 + 0] = sensor;
    rays[i * 2 + 1] = samples[i];

    // Update reference voxel
    ohm::Key key = map.voxelKey(samples[i]);
    auto ref = reference_voxels.find(key);
    if (ref == reference_voxels.end())
    {
      // New voxel. Initialise.
      CovTestVoxel ref_voxel;
      initialiseTestVoxel(&ref_voxel, float(ndt.map().resolution()));
      ref = reference_voxels.insert(std::make_pair(key, ref_voxel)).first;
    }

    // Update the reference algorithm.
    updateHit(&ref->second, samples[i]);
  }

  ndt.integrateRays(rays.data(), rays.size(), ohm::kRfExcludeRay);
  ndt.syncVoxels();

  // Validate
  int covariance_layer_index = -1;
  if (const ohm::MapLayer *layer = map.layout().layer(ohm::default_layer::covarianceLayerName()))
  {
    covariance_layer_index = layer->layerIndex();
  }

  ASSERT_GE(covariance_layer_index, 0);

  ohm::Voxel<const ohm::VoxelMean> mean(&map, map.layout().meanLayer());
  ohm::Voxel<const ohm::CovarianceVoxel> covariance(&map, map.layout().covarianceLayer());
  for (const auto &ref : reference_voxels)
  {
    ohm::setVoxelKey(ref.first, mean, covariance);
    ASSERT_TRUE(mean.isValid());
    ASSERT_TRUE(covariance.isValid());
    // Lookup the target voxel.
    ohm::VoxelMean voxel_mean;
    ohm::CovarianceVoxel cov_voxel;
    mean.read(&voxel_mean);
    covariance.read(&cov_voxel);
    EXPECT_TRUE(validate(ohm::positionSafe(mean), voxel_mean.count, cov_voxel, ref.second));
  }
}

void testNdtMiss(const glm::dvec3 &sensor, const std::vector<glm::dvec3> samples, double voxel_resolution,
                 float sensor_noise, const glm::dvec3 &map_origin, const std::vector<glm::dvec3> test_rays)
{
  (void)sensor;
  ohm::OccupancyMap map_cpu(voxel_resolution, ohm::MapFlag::kVoxelMean);
  ohm::NdtMap ndt_cpu(&map_cpu, true);

  map_cpu.setOrigin(map_origin);
  map_cpu.setMissProbability(0.45f);
  ndt_cpu.setSensorNoise(sensor_noise);

  // First process the samples
  ohm::Key target_key;
  for (size_t i = 0; i < samples.size(); ++i)
  {
    const glm::dvec3 &sample = samples[i];

    ohm::Key key = map_cpu.voxelKey(sample);
    target_key = key;
    ohm::integrateNdtHit(ndt_cpu, key, sample);
  }

  // Clone the map for use in GPU.
  ohm::GpuNdtMap ndt_gpu(map_cpu.clone(), false);
  // Copy parameterisation.
  ndt_gpu.setSensorNoise(ndt_cpu.sensorNoise());

  // Validate that our map objects differ.
  EXPECT_NE(&map_cpu, &ndt_gpu.map());

  // Fire rays to punch through the target voxel, and compare the delta one at a time.
  // In between we reset the probability value of the target voxel.
  ohm::Voxel<float> target_voxel_cpu(&map_cpu, map_cpu.layout().occupancyLayer(), target_key);
  ASSERT_TRUE(target_voxel_cpu.isValid());
  float initial_value;
  target_voxel_cpu.read(&initial_value);

  for (size_t i = 0; i < test_rays.size(); i += 2)
  {
    // Start the update in GPU (one ray -> inefficient).
    ndt_gpu.integrateRays(test_rays.data() + i, 2);

    ohm::integrateNdtMiss(ndt_cpu, target_key, test_rays[i], test_rays[i + 1]);

    // Sync from GPU.
    ndt_gpu.syncVoxels();
    // We will be updating on CPU. Invalidate the GPU cache.
    ndt_gpu.gpuCache()->clear();

    // Read the voxel value from GPU update.
    ohm::Voxel<float> target_voxel_gpu(&ndt_gpu.map(), ndt_gpu.map().layout().occupancyLayer(), target_key);
    ASSERT_TRUE(target_voxel_gpu.isValid());
    float ndt_cpu_value;
    target_voxel_cpu.read(&ndt_cpu_value);
    float ndt_gpu_value;
    target_voxel_gpu.read(&ndt_gpu_value);

    EXPECT_NEAR(ndt_gpu_value, ndt_cpu_value, 1e-4f);

    // Restore both voxel values.
    target_voxel_cpu.write(initial_value);
    target_voxel_gpu.write(initial_value);
  }
}


TEST(Ndt, Hit)
{
  // Use a fixed seed for test repeatability.
  uint32_t seed = 1153297050u;
  std::default_random_engine rng(seed);
  std::normal_distribution<double> gaussian(0.0, 1.0);
  // Generate points within a 2m cube.
  std::uniform_real_distribution<double> uniform(0.01, 1.99);
  const size_t sample_count = 10000;
  std::vector<glm::dvec3> samples;
  samples.reserve(sample_count);

  // First generate a sample surface to target. We have to make sure this falls within a single voxel.
  glm::dvec3 mean(0);
  glm::dmat3 cov(0);

  double num_pt = 0;
  while (num_pt < 4)
  {
    // glm::dvec3 pt = 0.5 * randomVector3();
    glm::dvec3 pt;

    pt[0] = uniform(rng);
    pt[1] = uniform(rng);
    pt[2] = uniform(rng);

    const glm::dvec3 diff = pt - mean;
    const double one_on_num_pt_plus_one = 1.0 / (num_pt + 1.0);
    mean = (num_pt * mean + pt) * one_on_num_pt_plus_one;
    cov = (num_pt * cov + glm::dot((num_pt * one_on_num_pt_plus_one) * diff, diff)) * one_on_num_pt_plus_one;
    num_pt += 1.0;
  }

  {
    // TODO: this is the only part we need Eigen for. Find an alternative Eigen decomposition solution.
    Eigen::Matrix3d cov_eigen;
    cov_eigen << glm::row(cov, 0)[0], glm::row(cov, 0)[1], glm::row(cov, 0)[2],  //
      glm::row(cov, 1)[0], glm::row(cov, 1)[1], glm::row(cov, 1)[1],             //
      glm::row(cov, 2)[0], glm::row(cov, 2)[1], glm::row(cov, 2)[1];
    Eigen::LDLT<Eigen::Matrix3d, Eigen::Lower> cov_d(cov_eigen);
    const Eigen::Vector3d &vD = cov_d.vectorD();

    const Eigen::Vector3d mean_eigen(mean.x, mean.y, mean.z);
    for (size_t i = 0; i < sample_count; ++i)
    {
      Eigen::Vector3d pt;
      for (size_t i = 0; i < 3; ++i)
      {
        pt[i] = sqrt(vD[i]) * gaussian(rng);
      }

      pt = mean_eigen + cov_d.matrixL() * pt;
      samples.emplace_back(glm::dvec3(pt.x(), pt.y(), pt.z()));
    }
  }

  testNdtHits(samples, 2.0);
}


// For the miss tests, we setup a GPU and a CPU based NDT map, populate one voxel and compare the miss results on one
// focus voxel. On CPU we only interate misses for that voxel, for GPU we do the full test rays and compare the
// results on the focus voxel.

TEST(Ndt, MissPlanar)
{
  uint32_t seed = 1153297050u;
  std::default_random_engine rng(seed);
  // Generate points within a 2m cube.
  const double voxel_resolution = 2.0;
  std::uniform_real_distribution<double> uniform(0.01, 1.99);
  const size_t sample_count = 10000;
  std::vector<glm::dvec3> samples;
  samples.reserve(sample_count);

  // Create samples which populate a single voxel and define a plane.
  const glm::dvec3 sensor(1, 1, 5);
  for (size_t i = 0; i < sample_count; ++i)
  {
    glm::dvec3 sample;
    sample.x = uniform(rng);
    sample.y = uniform(rng);
    sample.z = 1.0;
    // sample.z += 0.025 * uniform(rng);
    samples.emplace_back(sample);
  }

  // Build a set of rays to punch through the target voxel.
  std::vector<glm::dvec3> rays;
  // Ray straight down through the voxel
  rays.emplace_back(sensor);
  rays.emplace_back(glm::dvec3(1, 1, -5));
  // Reverse the ray above
  rays.emplace_back(rays[1]);
  rays.emplace_back(rays[0]);
  // Ray parallel to the voxel ellipsoid - expected to miss.
  rays.emplace_back(glm::dvec3(-5, 1, 0.25));
  rays.emplace_back(glm::dvec3(5, 1, 0.25));
  // Ray parallel to the voxel ellipsoid, but near the centre.
  // Note: we build a perfect plane above in the centre of the voxel. However, the quantisation of the mean
  // can offset the plane. Running exactly on the plane on which we generated the points will not result in the same
  // effect in testing. We must offset the ray a little to ensure we are a bit off the plane either when using
  // voxel mean quantisation or not.
  rays.emplace_back(glm::dvec3(1, 5, 1.01));
  rays.emplace_back(glm::dvec3(1, -5, 1.01));
  // Ray running across the voxel, down towards the ellipsoid, but not crossing. This simulates rays running near
  // parallel a ground plane as it approaches the sample position.
  rays.emplace_back(glm::dvec3(-5, 1, 2));
  rays.emplace_back(glm::dvec3(5, 1, 1));
  // Ray running across the voxel, and through the ellipsoid.
  rays.emplace_back(glm::dvec3(-5, 1, 2));
  rays.emplace_back(glm::dvec3(5, 1, 0.5));

  testNdtMiss(sensor, samples, voxel_resolution, 0.05f, glm::dvec3(0), rays);
}


TEST(Ndt, MissCylindrical)
{
  // Generate points within a 2m cube.
  const double voxel_resolution = 2.0;
  const float sensor_noise = 0.05f;
  const glm::dvec3 map_origin(-0.5 * voxel_resolution);

  const double cylinder_radius = 0.3;
  uint32_t seed = 1153297050u;
  std::default_random_engine rng(seed);
  std::uniform_real_distribution<double> uniform(-0.99, 0.99);
  std::uniform_real_distribution<double> uniform_radius(cylinder_radius - sensor_noise, cylinder_radius + sensor_noise);
  const size_t sample_count = 10000;
  std::vector<glm::dvec3> samples;
  samples.reserve(sample_count);

  // Create samples which populate a single voxel and define a plane.
  const glm::dvec3 sensor(0, 0, 5);

  for (size_t i = 0; i < sample_count; ++i)
  {
    glm::dvec3 sample;
    // Create a random sample in the voxel
    sample.x = uniform(rng);
    sample.y = uniform(rng);
    sample.z = uniform(rng);
    // Convert into a cylinder.
    // Normalise in X/Y, create a random radius (to the limit) and scale by this radius.
    double length_xy = std::sqrt(sample.x * sample.x + sample.y * sample.y);
    if (length_xy > 1e-6)
    {
      const double radius = uniform_radius(rng);
      sample.x = radius * sample.x / length_xy;
      sample.y = radius * sample.y / length_xy;
    }
    samples.emplace_back(sample);
  }

  // Build a set of rays to punch through the target voxel.
  std::vector<glm::dvec3> rays;
  // // Ray straight down through the voxel
  rays.emplace_back(sensor);
  rays.emplace_back(glm::dvec3(0, 0, -5));
  // Reverse the ray above
  rays.emplace_back(rays[1]);
  rays.emplace_back(rays[0]);
  // Ray running parallel to the cylinder near the edge. Should be a near hit.
  rays.emplace_back(glm::dvec3(cylinder_radius, cylinder_radius, 5));
  rays.emplace_back(glm::dvec3(cylinder_radius, cylinder_radius, -5));
  // Ray running parallel to the cylinder, but should miss.
  rays.emplace_back(glm::dvec3(1.5 * cylinder_radius, 1.5 * cylinder_radius, -5));
  rays.emplace_back(glm::dvec3(2.0 * cylinder_radius, 2.0 * cylinder_radius, 5));
  // Ray across the middle of the cylinder.
  rays.emplace_back(glm::dvec3(2, -cylinder_radius, 0));
  rays.emplace_back(glm::dvec3(-2, -cylinder_radius, 0));
  // Ray across the end (top) of the cylinder.
  rays.emplace_back(glm::dvec3(2, -cylinder_radius, 0.85));
  rays.emplace_back(glm::dvec3(-2, -cylinder_radius, 0.85));
  // Ray across the voxel, missing the cylinder (top) of the cylinder.
  rays.emplace_back(glm::dvec3(2, 2.0 * cylinder_radius, 0.85));
  rays.emplace_back(glm::dvec3(-2, 2.0 * cylinder_radius, 0.85));

  testNdtMiss(sensor, samples, voxel_resolution, sensor_noise, map_origin, rays);
}


TEST(Ndt, MissSpherical)
{
  // Generate points within a 2m cube.
  const double voxel_resolution = 2.0;
  const float sensor_noise = 0.05f;
  const glm::dvec3 map_origin(-0.5 * voxel_resolution);

  const double radius = 0.3;
  uint32_t seed = 1153297050u;
  std::default_random_engine rng(seed);
  std::uniform_real_distribution<double> uniform(-0.99, 0.99);
  std::uniform_real_distribution<double> uniform_radius(radius - sensor_noise, radius + sensor_noise);
  const size_t sample_count = 10000;
  std::vector<glm::dvec3> samples;
  samples.reserve(sample_count);

  // Create samples which populate a single voxel and define a plane.
  const glm::dvec3 sensor(0, 0, 5);
  for (size_t i = 0; i < sample_count; ++i)
  {
    glm::dvec3 sample;
    double sample_len2 = 0;
    // Keep trying while we have a near zero vector.
    while (sample_len2 < 1e-6)
    {
      // Create a random sample in the voxel
      sample.x = uniform(rng);
      sample.y = uniform(rng);
      sample.z = uniform(rng);
      sample_len2 = glm::length2(sample);
    }
    sample = glm::normalize(sample);
    sample *= uniform_radius(rng);
    samples.emplace_back(sample);
  }

  // Now trace a ray through the voxel, but not ending in the voxel.
  std::vector<glm::dvec3> rays;
  // // Ray straight down through the voxel
  rays.emplace_back(sensor);
  rays.emplace_back(glm::dvec3(0, 0, -5));
  // Reverse the ray above
  rays.emplace_back(rays[1]);
  rays.emplace_back(rays[0]);
  // Edge of the sphere..
  rays.emplace_back(glm::dvec3(radius, radius, 5));
  rays.emplace_back(glm::dvec3(radius, radius, -5));
  // Near the edge of the sphere, but should miss.
  rays.emplace_back(glm::dvec3(1.5 * radius, 1.5 * radius, -5));
  rays.emplace_back(glm::dvec3(2.0 * radius, 2.0 * radius, 5));

  testNdtMiss(sensor, samples, voxel_resolution, sensor_noise, map_origin, rays);
}
}  // namespace ndttests
