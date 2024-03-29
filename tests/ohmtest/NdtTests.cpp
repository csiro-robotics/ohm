// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas, Jason Williams
#include "OhmTestConfig.h"

#include <ohm/Key.h>
#include <ohm/NdtMap.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Trace.h>
#include <ohm/VoxelData.h>

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

#include <3esservermacros.h>

#include <Eigen/Dense>

using namespace ohm;
using namespace ohmtestutil;

namespace ndttests
{
/// Default tolerance for checking NDT probability changes.
///
/// We use random generators with fixed seeds to create our ray set for each run. This is deterministic for a given
/// platform, but not beteen platforms as the random generator code may differ. We set a tight, default tolerance,
/// but may tune larger values for specific cases.
const float default_ndt_tolerance = 0.001f;

/// Helper for testing covariance update via @p NdtMap .
///
/// We add each sample to the map as if the sensor were at the origin. The changes to the target voxels are
/// compared against a reference code implementation. Samples may affect any number of voxels.
///
/// @param samples The set of sample points to integrate into the map.
/// @param voxel_resolution The voxel size for the @c OccupancyMap
void testNdtHits(const std::vector<glm::dvec3> &samples, const std::vector<glm::dvec3> &sensors, double resolution)
{
  std::unordered_map<ohm::Key, CovTestVoxel> reference_voxels;

  ohm::OccupancyMap map(resolution, ohm::MapFlag::kVoxelMean);
  ohm::NdtMap ndt(&map, true);

  // Simulate a sensor at the origin. Not used.
  const glm::dvec3 sensor(0.0);

  ohm::Voxel<const ohm::VoxelMean> mean(&map, map.layout().meanLayer());
  ohm::Voxel<const ohm::CovarianceVoxel> covariance(&map, map.layout().covarianceLayer());
  ASSERT_TRUE(mean.isLayerValid());
  ASSERT_TRUE(covariance.isLayerValid());

  // Build a a set of samples into the NDT map and into a reference data set.
  for (size_t i = 0; i < samples.size(); ++i)
  {
    const glm::dvec3 &sample = samples[i];
    const glm::dvec3 &sensor = sensors[i];

    ohm::Key key = map.voxelKey(sample);

    ohm::integrateNdtHit(ndt, key, sensor, sample);
    setVoxelKey(key, mean, covariance);

    ASSERT_TRUE(mean.isValid());
    ASSERT_TRUE(covariance.isValid());
    ohm::CovarianceVoxel cov_voxel;
    ohm::VoxelMean voxel_mean;
    covariance.read(&cov_voxel);
    mean.read(&voxel_mean);

    // Update the reference voxel as well.
    auto ref = reference_voxels.find(key);
    if (ref == reference_voxels.end())
    {
      // New voxel. Initialise.
      CovTestVoxel ref_voxel;
      initialiseTestVoxel(&ref_voxel, float(ndt.map().resolution()));
      ref = reference_voxels.insert(std::make_pair(key, ref_voxel)).first;
    }

    // Update the reference algorithm.
    updateHit(&ref->second, sample);

    // Progressive validation
    EXPECT_TRUE(validate(positionSafe(mean), voxel_mean.count, cov_voxel, ref->second));
  }

  // Finalise validation
  // Reset the voxel references.
  mean = ohm::Voxel<const ohm::VoxelMean>(&map, map.layout().meanLayer());
  covariance = ohm::Voxel<const ohm::CovarianceVoxel>(&map, map.layout().covarianceLayer());
  for (const auto &ref : reference_voxels)
  {
    // Lookup the target voxel.
    setVoxelKey(ref.first, mean, covariance);
    ASSERT_TRUE(mean.isValid());
    ASSERT_TRUE(covariance.isValid());
    ohm::CovarianceVoxel cov_voxel;
    ohm::VoxelMean voxel_mean;
    covariance.read(&cov_voxel);
    mean.read(&voxel_mean);
    EXPECT_TRUE(validate(positionSafe(mean), voxel_mean.count, cov_voxel, ref.second));
  }

  mean.reset();
  covariance.reset();
}

/// Helper for testing NDT based voxel miss logic.
///
/// We first build an @c NdtMap with the given set of @p samples . While we allow for these to be traced from
/// @p sensor only modify the voxel for each sample rather than integrating the full ray into the map. We assume that
/// all @p samples fall with in the same voxel.
///
/// Once the sample voxel is populated, we trace all the @p test_rays through that voxel and compare the *change* in
/// probability against the @p expected_prob_and_tolerance .
///
/// @param sensor The simulated sensor position for all the @p samples .
/// @param samples The set of sample points to integrate into the map. These should all fall in the same voxel.
/// @param voxel_resolution The voxel size for the @c OccupancyMap
/// @param sensor_noise Sensor noise value for the @c NdtMap
/// @param map_origin Origin for the @p OccupancyMap
/// @param test_rays Set of rays to test against the sample voxel. These are expected to all pass through, but not
///   end in the sample voxel. There are two elements per ray in this array making ray origin, end pairs.
/// @param expected_prob_and_tolerance Expected probability adjustments for each of the @p test_rays . This array has
///   half the elements of @p test_rays .
void testNdtMiss(const glm::dvec3 &sensor, const std::vector<glm::dvec3> samples, double voxel_resolution,
                 float sensor_noise, const glm::dvec3 &map_origin, const std::vector<glm::dvec3> &test_rays,
                 const std::vector<std::pair<float, float>> &expected_prob_and_tolerance)
{
  (void)sensor;  // Only used for 3es debugging.
  ohm::OccupancyMap map(voxel_resolution, ohm::MapFlag::kVoxelMean);
  ohm::NdtMap ndt(&map, true);

  map.setOrigin(map_origin);
  // Set explicit probabilities
  map.setHitProbability(0.55f);
  map.setMissProbability(0.45f);
  ndt.setAdaptationRate(1.0f);
  ndt.setSensorNoise(sensor_noise);

  // Integrate all the samples into the map to build voxel covariance and mean. We expect all samples to fall in one
  // voxel and test rays to pass through this voxel. While this assumption is not enforced by the integrateHit() calls
  // below, but we do cache the corresponding voxel key in target_key.
  ohm::Key target_key;
  TES_STMT(std::vector<tes::Vector3d> lines);
  for (size_t i = 0; i < samples.size(); ++i)
  {
    const glm::dvec3 &sample = samples[i];

    ohm::Key key = map.voxelKey(sample);
    target_key = key;
    ohm::integrateNdtHit(ndt, key, sensor, sample);
    TES_LINE(ohm::g_tes, TES_COLOUR(Yellow), tes::Id(), glm::value_ptr(sensor), glm::value_ptr(sample));
    TES_STMT(lines.emplace_back(tes::Vector3d(glm::value_ptr(sensor))));
    TES_STMT(lines.emplace_back(tes::Vector3d(glm::value_ptr(sample))));
  }
  TES_LINES(ohm::g_tes, TES_COLOUR(Yellow), tes::Id(), tes::DataBuffer(lines));
  TES_SERVER_UPDATE(ohm::g_tes, 0.0f);

  // Fetch the target_voxel in which we expect all samples to fall.
  Voxel<float> occupancy(&map, map.layout().occupancyLayer(), target_key);
  ASSERT_TRUE(occupancy.isValid());
  float initial_value;
  occupancy.read(&initial_value);
  ndt.setTrace(true);  // For 3es debugging

  // Now trace all the test_rays through the target_voxel. For each we will restore the initial voxel value, so
  // each test is independent. We then integrate the ray miss/passthrough and validate the probability adjustment
  // against the expected result.
  for (size_t i = 0; i < test_rays.size(); i += 2)
  {
    occupancy.write(initial_value);
    TES_LINE(ohm::g_tes, TES_COLOUR(Cornsilk), tes::Id(&sensor), glm::value_ptr(test_rays[i]),
             glm::value_ptr(test_rays[i + 1]));
    TES_SERVER_UPDATE(ohm::g_tes, 0.0f);
    ohm::integrateNdtMiss(ndt, target_key, test_rays[i], test_rays[i + 1]);
    TES_LINES_END(ohm::g_tes, tes::Id(&sensor));
    // Calculate the value adjustment.
    float adjusted_value;
    occupancy.read(&adjusted_value);
    float value_adjustment = adjusted_value - initial_value;
    // Convert to probability.
    float ray_probability = ohm::valueToProbability(value_adjustment);
    // Validate
    // Note: there is some variance here across platforms as the random generators differ even though we use
    // a fixed seed.
    EXPECT_NEAR(ray_probability, expected_prob_and_tolerance[i / 2].first, expected_prob_and_tolerance[i / 2].second);
  }
  occupancy.reset();
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
  std::vector<glm::dvec3> sensors;
  sensors.reserve(sample_count);

  // First generate a sample surface to target. We have to make sure this falls within a single voxel.
  glm::dvec3 mean(0);
  glm::dmat3 cov(0);

  // Sensor position from which measurements were taken
  glm::dvec3 sensor(0);

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
      sensors.push_back(sensor);
    }
  }

  testNdtHits(samples, sensors, 2.0);
}


TEST(Ndt, MissPlanar)
{
  ohm::Trace trace("ndt-miss-planar.3es");

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

  // Setup to trace a ray through the voxel, but not ending in the voxel.
  std::vector<std::pair<float, float>> expected_prob_and_tolerance;
  std::vector<glm::dvec3> rays;
  // Ray straight down through the voxel
  rays.emplace_back(sensor);
  rays.emplace_back(glm::dvec3(1, 1, -5));
  expected_prob_and_tolerance.emplace_back(0.004f, default_ndt_tolerance);
  // Reverse the ray above
  rays.emplace_back(rays[1]);
  rays.emplace_back(rays[0]);
  expected_prob_and_tolerance.emplace_back(0.004f, default_ndt_tolerance);
  // Ray parallel to the voxel ellipsoid - expected to miss.
  rays.emplace_back(glm::dvec3(-5, 1, 0.25));
  rays.emplace_back(glm::dvec3(5, 1, 0.25));
  expected_prob_and_tolerance.emplace_back(0.5f, default_ndt_tolerance);
  // Ray parallel to the voxel ellipsoid, but near the centre.
  // Note: we build a perfect plane above in the centre of the voxel. However, the quantisation of the mean
  // can offset the plane. Running exactly on the plane on which we generated the points will not result in the same
  // effect in testing. We must offset the ray a little to ensure we are a bit off the plane either when using
  // voxel mean quantisation or not.
  rays.emplace_back(glm::dvec3(1, 5, 1.01));
  rays.emplace_back(glm::dvec3(1, -5, 1.01));
  expected_prob_and_tolerance.emplace_back(0.5f, default_ndt_tolerance);
  // Ray running across the voxel, down towards the ellipsoid, but not crossing. This simulates rays running near
  // parallel a ground plane as it approaches the sample position.
  rays.emplace_back(glm::dvec3(-5, 1, 2));
  rays.emplace_back(glm::dvec3(5, 1, 1));
  expected_prob_and_tolerance.emplace_back(0.5f, default_ndt_tolerance);
  // Ray running across the voxel, and through the ellipsoid.
  rays.emplace_back(glm::dvec3(-5, 1, 2));
  rays.emplace_back(glm::dvec3(5, 1, 0.5));
  expected_prob_and_tolerance.emplace_back(0.23f, 0.02f);

  testNdtMiss(sensor, samples, voxel_resolution, 0.05f, glm::dvec3(0), rays, expected_prob_and_tolerance);
}


TEST(Ndt, MissCylindrical)
{
  ohm::Trace trace("ndt-miss-cylindrical.3es");

  // Generate points within a 2m cube.
  const double voxel_resolution = 2.0;
  const float sensor_noise = 0.05f;

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

  // Setup rays through the voxel, but not ending in the voxel.
  std::vector<std::pair<float, float>> expected_prob_and_tolerance;
  std::vector<glm::dvec3> rays;
  // Ray straight down through the voxel
  rays.emplace_back(sensor);
  rays.emplace_back(glm::dvec3(0, 0, -5));
  expected_prob_and_tolerance.emplace_back(0.004f, default_ndt_tolerance);
  // Reverse the ray above
  rays.emplace_back(rays[1]);
  rays.emplace_back(rays[0]);
  expected_prob_and_tolerance.emplace_back(0.004f, default_ndt_tolerance);
  // Ray running parallel to the cylinder near the edge. Should be a near hit.
  rays.emplace_back(glm::dvec3(cylinder_radius, cylinder_radius, 5));
  rays.emplace_back(glm::dvec3(cylinder_radius, cylinder_radius, -5));
  expected_prob_and_tolerance.emplace_back(0.425f, 0.01f);
  // Ray running parallel to the cylinder, but should miss.
  rays.emplace_back(glm::dvec3(1.5 * cylinder_radius, 1.5 * cylinder_radius, -5));
  rays.emplace_back(glm::dvec3(2.0 * cylinder_radius, 2.0 * cylinder_radius, 5));
  expected_prob_and_tolerance.emplace_back(0.499f, default_ndt_tolerance);
  // Ray across the middle of the cylinder.
  rays.emplace_back(glm::dvec3(2, -cylinder_radius, 0));
  rays.emplace_back(glm::dvec3(-2, -cylinder_radius, 0));
  expected_prob_and_tolerance.emplace_back(0.312f, 0.005f);
  // Ray across the end (top) of the cylinder.
  rays.emplace_back(glm::dvec3(2, -cylinder_radius, 0.85));
  rays.emplace_back(glm::dvec3(-2, -cylinder_radius, 0.85));
  expected_prob_and_tolerance.emplace_back(0.436f, 0.003f);
  // Ray across the voxel, missing the cylinder (top) of the cylinder.
  rays.emplace_back(glm::dvec3(2, 2.0 * cylinder_radius, 0.85));
  rays.emplace_back(glm::dvec3(-2, 2.0 * cylinder_radius, 0.85));
  expected_prob_and_tolerance.emplace_back(0.497f, default_ndt_tolerance);
  testNdtMiss(sensor, samples, voxel_resolution, sensor_noise, glm::dvec3(-0.5 * voxel_resolution), rays,
              expected_prob_and_tolerance);
}


TEST(Ndt, MissSpherical)
{
  ohm::Trace trace("ndt-miss-spherical.3es");

  // Generate points within a 2m cube.
  const double voxel_resolution = 2.0;
  const float sensor_noise = 0.05f;

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

  // Setup rays through the voxel, but not ending in the voxel.
  std::vector<std::pair<float, float>> expected_prob_and_tolerance;
  std::vector<glm::dvec3> rays;
  // Ray straight down through the voxel
  rays.emplace_back(sensor);
  rays.emplace_back(glm::dvec3(0, 0, -5));
  expected_prob_and_tolerance.emplace_back(0.004f, default_ndt_tolerance);
  // Reverse the ray above
  rays.emplace_back(rays[1]);
  rays.emplace_back(rays[0]);
  expected_prob_and_tolerance.emplace_back(0.004f, default_ndt_tolerance);
  // Edge of the sphere..
  rays.emplace_back(glm::dvec3(radius, radius, 5));
  rays.emplace_back(glm::dvec3(radius, radius, -5));
  expected_prob_and_tolerance.emplace_back(0.469f, 0.006f);
  // Near the edge of the sphere, but should miss.
  rays.emplace_back(glm::dvec3(1.5 * radius, 1.5 * radius, -5));
  rays.emplace_back(glm::dvec3(2.0 * radius, 2.0 * radius, 5));
  expected_prob_and_tolerance.emplace_back(0.5f, default_ndt_tolerance);
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

  testNdtMiss(sensor, samples, voxel_resolution, sensor_noise, glm::dvec3(-0.5 * voxel_resolution), rays,
              expected_prob_and_tolerance);
}
}  // namespace ndttests
