// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas, Jason Williams
#include "OhmTestConfig.h"

#include <ohm/Key.h>
#include <ohm/KeyHash.h>
#include <ohm/MapCache.h>
#include <ohm/NdtMap.h>
#include <ohm/NdtVoxel.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Trace.h>
#include <ohm/VoxelMean.h>

#include <gtest/gtest.h>
#include "ohmtestcommon/OhmTestUtil.h"

#include <chrono>
#include <random>
#include <unordered_map>

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <3esservermacros.h>

#include <Eigen/Dense>

using namespace ohm;

namespace ndttests
{
  // The reference code below was provided by Jason Williams as a guide to implementing the NDT map epsecially with
  // consideration to using a packed covariance matrix. We use this code to validate the NDT results.
  // Note: Jason's code was written using Eigen for mathematical operations which we've converted to glm. The ohm
  // library prefers glm as it is a much ligher weight library and we do not require Eigen's advanced functionality.


  /// Ndt reference data.
  struct NdtTestVoxel : public ohm::NdtVoxel
  {
    double mean[3];
    unsigned point_count;
  };


  void initialiseTestVoxel(NdtTestVoxel *ref_voxel, double sensor_noise)
  {
    initialiseNdt(ref_voxel, sensor_noise);
    ref_voxel->mean[0] = ref_voxel->mean[1] = ref_voxel->mean[2] = 0;
    ref_voxel->point_count = 0;
  }

  // dot product of j-th and k-th columns of A
  // A is (4,3), assumed to be packed as follows, where z is non-represented zero
  // 0 1 3
  // z 2 4
  // z z 5
  // 6 7 8
  double packed_dot(const double *A, const size_t j, const size_t k)
  {
    const static size_t col_first_el[] = { 0, 1, 3 };
    double d = A[6 + k] * A[6 + j];
    const size_t indj = col_first_el[j], indk = col_first_el[k], m = std::min(j, k);
    for (size_t i = 0; i <= m; ++i)
    {
      d += A[indj + i] * A[indk + i];
    }
    return d;
  }

  void updateHit(NdtTestVoxel *ndt, const glm::dvec3 &sample)
  {
    const double num_pt = double(ndt->point_count);
    const double one_on_num_pt_plus_one = 1.0 / (num_pt + 1.0);
    glm::dvec3 mean(ndt->mean[0], ndt->mean[1], ndt->mean[2]);
    const glm::dvec3 diff = sample - mean;
    const double sc_1 = num_pt ? std::sqrt(num_pt * one_on_num_pt_plus_one) : 1;
    const double sc_2 = one_on_num_pt_plus_one * std::sqrt(num_pt);
    std::vector<double> A(9);
    for (size_t i = 0; i < 6; ++i)
    {
      A[i] = sc_1 * ndt->cov_sqrt_diag[i];
    }
    for (size_t i = 0; i < 3; ++i)
    {
      A[i + 6] = sc_2 * diff[i];
    }
    for (size_t k = 0; k < 3; ++k)
    {
      const size_t ind1 = (k * (k + 3)) >> 1,  // packed index of (k,k) term
        indk = ind1 - k;                       // packed index of (1,k)
      const double ak = std::sqrt(packed_dot(&A[0], k, k));
      ndt->cov_sqrt_diag[ind1] = ak;
      if (ak > 0.0)
      {
        const double aki = 1.0 / ak;
        for (size_t j = k + 1; j < 3; ++j)
        {
          const size_t indj = (j * (j + 1)) >> 1, indkj = indj + k;
          double c = packed_dot(&A[0], j, k) * aki;
          ndt->cov_sqrt_diag[indkj] = float(c);
          c *= aki;
          A[j + 6] -= c * A[k + 6];
          for (size_t l = 0; l <= k; ++l)
          {
            A[indj + l] -= c * A[indk + l];
          }
        }
      }
    }

    mean = (num_pt * mean + sample) * one_on_num_pt_plus_one;
    ndt->mean[0] = mean[0];
    ndt->mean[1] = mean[1];
    ndt->mean[2] = mean[2];
    ++ndt->point_count;
  }

  void validate(const glm::dvec3 &mean, unsigned point_count, const ohm::NdtVoxel &ndt, const NdtTestVoxel &ref)
  {
    // Quantisation in the mean storage create more signficant absolute errors in the covariance and mean.
    const double epsilon_cov = 1e-2;
    const double epsilon_mean = 1e-1;
    for (int i = 0; i < 6; ++i)
    {
      EXPECT_NEAR(ndt.cov_sqrt_diag[i], ref.cov_sqrt_diag[i], epsilon_cov);
    }

    EXPECT_EQ(point_count, ref.point_count);

    const glm::dvec3 ref_mean(ref.mean[0], ref.mean[1], ref.mean[2]);
    const double mean_diff = glm::length(mean - ref_mean);
    EXPECT_NEAR(mean_diff, 0.0, epsilon_mean);
  }

  void testNdtHits(const std::vector<glm::dvec3> &samples, double resolution)
  {
    std::unordered_map<ohm::Key, NdtTestVoxel, ohm::KeyHash> reference_voxels;

    ohm::OccupancyMap map(resolution, ohm::MapFlag::kVoxelMean);
    ohm::NdtMap ndt(&map, true);
    ohm::MapCache cache;

    // Simulate a sensor at the origin. Not used.
    const glm::dvec3 sensor(0.0);

    // First process the
    for (size_t i = 0; i < samples.size(); ++i)
    {
      const glm::dvec3 &sample = samples[i];

      ohm::Key key = map.voxelKey(sample);
      ohm::Voxel voxel = map.voxel(key, true, &cache);
      ndt.integrateHit(voxel, sensor, sample);
      const ohm::NdtVoxel &ndt_voxel = *voxel.layerContent<const ohm::NdtVoxel *>(ndt.covarianceLayerIndex());
      const ohm::VoxelMean &voxel_mean = *voxel.layerContent<const ohm::VoxelMean *>(ndt.map().layout().meanLayer());

      // Update the reference voxel as well.
      auto ref = reference_voxels.find(key);
      if (ref == reference_voxels.end())
      {
        // New voxel. Initialise.
        NdtTestVoxel ref_voxel;
        initialiseTestVoxel(&ref_voxel, ndt.sensorNoise());
        ref = reference_voxels.insert(std::make_pair(key, ref_voxel)).first;
      }

      // Update the reference algorithm.
      updateHit(&ref->second, sample);

      // Progressive validation
      validate(voxel.position(), voxel_mean.count, ndt_voxel, ref->second);
    }

    // Finalise validation
    for (const auto &ref : reference_voxels)
    {
      // Lookup the target voxel.
      ohm::VoxelConst voxel = const_cast<const ohm::OccupancyMap &>(map).voxel(ref.first, &cache);
      const ohm::NdtVoxel &ndt_voxel = *voxel.layerContent<const ohm::NdtVoxel *>(ndt.covarianceLayerIndex());
      const ohm::VoxelMean &voxel_mean = *voxel.layerContent<const ohm::VoxelMean *>(ndt.map().layout().meanLayer());
      validate(voxel.position(), voxel_mean.count, ndt_voxel, ref.second);
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


  TEST(Ndt, MissPlanar)
  {
    ohm::Trace trace("ndt-miss-planar.3es");

    uint32_t seed = 1153297050u;
    std::default_random_engine rng(seed);
    // Generate points within a 2m cube.
    const double voxel_resolution = 2.0;
    std::uniform_real_distribution<double> uniform(0.01, 1.99);
    const size_t sample_count = 10000;
    std::vector<glm::dvec3> vertices;
    vertices.reserve(sample_count);


    // Create samples which populate a single voxel and define a plane.

    for (size_t i = 0; i < sample_count; ++i)
    {
      glm::dvec3 sample;
      sample.x = uniform(rng);
      sample.y = uniform(rng);
      sample.z = 1.0;
      // sample.z += 0.025 * uniform(rng);
      vertices.emplace_back(sample);
    }

    ohm::OccupancyMap map(voxel_resolution, ohm::MapFlag::kVoxelMean);
    ohm::NdtMap ndt(&map, true);
    ohm::MapCache cache;

    map.setMissProbability(0.45f);
    ndt.setSensorNoise(0.05f);

    // Simulate a sensor at the origin. Not used.
    const glm::dvec3 sensor(1, 1, 5);

    // First process the
    ohm::Key target_key;
    TES_STMT(std::vector<glm::vec3> lines);
    for (size_t i = 0; i < vertices.size(); ++i)
    {
      const glm::dvec3 &sample = vertices[i];

      ohm::Key key = map.voxelKey(sample);
      target_key = key;
      ohm::Voxel voxel = map.voxel(key, true, &cache);
      ndt.integrateHit(voxel, sensor, sample);
      TES_LINE(ohm::g_3es, TES_COLOUR(Yellow), glm::value_ptr(sensor), glm::value_ptr(sample));
      TES_STMT(lines.emplace_back(glm::vec3(sensor)));
      TES_STMT(lines.emplace_back(glm::vec3(sample)));
    }
    TES_LINES(ohm::g_3es, TES_COLOUR(Yellow), glm::value_ptr(*lines.data()), lines.size(), sizeof(*lines.data()));
    TES_SERVER_UPDATE(ohm::g_3es, 0.0f);

    // Now trace a ray through the voxel, but not ending in the voxel.
    std::vector<float> expected_prob_approx;
    vertices.clear();
    // Ray straight down through the voxel
    vertices.emplace_back(sensor);
    vertices.emplace_back(glm::dvec3(1, 1, -5));
    expected_prob_approx.emplace_back(0.05f);
    // Reverse the ray above
    vertices.emplace_back(vertices[1]);
    vertices.emplace_back(vertices[0]);
    expected_prob_approx.emplace_back(0.05f);
    // Ray parallel to the voxel ellipsoid - expected to miss.
    vertices.emplace_back(glm::dvec3(-5, 1, 0.25));
    vertices.emplace_back(glm::dvec3(5, 1, 0.25));
    expected_prob_approx.emplace_back(0.5f);
    // Ray parallel to the voxel ellipsoid, but near the centre.
    // Note: we build a perfect plane above in the centre of the voxel. However, the quantisation of the mean
    // can offset the plane. Running exactly on the plane on which we generated the points will not result in the same
    // effect in testing. We must offset the ray a little to ensure we are a bit off the plane either when using
    // voxel mean quantisation or not.
    vertices.emplace_back(glm::dvec3(1, 5, 1.01));
    vertices.emplace_back(glm::dvec3(1, -5, 1.01));
    expected_prob_approx.emplace_back(0.5f);
    // Ray running across the voxel, down towards the ellipsoid, but not crossing. This simulates rays running near
    // parallel a ground plane as it approaches the sample position.
    vertices.emplace_back(glm::dvec3(-5, 1, 2));
    vertices.emplace_back(glm::dvec3(5, 1, 1));
    expected_prob_approx.emplace_back(0.5f);
    // Ray running across the voxel, and through the ellipsoid.
    vertices.emplace_back(glm::dvec3(-5, 1, 2));
    vertices.emplace_back(glm::dvec3(5, 1, 0.5));
    expected_prob_approx.emplace_back(0.269);

    ohm::Voxel target_voxel = map.voxel(target_key, false, &cache);
    const float initial_value = target_voxel;
    ndt.setTrace(true);

    for (size_t i = 0; i < vertices.size(); i += 2)
    {
      target_voxel.setValue(initial_value);
      TES_LINE(ohm::g_3es, TES_COLOUR(Cornsilk), glm::value_ptr(vertices[i]), glm::value_ptr(vertices[i + 1]),
               TES_PTR_ID(&sensor));
      TES_SERVER_UPDATE(ohm::g_3es, 0.0f);
      ndt.integrateMiss(target_voxel, vertices[i], vertices[i + 1]);
      TES_LINES_END(ohm::g_3es, TES_PTR_ID(&sensor));
      // Calculate the value adjustment.
      float value_adjustment = target_voxel.value() - initial_value;
      // Convert to probability.
      float ray_probability = ohm::valueToProbability(value_adjustment);
      // Validate
      EXPECT_NEAR(ray_probability, expected_prob_approx[i / 2], 0.01f);
    }
  }


  TEST(Ndt, MissCylindrical)
  {
    ohm::Trace trace("ndt-miss-cylindrical.3es");

    // Generate points within a 2m cube.
    const double voxel_resolution = 2.0;
    ohm::OccupancyMap map(voxel_resolution, ohm::MapFlag::kVoxelMean);
    ohm::NdtMap ndt(&map, true);
    ohm::MapCache cache;

    // Offset the map so that we have a voxel centred on the origin.
    map.setOrigin(glm::dvec3(-0.5 * voxel_resolution));
    map.setMissProbability(0.45f);
    ndt.setSensorNoise(0.05f);

    const double cylinder_radius = 0.3;
    uint32_t seed = 1153297050u;
    std::default_random_engine rng(seed);
    std::uniform_real_distribution<double> uniform(-0.99, 0.99);
    std::uniform_real_distribution<double> uniform_radius(cylinder_radius - ndt.sensorNoise(),
                                                          cylinder_radius + ndt.sensorNoise());
    const size_t sample_count = 10000;
    std::vector<glm::dvec3> vertices;
    vertices.reserve(sample_count);

    // Create samples which populate a single voxel and define a plane.

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
      vertices.emplace_back(sample);
    }

    // Simulate a sensor at the origin. Not used.
    const glm::dvec3 sensor(0, 0, 5);

    // First process the
    ohm::Key target_key;
    TES_STMT(std::vector<glm::vec3> lines);
    for (size_t i = 0; i < vertices.size(); ++i)
    {
      const glm::dvec3 &sample = vertices[i];

      ohm::Key key = map.voxelKey(sample);
      target_key = key;
      ohm::Voxel voxel = map.voxel(key, true, &cache);
      ndt.integrateHit(voxel, sensor, sample);
      TES_LINE(ohm::g_3es, TES_COLOUR(Yellow), glm::value_ptr(sensor), glm::value_ptr(sample));
      TES_STMT(lines.emplace_back(glm::vec3(sensor)));
      TES_STMT(lines.emplace_back(glm::vec3(sample)));
    }
    TES_LINES(ohm::g_3es, TES_COLOUR(Yellow), glm::value_ptr(*lines.data()), lines.size(), sizeof(*lines.data()));
    TES_SERVER_UPDATE(ohm::g_3es, 0.0f);

    // Now trace a ray through the voxel, but not ending in the voxel.
    std::vector<float> expected_prob_approx;
    vertices.clear();
    // // Ray straight down through the voxel
    vertices.emplace_back(sensor);
    vertices.emplace_back(glm::dvec3(0, 0, -5));
    expected_prob_approx.emplace_back(0.05f);
    // Reverse the ray above
    vertices.emplace_back(vertices[1]);
    vertices.emplace_back(vertices[0]);
    expected_prob_approx.emplace_back(0.05f);
    // Ray running parallel to the cylinder near the edge. Should be a near hit.
    vertices.emplace_back(glm::dvec3(cylinder_radius, cylinder_radius, 5));
    vertices.emplace_back(glm::dvec3(cylinder_radius, cylinder_radius, -5));
    expected_prob_approx.emplace_back(0.438f);
    // Ray running parallel to the cylinder, but should miss.
    vertices.emplace_back(glm::dvec3(1.5 * cylinder_radius, 1.5 * cylinder_radius, -5));
    vertices.emplace_back(glm::dvec3(2.0 * cylinder_radius, 2.0 * cylinder_radius, 5));
    expected_prob_approx.emplace_back(0.499f);
    // Ray across the middle of the cylinder.
    vertices.emplace_back(glm::dvec3(2, -cylinder_radius, 0));
    vertices.emplace_back(glm::dvec3(-2, -cylinder_radius, 0));
    expected_prob_approx.emplace_back(0.332f);
    // Ray across the end (top) of the cylinder.
    vertices.emplace_back(glm::dvec3(2, -cylinder_radius, 0.85));
    vertices.emplace_back(glm::dvec3(-2, -cylinder_radius, 0.85));
    expected_prob_approx.emplace_back(0.444f);
    // Ray across the voxel, missing the cylinder (top) of the cylinder.
    vertices.emplace_back(glm::dvec3(2, 2.0 * cylinder_radius, 0.85));
    vertices.emplace_back(glm::dvec3(-2, 2.0 * cylinder_radius, 0.85));
    expected_prob_approx.emplace_back(0.497f);

    ohm::Voxel target_voxel = map.voxel(target_key, false, &cache);
    const float initial_value = target_voxel;
    ndt.setTrace(true);

    for (size_t i = 0; i < vertices.size(); i += 2)
    {
      target_voxel.setValue(initial_value);
      TES_LINE(ohm::g_3es, TES_COLOUR(Cornsilk), glm::value_ptr(vertices[i]), glm::value_ptr(vertices[i + 1]),
               TES_PTR_ID(&sensor));
      TES_SERVER_UPDATE(ohm::g_3es, 0.0f);
      ndt.integrateMiss(target_voxel, vertices[i], vertices[i + 1]);
      TES_LINES_END(ohm::g_3es, TES_PTR_ID(&sensor));
      // Calculate the value adjustment.
      float value_adjustment = target_voxel.value() - initial_value;
      // Convert to probability.
      float ray_probability = ohm::valueToProbability(value_adjustment);
      // Validate
      EXPECT_NEAR(ray_probability, expected_prob_approx[i / 2], 0.01f);
    }
  }


  TEST(Ndt, MissSpherical)
  {
    ohm::Trace trace("ndt-miss-spherical.3es");

    // Generate points within a 2m cube.
    const double voxel_resolution = 2.0;
    ohm::OccupancyMap map(voxel_resolution, ohm::MapFlag::kVoxelMean);
    ohm::NdtMap ndt(&map, true);
    ohm::MapCache cache;

    // Offset the map so that we have a voxel centred on the origin.
    map.setOrigin(glm::dvec3(-0.5 * voxel_resolution));
    map.setMissProbability(0.45f);
    ndt.setSensorNoise(0.05f);

    const double radius = 0.3;
    uint32_t seed = 1153297050u;
    std::default_random_engine rng(seed);
    std::uniform_real_distribution<double> uniform(-0.99, 0.99);
    std::uniform_real_distribution<double> uniform_radius(radius - ndt.sensorNoise(), radius + ndt.sensorNoise());
    const size_t sample_count = 10000;
    std::vector<glm::dvec3> vertices;
    vertices.reserve(sample_count);

    // Create samples which populate a single voxel and define a plane.

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
      vertices.emplace_back(sample);
    }

    // Simulate a sensor at the origin. Not used.
    const glm::dvec3 sensor(0, 0, 5);

    // First process the
    ohm::Key target_key;
    TES_STMT(std::vector<glm::vec3> lines);
    for (size_t i = 0; i < vertices.size(); ++i)
    {
      const glm::dvec3 &sample = vertices[i];

      ohm::Key key = map.voxelKey(sample);
      target_key = key;
      ohm::Voxel voxel = map.voxel(key, true, &cache);
      ndt.integrateHit(voxel, sensor, sample);
      TES_LINE(ohm::g_3es, TES_COLOUR(Yellow), glm::value_ptr(sensor), glm::value_ptr(sample));
      TES_STMT(lines.emplace_back(glm::vec3(sensor)));
      TES_STMT(lines.emplace_back(glm::vec3(sample)));
    }
    TES_LINES(ohm::g_3es, TES_COLOUR(Yellow), glm::value_ptr(*lines.data()), lines.size(), sizeof(*lines.data()));
    TES_SERVER_UPDATE(ohm::g_3es, 0.0f);

    // Now trace a ray through the voxel, but not ending in the voxel.
    std::vector<float> expected_prob_approx;
    vertices.clear();
    // // Ray straight down through the voxel
    vertices.emplace_back(sensor);
    vertices.emplace_back(glm::dvec3(0, 0, -5));
    expected_prob_approx.emplace_back(0.05f);
    // Reverse the ray above
    vertices.emplace_back(vertices[1]);
    vertices.emplace_back(vertices[0]);
    expected_prob_approx.emplace_back(0.05f);
    // Edge of the sphere..
    vertices.emplace_back(glm::dvec3(radius, radius, 5));
    vertices.emplace_back(glm::dvec3(radius, radius, -5));
    expected_prob_approx.emplace_back(0.477f);
    // Near the edge of the sphere, but should miss.
    vertices.emplace_back(glm::dvec3(1.5 * radius, 1.5 * radius, -5));
    vertices.emplace_back(glm::dvec3(2.0 * radius, 2.0 * radius, 5));
    expected_prob_approx.emplace_back(0.5f);

    ohm::Voxel target_voxel = map.voxel(target_key, false, &cache);
    const float initial_value = target_voxel;
    ndt.setTrace(true);

    for (size_t i = 0; i < vertices.size(); i += 2)
    {
      target_voxel.setValue(initial_value);
      TES_LINE(ohm::g_3es, TES_COLOUR(Cornsilk), glm::value_ptr(vertices[i]), glm::value_ptr(vertices[i + 1]),
               TES_PTR_ID(&sensor));
      TES_SERVER_UPDATE(ohm::g_3es, 0.0f);
      ndt.integrateMiss(target_voxel, vertices[i], vertices[i + 1]);
      TES_LINES_END(ohm::g_3es, TES_PTR_ID(&sensor));
      // Calculate the value adjustment.
      float value_adjustment = target_voxel.value() - initial_value;
      // Convert to probability.
      float ray_probability = ohm::valueToProbability(value_adjustment);
      // Validate
      EXPECT_NEAR(ray_probability, expected_prob_approx[i / 2], 0.01f);
    }
  }
}  // namespace ndttests
