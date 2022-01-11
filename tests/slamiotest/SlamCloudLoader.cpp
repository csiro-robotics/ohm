// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include "slamio/PointCloudReader.h"
#include "slamio/SlamCloudLoader.h"
#include "slamio/SlamIO.h"

#include <glm/glm.hpp>

#include <fstream>
#include <limits>
#include <random>
#include <vector>

namespace
{
const size_t traj_sample_hz = 100;
const size_t sample_count = 10000;
const double data_time = 10.0;
const double room_extents = 10.0;
const glm::dvec3 traj_start(0, 0, 0.5);
const glm::dvec3 traj_end(0.5 * room_extents, 0.5 * room_extents, 1.5);
const double e0 = 1e-12;

glm::dvec4 generateTrajectoryPoint(double time)
{
  const glm::dvec4 pt = glm::dvec4(traj_start + (traj_end - traj_start) * time / data_time, time);
  return pt;
}

void writeTimestampedPlyCloud(const std::string &path, const std::vector<glm::dvec4> &timestamped_points)
{
  std::ofstream out(path.c_str(), std::ios::binary);

  out << "ply\n";
  out << "format ascii 1.0\n";
  out << "comment Test data for ras slamio\n";
  out << "element vertex " << timestamped_points.size() << '\n';
  out << "property double time\n";
  out << "property double x\n";
  out << "property double y\n";
  out << "property double z\n";
  out << "end_header\n";
  out << std::flush;

  out.precision(std::numeric_limits<double>::max_digits10);
  for (const auto &time_point : timestamped_points)
  {
    out << time_point.w << ' ' << time_point.x << ' ' << time_point.y << ' ' << time_point.z << '\n';
  }

  out << std::flush;
  out.close();
}

void writeTextTrajectory(const std::string &path, const std::vector<glm::dvec4> &timestamped_points)
{
  std::ofstream out(path.c_str(), std::ios::binary);

  out << "time x y z\n";
  out.precision(std::numeric_limits<double>::max_digits10);
  for (const auto &time_point : timestamped_points)
  {
    out << time_point.w << ' ' << time_point.x << ' ' << time_point.y << ' ' << time_point.z << '\n';
  }

  out << std::flush;
  out.close();
}

void generateSlamCloud(std::vector<glm::dvec4> *timestamped_samples, std::vector<glm::dvec4> *timestamped_trajectory)
{
  // Generate a simplistic trajectory and cloud.

  // Generate trajectory.
  if (timestamped_trajectory)
  {
    for (double time = 0.0; time <= data_time; time += 1.0 / traj_sample_hz)
    {
      timestamped_trajectory->emplace_back(generateTrajectoryPoint(time));
    }
  }

  // Generate large sphere around the trajectory region.
  if (timestamped_samples)
  {
    std::mt19937 rand_engine(0x12345678);
    std::uniform_real_distribution<double> unit_rand(-1, 1);
    for (size_t i = 0; i < sample_count; ++i)
    {
      const double time = data_time / sample_count;
      glm::dvec3 pt(unit_rand(rand_engine), unit_rand(rand_engine), unit_rand(rand_engine));
      pt = glm::normalize(pt);
      pt *= room_extents;
      timestamped_samples->emplace_back(glm::dvec4(pt, time));
    }
  }
}
}  // namespace


namespace slamio
{
TEST(SlamIO, SlamRead)
{
  std::vector<glm::dvec4> samples;
  std::vector<glm::dvec4> trajectory;

  generateSlamCloud(&samples, &trajectory);
  const std::string sample_file = "slam-samples.ply";
  const std::string trajectory_file = "slam-trajectory.ply";
  writeTimestampedPlyCloud(sample_file.c_str(), samples);
  writeTimestampedPlyCloud(trajectory_file.c_str(), trajectory);

  slamio::SlamCloudLoader reader;
  reader.setErrorLog([](const char *msg) { std::cerr << msg << std::flush; });

  ASSERT_TRUE(reader.openWithTrajectory(sample_file.c_str(), trajectory_file.c_str()));

  slamio::SamplePoint sample{};
  size_t i = 0;
  while (reader.nextSample(sample))
  {
    ASSERT_NEAR(sample.timestamp, samples[i].w, e0);
    ASSERT_NEAR(sample.sample.x, samples[i].x, e0);
    ASSERT_NEAR(sample.sample.y, samples[i].y, e0);
    ASSERT_NEAR(sample.sample.z, samples[i].z, e0);
    // Generate a trajectory point at the sample time and check it's near the reported position.
    const glm::dvec3 traj_pt = glm::dvec3(generateTrajectoryPoint(sample.timestamp));
    ASSERT_NEAR(sample.origin.x, traj_pt.x, e0);
    ASSERT_NEAR(sample.origin.y, traj_pt.y, e0);
    ASSERT_NEAR(sample.origin.z, traj_pt.z, e0);
    ++i;
  }
}

TEST(SlamIO, CloudRead)
{
  std::vector<glm::dvec4> samples;

  generateSlamCloud(&samples, nullptr);
  const std::string sample_file = "cloud-read-samples.ply";
  writeTimestampedPlyCloud(sample_file, samples);

  auto reader = slamio::createCloudReaderFromFilename(sample_file.c_str());
  ASSERT_NE(reader, nullptr);
  // reader->setErrorLog([](const char *msg) { std::cerr << msg << std::flush; });

  ASSERT_TRUE(reader->open(sample_file.c_str()));
  ASSERT_EQ(reader->pointCount(), samples.size());

  slamio::CloudPoint point{};
  for (size_t i = 0; i < samples.size(); ++i)
  {
    ASSERT_TRUE(reader->readNext(point));
    ASSERT_NEAR(point.timestamp, samples[i].w, e0);
    ASSERT_NEAR(point.position.x, samples[i].x, e0);
    ASSERT_NEAR(point.position.y, samples[i].y, e0);
    ASSERT_NEAR(point.position.z, samples[i].z, e0);
  }
}

TEST(SlamIO, TrajectoryRead)
{
  std::vector<glm::dvec4> trajectory;

  generateSlamCloud(nullptr, &trajectory);
  const std::string trajectory_file = "trajectory-read-trajectory.txt";
  writeTextTrajectory(trajectory_file, trajectory);

  auto reader = slamio::createCloudReaderFromFilename(trajectory_file.c_str());
  ASSERT_NE(reader, nullptr);
  // reader->setErrorLog([](const char *msg) { std::cerr << msg << std::flush; });

  ASSERT_TRUE(reader->open(trajectory_file.c_str()));
  // Note: point count will not be known for a text trajectory.

  slamio::CloudPoint point{};
  for (size_t i = 0; i < trajectory.size(); ++i)
  {
    ASSERT_TRUE(reader->readNext(point));
    ASSERT_EQ(point.timestamp, trajectory[i].w);
    ASSERT_NEAR(point.timestamp, trajectory[i].w, e0);
    ASSERT_NEAR(point.position.x, trajectory[i].x, e0);
    ASSERT_NEAR(point.position.y, trajectory[i].y, e0);
    ASSERT_NEAR(point.position.z, trajectory[i].z, e0);
  }
}
}  // namespace slamio
