// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "SlamCloudLoader.h"

#include "PointCloudReader.h"
#include "SlamIO.h"

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
using Clock = std::chrono::high_resolution_clock;
}  // namespace

namespace slamio
{
struct SlamCloudLoaderDetail
{
  PointCloudReaderPtr sample_reader;
  PointCloudReaderPtr trajectory_reader;

  CloudPoint trajectory_buffer[2] = {};
  glm::dvec3 trajectory_to_sensor_offset{};

  SamplePoint next_sample;
  uint64_t read_count = 0;
  uint64_t preload_index = 0;

  Clock::time_point first_sample_read_time;
  double first_sample_timestamp = -1.0;
  bool ray_cloud = false;
  bool real_time_mode = false;

  std::vector<SamplePoint> preload_samples;

  SlamCloudLoader::Log error_log;
};

template <typename T>
void error(std::ostream &out, const T &msg)
{
  out << msg << std::endl;
}

template <typename T, typename... Args>
void error(std::ostream &out, const T &msg, Args... args)
{
  out << msg;
  error(out, args...);
}

template <typename T>
void error(SlamCloudLoader::Log log, const T &msg)
{
  if (log)
  {
    std::ostringstream out;
    error(out, msg);
    log(out.str().c_str());
  }
}

template <typename T, typename... Args>
void error(SlamCloudLoader::Log log, const T &msg, Args... args)
{
  if (log)
  {
    std::ostringstream out;
    error(out, msg, args...);
    log(out.str().c_str());
  }
}

SlamCloudLoader::SlamCloudLoader(bool real_time_mode)
  : imp_(std::make_unique<SlamCloudLoaderDetail>())
{
  imp_->real_time_mode = real_time_mode;
}


SlamCloudLoader::~SlamCloudLoader() = default;


void SlamCloudLoader::setErrorLog(Log error_log)
{
  imp_->error_log = error_log;
}


SlamCloudLoader::Log SlamCloudLoader::errorLog() const
{
  return imp_->error_log;
}


void SlamCloudLoader::setSensorOffset(const glm::dvec3 &offset)
{
  imp_->trajectory_to_sensor_offset = offset;
}


glm::dvec3 SlamCloudLoader::sensorOffset() const
{
  return imp_->trajectory_to_sensor_offset;
}


bool SlamCloudLoader::openWithTrajectory(const char *sample_file_path, const char *trajectory_file_path)
{
  return open(sample_file_path, trajectory_file_path, false);
}


bool SlamCloudLoader::openPointCloud(const char *sample_file_path)
{
  return open(sample_file_path, nullptr, false);
}


bool SlamCloudLoader::openRayCloud(const char *sample_file_path)
{
  return open(sample_file_path, nullptr, true);
}


void SlamCloudLoader::close()
{
  imp_->sample_reader = nullptr;
  imp_->trajectory_reader = nullptr;
  imp_->read_count = 0;
  imp_->preload_index = 0;
  imp_->first_sample_timestamp = -1.0;
  imp_->ray_cloud = false;
  imp_->preload_samples = std::vector<SamplePoint>();
}


size_t SlamCloudLoader::numberOfPoints() const
{
  return imp_->sample_reader ? imp_->sample_reader->pointCount() : 0;
}


bool SlamCloudLoader::realTimeMode() const
{
  return imp_->real_time_mode;
}


bool SlamCloudLoader::sampleFileIsOpen() const
{
  return imp_->sample_reader != nullptr;
}


bool SlamCloudLoader::trajectoryFileIsOpen() const
{
  return imp_->trajectory_reader != nullptr;
}


bool SlamCloudLoader::hasTimestamp() const
{
  return imp_->sample_reader && (imp_->sample_reader->availableChannels() & DataChannel::Time) != DataChannel::None;
}


bool SlamCloudLoader::hasOrigin() const
{
  return trajectoryFileIsOpen();
}


bool SlamCloudLoader::hasIntensity() const
{
  return imp_->sample_reader &&
         (imp_->sample_reader->availableChannels() & DataChannel::Intensity) != DataChannel::None;
}


bool SlamCloudLoader::hasColour() const
{
  return imp_->sample_reader &&
         (imp_->sample_reader->availableChannels() & DataChannel::ColourRgb) == DataChannel::ColourRgb;
}


void SlamCloudLoader::preload(size_t point_count)
{
  if (!imp_->sample_reader)
  {
    return;
  }

  if (point_count && numberOfPoints())
  {
    point_count = std::min(point_count, numberOfPoints());
  }

  const bool load_all = point_count == 0 || (numberOfPoints() && point_count >= numberOfPoints());

  std::vector<SamplePoint> preload_data;
  // Cache the read count. It will increment as we preload, but need it to remain unchanged on return.
  uint64_t initial_read_count = imp_->read_count;
  preload_data.reserve(point_count);

  SamplePoint sample{};
  while (nextSample(sample) && (preload_data.size() < point_count || load_all))
  {
    preload_data.emplace_back(sample);
  }

  std::swap(imp_->preload_samples, preload_data);
  imp_->preload_index = 0;
  imp_->read_count = initial_read_count;
}


bool SlamCloudLoader::nextSample(SamplePoint &sample)
{
  if (loadPoint())
  {
    ++imp_->read_count;

    // Read next sample.
    sample = imp_->next_sample;

    // If in real time mode, sleep until we should deliver this sample.
    if (imp_->real_time_mode && imp_->first_sample_timestamp >= 0)
    {
      const double sample_relative_time = sample.timestamp - imp_->first_sample_timestamp;
      if (sample_relative_time > 0)
      {
        auto uptime = Clock::now() - imp_->first_sample_read_time;
        const double sleep_time =
          sample_relative_time - std::chrono::duration_cast<std::chrono::duration<double>>(uptime).count();
        if (sleep_time > 0)
        {
          std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
      }
    }
    return true;
  }
  return false;
}


bool SlamCloudLoader::open(const char *sample_file_path, const char *trajectory_file_path, bool ray_cloud)
{
  close();

  imp_->sample_reader = slamio::createCloudReaderFromFilename(sample_file_path);

  if (!ray_cloud)
  {
    if (trajectory_file_path && trajectory_file_path[0])
    {
      imp_->trajectory_reader = slamio::createCloudReaderFromFilename(trajectory_file_path);

      if (!imp_->trajectory_reader)
      {
        error(imp_->error_log, "Unsupported extension for trajectory file ", trajectory_file_path);
        close();
        return false;
      }
    }
  }

  if (!imp_->sample_reader)
  {
    error(imp_->error_log, "Unsupported extension for point cloud file ", sample_file_path);
    close();
    return false;
  }

  DataChannel required_channels = DataChannel::Position;
  if (imp_->trajectory_reader)
  {
    if (!imp_->trajectory_reader->open(trajectory_file_path))
    {
      error(imp_->error_log, "Failed to open trajectory file ", trajectory_file_path);
      close();
      return false;
    }

    if (imp_->trajectory_reader->readChunk(imp_->trajectory_buffer, 2) != 2)
    {
      std::cerr << "Failed to read data from trajectory file " << trajectory_file_path << std::endl;
      error(imp_->error_log, "Failed to read data from trajectory file ", trajectory_file_path);
      close();
      return false;
    }

    const DataChannel required_trajectory_channels = DataChannel::Time | DataChannel::Position;
    if ((required_trajectory_channels & imp_->trajectory_reader->availableChannels()) != required_trajectory_channels)
    {
      error(imp_->error_log, "Unable to load required data channels from trajectory from file ", trajectory_file_path);
      close();
      return false;
    }

    // Need time to correlate with trajectory.
    required_channels |= DataChannel::Time;
  }

  if (!imp_->sample_reader->open(sample_file_path))
  {
    error(imp_->error_log, "Unable to open point cloud ", sample_file_path);
    close();
    return false;
  }

  if (!imp_->trajectory_reader)
  {
    if (ray_cloud)
    {
      // Ray cloud expects normals.
      required_channels |= DataChannel::Normal;
    }
  }

  // Check for required channels.
  if ((required_channels & imp_->sample_reader->availableChannels()) != required_channels)
  {
    error(imp_->error_log, "Unable to load required data channels from point cloud ", sample_file_path);
    close();
    return false;
  }

  imp_->ray_cloud = ray_cloud;
  return true;
}

bool SlamCloudLoader::loadPoint()
{
  if (imp_->preload_index < imp_->preload_samples.size())
  {
    imp_->next_sample = imp_->preload_samples[imp_->preload_index++];
    return true;
  }

  if (!imp_->preload_samples.empty())
  {
    // Preload done. Release the memory for preload_samples
    imp_->preload_samples.shrink_to_fit();
    imp_->preload_index = 0;
  }

  CloudPoint point{};
  if (imp_->sample_reader && imp_->sample_reader->readNext(point))
  {
    SamplePoint sample = c2sPt(point);
    if (imp_->ray_cloud)
    {
      // Loading a ray cloud. The normal is the vector from sample back to sensor.
      sample.origin = point.position + point.normal;
    }
    else
    {
      sampleTrajectory(sample.origin, sample.sample, sample.timestamp);
      imp_->next_sample = sample;
    }

    if (imp_->first_sample_timestamp < 0)
    {
      imp_->first_sample_timestamp = sample.timestamp;
      imp_->first_sample_read_time = Clock::now();
    }
    return true;
  }
  return false;
}


bool SlamCloudLoader::sampleTrajectory(glm::dvec3 &position, const glm::dvec3 &sample, double timestamp)
{
  if (imp_->trajectory_reader)
  {
    CloudPoint traj_point;

    while (timestamp > imp_->trajectory_buffer[1].timestamp && imp_->trajectory_reader->readNext(traj_point))
    {
      imp_->trajectory_buffer[0] = imp_->trajectory_buffer[1];
      imp_->trajectory_buffer[1] = traj_point;
    }

    if (imp_->trajectory_buffer[0].timestamp <= timestamp && timestamp <= imp_->trajectory_buffer[1].timestamp &&
        imp_->trajectory_buffer[0].timestamp != imp_->trajectory_buffer[1].timestamp)
    {
      const double lerp = (timestamp - imp_->trajectory_buffer[0].timestamp) /
                          (imp_->trajectory_buffer[1].timestamp - imp_->trajectory_buffer[0].timestamp);
      position = imp_->trajectory_buffer[0].position +
                 lerp * (imp_->trajectory_buffer[1].position - imp_->trajectory_buffer[0].position);
      position += imp_->trajectory_to_sensor_offset;
      return true;
    }
  }
  else
  {
    position = sample;
  }
  return false;
}
}  // namespace slamio
