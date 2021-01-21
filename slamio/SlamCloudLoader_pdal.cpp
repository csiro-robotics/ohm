//
// author Kazys Stepanas
//
#include "SlamCloudLoader.h"

#ifdef _MSC_VER
// std::equal with parameters that may be unsafe warning under Visual Studio.
#pragma warning(disable : 4996)
#endif  // _MSC_VER

#include <ohmutil/SafeIO.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>

// This is a hacky port of the libLAS version. The text trajectory reader could be ported to a PDAL reader.

namespace
{
struct TrajectoryPoint
{
  double timestamp;
  glm::dvec3 origin;
};

struct SamplePoint : TrajectoryPoint
{
  glm::dvec3 sample;
};

using Clock = std::chrono::high_resolution_clock;
}  // namespace

using ReadTrajectoryFunc = std::function<bool(TrajectoryPoint &point)>;

struct SlamCloudLoaderDetail
{
  std::unique_ptr<pdal::StageFactory> pdal_factory = std::make_unique<pdal::StageFactory>();
  std::shared_ptr<pdal::Stage> sample_reader = nullptr;
  std::shared_ptr<pdal::Stage> trajectory_reader = nullptr;

  std::unique_ptr<pdal::PointTable> sample_table = nullptr;
  std::unique_ptr<pdal::PointTable> traj_table = nullptr;

  pdal::Dimension::Id sample_time_field = pdal::Dimension::Id::Unknown;
  pdal::Dimension::Id trajectory_time_field = pdal::Dimension::Id::Unknown;

  pdal::PointViewPtr samples;
  pdal::PointViewPtr trajectory;
  uint64_t samples_view_index = 0;
  uint64_t traj_view_index = 0;

  std::string sample_file_path;
  std::string trajectory_file_path;
  ReadTrajectoryFunc read_trajectory_point;
  std::ifstream trajectory_file;
  std::string traj_line;
  glm::dvec3 trajectory_to_sensor_offset = glm::dvec3(0);
  TrajectoryPoint trajectory_buffer[2] = { TrajectoryPoint{}, TrajectoryPoint{} };

  SamplePoint next_sample = SamplePoint{};
  uint64_t next_sample_read_index = 0;

  Clock::time_point first_sample_read_time;
  double first_sample_timestamp = -1.0;
  bool real_time_mode = false;

  inline SlamCloudLoaderDetail() { memset(&trajectory_buffer, 0, sizeof(trajectory_buffer)); }
};

namespace
{
std::string getFileExtension(const std::string &file)
{
  const size_t last_dot = file.find_last_of('.');
  if (last_dot != std::string::npos)
  {
    return file.substr(last_dot + 1);
  }

  return "";
}

std::shared_ptr<pdal::Stage> createReader(pdal::StageFactory &factory, const std::string &file_name)
{
  const std::string ext = getFileExtension(file_name);
  std::string reader_type;
  pdal::Options options;

  reader_type = ext;

  if (ext.compare("laz") == 0)
  {
    reader_type = "las";
    options.add("compression", "EITHER");
  }

  reader_type = "readers." + reader_type;
  std::shared_ptr<pdal::Stage> reader(factory.createStage(reader_type),  //
                                      [&factory](pdal::Stage *stage) { factory.destroyStage(stage); });

  if (reader)
  {
    options.add("filename", file_name);
    reader->setOptions(options);
  }

  return reader;
}

/// Ordered field IDs to try to use to resolve time.
pdal::Dimension::Id selectTimeField(const pdal::PointTable &points)
{
  const pdal::Dimension::Id time_fields[] = { pdal::Dimension::Id::GpsTime, pdal::Dimension::Id::InternalTime,
                                              pdal::Dimension::Id::OffsetTime };

  for (const auto field : time_fields)
  {
    if (points.layout()->hasDim(field))
    {
      return field;
    }
  }

  return pdal::Dimension::Id::Unknown;
}
}  // namespace

SlamCloudLoader::SlamCloudLoader(bool real_time_mode)
  : imp_(new SlamCloudLoaderDetail)
{
  imp_->real_time_mode = real_time_mode;
}


SlamCloudLoader::~SlamCloudLoader()
{
  // Must close first to have the correct cleanup order for PDAL
  close();
  delete imp_;
}


void SlamCloudLoader::setSensorOffset(const glm::dvec3 &offset)
{
  imp_->trajectory_to_sensor_offset = offset;
}


glm::dvec3 SlamCloudLoader::sensorOffset() const
{
  return imp_->trajectory_to_sensor_offset;
}


bool SlamCloudLoader::open(const char *sample_file_path, const char *trajectory_file_path)
{
  close();

  imp_->sample_file_path = sample_file_path;
  imp_->trajectory_file_path = trajectory_file_path;

  // Get the reader name for the file being loaded.
  imp_->sample_reader = createReader(*imp_->pdal_factory, sample_file_path);

  if (!imp_->sample_reader)
  {
    close();
    return false;
  }

  bool text_trajectory = false;
  if (trajectory_file_path && trajectory_file_path[0])
  {
    const std::string traj_ext = getFileExtension(trajectory_file_path);
    if (traj_ext.compare("txt") == 0)
    {
      // Text based trajectory.
      imp_->trajectory_file.open(trajectory_file_path, std::ios::binary);
      text_trajectory = true;

      if (!imp_->trajectory_file.is_open())
      {
        close();
        return false;
      }
    }
    else
    {
      imp_->trajectory_reader = createReader(*imp_->pdal_factory, trajectory_file_path);
      if (!imp_->trajectory_reader)
      {
        close();
        return false;
      }
    }
  }

  imp_->sample_table = std::make_unique<pdal::PointTable>();
  imp_->sample_reader->prepare(*imp_->sample_table);
  imp_->sample_time_field = selectTimeField(*imp_->sample_table);

  if (imp_->sample_time_field == pdal::Dimension::Id::Unknown)
  {
    close();
    std::cerr << "Point cloud has not time data field" << std::endl;
    return false;
  }

  pdal::PointViewSet point_sets = imp_->sample_reader->execute(*imp_->sample_table);
  if (point_sets.empty())
  {
    close();
    return false;
  }

  imp_->samples = *point_sets.begin();

  imp_->traj_view_index = 0u;
  if (text_trajectory)
  {
    imp_->read_trajectory_point = [this](TrajectoryPoint &point) -> bool {
      if (!std::getline(imp_->trajectory_file, imp_->traj_line))
      {
        // End of file.
        return false;
      }

      // sscanf is far faster than using stream operators.
      glm::dquat orientation;
      if (sscanf_s(imp_->traj_line.c_str(), "%lg %lg %lg %lg %lg %lg %lg %lg", &point.timestamp, &point.origin.x,
                   &point.origin.y, &point.origin.z, &orientation.w, &orientation.x, &orientation.y,
                   &orientation.z) != 8)
      {
        return false;
      }

      ++imp_->traj_view_index;
      return true;
    };
  }
  else if (imp_->trajectory_reader)
  {
    imp_->traj_table = std::make_unique<pdal::PointTable>();
    imp_->trajectory_reader->prepare(*imp_->traj_table);

    imp_->trajectory_time_field = selectTimeField(*imp_->sample_table);
    if (imp_->trajectory_time_field == pdal::Dimension::Id::Unknown)
    {
      std::cerr << "Unable to resolve trajectory time field" << std::endl;
      close();
      return false;
    }

    point_sets = imp_->trajectory_reader->execute(*imp_->traj_table);
    if (point_sets.empty())
    {
      close();
      return false;
    }

    imp_->trajectory = *point_sets.begin();
    imp_->traj_view_index = 0u;

    imp_->read_trajectory_point = [this](TrajectoryPoint &point) -> bool {
      if (imp_->traj_view_index >= imp_->trajectory->size())
      {
        return false;
      }

      point.origin.x = imp_->trajectory->getFieldAs<double>(pdal::Dimension::Id::X, imp_->traj_view_index);
      point.origin.y = imp_->trajectory->getFieldAs<double>(pdal::Dimension::Id::Y, imp_->traj_view_index);
      point.origin.z = imp_->trajectory->getFieldAs<double>(pdal::Dimension::Id::Z, imp_->traj_view_index);
      point.timestamp = imp_->trajectory->getFieldAs<double>(imp_->trajectory_time_field, imp_->traj_view_index);
      ++imp_->traj_view_index;
      return true;
    };
  }

  // Prime the trajectory buffer.
  bool trajectory_primed = imp_->read_trajectory_point(imp_->trajectory_buffer[0]);
  if (!trajectory_primed && text_trajectory)
  {
    // Try a second time to read the first trajectory point from text file. First line may be headings.
    trajectory_primed = imp_->read_trajectory_point(imp_->trajectory_buffer[0]);
  }
  trajectory_primed = trajectory_primed && imp_->read_trajectory_point(imp_->trajectory_buffer[1]);

  if (!trajectory_primed)
  {
    close();
    return false;
  }

  return true;
}


void SlamCloudLoader::close()
{
  imp_->trajectory_reader.reset();
  imp_->sample_reader.reset();
  imp_->sample_table.reset();
  imp_->traj_table.reset();
  imp_->sample_file_path.clear();
  imp_->trajectory_file_path.clear();
  imp_->read_trajectory_point = ReadTrajectoryFunc();
  imp_->trajectory_file.close();
  imp_->next_sample_read_index = imp_->samples_view_index = imp_->traj_view_index = 0u;
}


size_t SlamCloudLoader::numberOfPoints() const
{
  return imp_->samples ? imp_->samples->size() : 0u;
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


void SlamCloudLoader::preload(size_t /*point_count*/)
{
  // PDAL doesn't support streaming.
}


bool SlamCloudLoader::nextPoint(glm::dvec3 &sample, glm::dvec3 *origin, double *timestamp_out)
{
  loadPoint();
  if (imp_->next_sample_read_index < imp_->samples->size())
  {
    ++imp_->next_sample_read_index;
    // Read next sample.
    const SamplePoint sample_point = imp_->next_sample;
    // if (_imp->nextSampleReadIndex == _imp->samplesBuffer.size() || _imp->nextSampleReadIndex >= 1024)
    //{
    //  // Shrink the remaining points.
    //  const size_t newSize = (_imp->nextSampleReadIndex < _imp->samplesBuffer.size()) ? _imp->samplesBuffer.size() -
    //  _imp->nextSampleReadIndex : 0; if (_imp->nextSampleReadIndex < _imp->samplesBuffer.size())
    //  {
    //    std::memmove(_imp->samplesBuffer.data(),
    //            _imp->samplesBuffer.data() + _imp->nextSampleReadIndex,
    //            sizeof(*_imp->samplesBuffer.data()) * newSize
    //            );
    //  }
    //  _imp->samplesBuffer.resize(newSize);
    //  _imp->nextSampleReadIndex = 0;
    //}
    sample = sample_point.sample;
    if (timestamp_out)
    {
      *timestamp_out = sample_point.timestamp;
    }
    if (origin)
    {
      *origin = sample_point.origin + imp_->trajectory_to_sensor_offset;
    }

    // If in real time mode, sleep until we should deliver this sample.
    if (imp_->real_time_mode && imp_->first_sample_timestamp >= 0)
    {
      const double sample_relative_time = sample_point.timestamp - imp_->first_sample_timestamp;
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


bool SlamCloudLoader::loadPoint()
{
  if (imp_->samples_view_index < imp_->samples->size())
  {
    SamplePoint sample{};

    sample.timestamp = imp_->samples->getFieldAs<double>(imp_->sample_time_field, imp_->samples_view_index);
    sample.sample.x = imp_->samples->getFieldAs<double>(pdal::Dimension::Id::X, imp_->samples_view_index);
    sample.sample.y = imp_->samples->getFieldAs<double>(pdal::Dimension::Id::Y, imp_->samples_view_index);
    sample.sample.z = imp_->samples->getFieldAs<double>(pdal::Dimension::Id::Z, imp_->samples_view_index);
    sample.origin = glm::dvec3(0);

    ++imp_->samples_view_index;

    sampleTrajectory(sample.origin, sample.sample, sample.timestamp);
    imp_->next_sample = sample;

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
  if (imp_->read_trajectory_point)
  {
    TrajectoryPoint traj_point{};

    while (timestamp > imp_->trajectory_buffer[1].timestamp && imp_->read_trajectory_point(traj_point))
    {
      imp_->trajectory_buffer[0] = imp_->trajectory_buffer[1];
      imp_->trajectory_buffer[1] = traj_point;
    }

    if (imp_->trajectory_buffer[0].timestamp <= timestamp && timestamp <= imp_->trajectory_buffer[1].timestamp &&
        imp_->trajectory_buffer[0].timestamp != imp_->trajectory_buffer[1].timestamp)
    {
      const double lerp = (timestamp - imp_->trajectory_buffer[0].timestamp) /
                          (imp_->trajectory_buffer[1].timestamp - imp_->trajectory_buffer[0].timestamp);
      position = imp_->trajectory_buffer[0].origin +
                 lerp * (imp_->trajectory_buffer[1].origin - imp_->trajectory_buffer[0].origin);
      return true;
    }
  }
  position = sample;
  return false;
}
