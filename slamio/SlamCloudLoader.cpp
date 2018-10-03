//
// author Kazys Stepanas
//
#include "SlamCloudLoader.h"

#ifdef _MSC_VER
// std::equal with parameters that may be unsafe warning under Visual Studio.
#pragma warning(disable : 4996)
#endif // _MSC_VER

#include <ohmutil/SafeIO.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <liblas/liblas.hpp>
#include <liblas/factory.hpp>
#include <liblas/reader.hpp>

namespace
{
  struct TrajectoryPoint
  {
    double timestamp;
    glm::dvec3 origin;
    glm::dquat orientation;
  };

  struct SamplePoint : TrajectoryPoint
  {
    glm::dvec3 sample;
  };

  typedef std::chrono::high_resolution_clock Clock;
}

struct SlamCloudLoaderDetail
{
  liblas::ReaderFactory *las_reader_factory;
  liblas::Reader *sample_reader;
  liblas::Reader *trajectory_reader;
  std::string sample_file_path;
  std::string trajectory_file_path;
  std::ifstream sample_file;
  std::ifstream trajectory_file;
  std::function<bool(TrajectoryPoint &point)> read_trajectory_point;
  std::string traj_line;
  TrajectoryPoint trajectory_buffer[2];
  size_t number_of_points;

  std::vector<SamplePoint> samples_buffer;
  size_t next_sample_read_index;

  Clock::time_point first_sample_read_time;
  double first_sample_timestamp;
  bool real_time_mode;

  inline SlamCloudLoaderDetail()
    : las_reader_factory(nullptr)
    , sample_reader(nullptr)
    , trajectory_reader(nullptr)
    , number_of_points(0)
    , next_sample_read_index(0)
    , first_sample_timestamp(-1.0)
    , real_time_mode(false)
  {
    memset(&trajectory_buffer, 0, sizeof(trajectory_buffer));
  }

  inline ~SlamCloudLoaderDetail()
  {
    // Factory must be destroyed last.
    delete trajectory_reader;
    delete sample_reader;
    delete las_reader_factory;
  }
};

namespace
{
  std::string getFileExtension(const std::string &file)
  {
    const size_t last_dot = file.find_last_of(".");
    if (last_dot != std::string::npos)
    {
      return file.substr(last_dot + 1);
    }

    return "";
  }

  bool openLasFile(std::ifstream &in, const std::string &file_name)
  {
    const std::string ext = getFileExtension(file_name);
    if (ext.compare("laz") == 0 || ext.compare("las") == 0)
    {
      return liblas::Open(in, file_name);
    }

    // Extension omitted.
    // Try for a LAZ file (compressed), then a LAS file.
    if (liblas::Open(in, file_name + ".laz"))
    {
      return true;
    }
    return liblas::Open(in, file_name + ".las");
  }
}

SlamCloudLoader::SlamCloudLoader(bool real_time_mode)
  : imp_(new SlamCloudLoaderDetail)
{
  imp_->las_reader_factory = new liblas::ReaderFactory;
  imp_->real_time_mode = real_time_mode;
}


SlamCloudLoader::~SlamCloudLoader()
{
  delete imp_;
}


bool SlamCloudLoader::open(const char *sample_file_path, const char *trajectory_file_path)
{
  close();

  imp_->sample_file_path = sample_file_path;
  imp_->trajectory_file_path = trajectory_file_path;

  openLasFile(imp_->sample_file, imp_->sample_file_path);
  bool text_trajectory = false;
  {
    const std::string traj_ext = getFileExtension(trajectory_file_path);
    if (traj_ext.compare("txt") == 0)
    {
      // Text based trajectory.
      imp_->trajectory_file.open(trajectory_file_path, std::ios::binary);
      text_trajectory = true;
    }
    else
    {
      openLasFile(imp_->trajectory_file, imp_->trajectory_file_path);
    }
  }

  if (!sampleFileIsOpen() || !trajectoryFileIsOpen())
  {
    close();
    return false;
  }

  imp_->sample_reader = new liblas::Reader(imp_->las_reader_factory->CreateWithStream(imp_->sample_file));

  if (text_trajectory)
  {
    imp_->read_trajectory_point = [this](TrajectoryPoint &point) -> bool
    {
      if (!std::getline(imp_->trajectory_file, imp_->traj_line))
      {
        // End of file.
        return false;
      }

      // sscanf is far faster than using stream operators.
      if (sscanf_s(imp_->traj_line.c_str(), "%lg %lg %lg %lg %lg %lg %lg %lg",
                 &point.timestamp,
                 &point.origin.x, &point.origin.y, &point.origin.z,
                 &point.orientation.x, &point.orientation.y, &point.orientation.z, &point.orientation.w) != 8)
      {
        return false;
      }

      return true;
    };
  }
  else
  {
    imp_->trajectory_reader = new liblas::Reader(imp_->las_reader_factory->CreateWithStream(imp_->trajectory_file));
    imp_->read_trajectory_point = [this](TrajectoryPoint &point) -> bool
    {
      if (imp_->trajectory_reader->ReadNextPoint())
      {
        const liblas::Point &p = imp_->trajectory_reader->GetPoint();
        point.timestamp = p.GetTime();
        point.origin = glm::dvec3(p.GetX(), p.GetY(), p.GetZ());
        point.orientation = glm::dquat();
        return true;
      }
      return false;
    };
  }
  imp_->number_of_points = imp_->sample_reader->GetHeader().GetPointRecordsCount();

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
  delete imp_->trajectory_reader;
  delete imp_->sample_reader;
  imp_->sample_reader = imp_->trajectory_reader = nullptr;
  imp_->sample_file.close();
  imp_->trajectory_file.close();
  imp_->sample_file_path.clear();
  imp_->trajectory_file_path.clear();
  imp_->read_trajectory_point = [](TrajectoryPoint &) { return false; };
  imp_->number_of_points = 0;
}


size_t SlamCloudLoader::numberOfPoints() const
{
  return imp_->number_of_points;
}


bool SlamCloudLoader::realTimeMode() const
{
  return imp_->real_time_mode;
}


bool SlamCloudLoader::sampleFileIsOpen() const
{
  return imp_->sample_file.is_open();
}


bool SlamCloudLoader::trajectoryFileIsOpen() const
{
  return imp_->trajectory_file.is_open();
}


void SlamCloudLoader::preload(size_t point_count)
{
  if (point_count)
  {
    imp_->samples_buffer.reserve(point_count);
    while (point_count && loadPoint())
    {
      --point_count;
    }
  }
  else
  {
    while (loadPoint());
  }
}


bool SlamCloudLoader::nextPoint(glm::dvec3 &sample, glm::dvec3 *origin, double *timestamp_out, glm::dquat *orientation)
{
  loadPoint();
  if (imp_->next_sample_read_index < imp_->samples_buffer.size())
  {
    // Read next sample.
    const SamplePoint sample_point = imp_->samples_buffer[imp_->next_sample_read_index++];
    //if (_imp->nextSampleReadIndex == _imp->samplesBuffer.size() || _imp->nextSampleReadIndex >= 1024)
    //{
    //  // Shrink the remaining points.
    //  const size_t newSize = (_imp->nextSampleReadIndex < _imp->samplesBuffer.size()) ? _imp->samplesBuffer.size() - _imp->nextSampleReadIndex : 0;
    //  if (_imp->nextSampleReadIndex < _imp->samplesBuffer.size())
    //  {
    //    memmove(_imp->samplesBuffer.data(),
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
      *origin = sample_point.origin;
    }
    if (orientation)
    {
      *orientation = sample_point.orientation;
    }

    // If in real time mode, sleep until we should deliver this sample.
    if (imp_->real_time_mode && imp_->first_sample_timestamp >= 0)
    {
      const double sample_relative_time = sample_point.timestamp - imp_->first_sample_timestamp;
      if (sample_relative_time > 0)
      {
        auto uptime = Clock::now() - imp_->first_sample_read_time;
        const double sleep_time = sample_relative_time - std::chrono::duration_cast<std::chrono::duration<double>>(uptime).count();
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
  if (imp_->sample_reader && imp_->sample_reader->ReadNextPoint())
  {
    const liblas::Point &p = imp_->sample_reader->GetPoint();
    SamplePoint sample;
    sample.timestamp = p.GetTime();
    sample.sample = glm::dvec3(p.GetX(), p.GetY(), p.GetZ());
    sample.origin = glm::dvec3(0);
    sample.orientation = glm::dquat();
    sampleTrajectory(sample.origin, sample.orientation, sample.timestamp);
    imp_->samples_buffer.push_back(sample);

    if (imp_->first_sample_timestamp < 0)
    {
      imp_->first_sample_timestamp = sample.timestamp;
      imp_->first_sample_read_time = Clock::now();
    }
    return true;
  }
  return false;
}


bool SlamCloudLoader::sampleTrajectory(glm::dvec3 &position, glm::dquat &orientation, double timestamp)
{
  if (imp_->read_trajectory_point)
  {
    TrajectoryPoint traj_point;

    while (timestamp > imp_->trajectory_buffer[1].timestamp && imp_->read_trajectory_point(traj_point))
    {
      imp_->trajectory_buffer[0] = imp_->trajectory_buffer[1];
      imp_->trajectory_buffer[1] = traj_point;
    }

    if (imp_->trajectory_buffer[0].timestamp <= timestamp &&
        timestamp <= imp_->trajectory_buffer[1].timestamp &&
        imp_->trajectory_buffer[0].timestamp != imp_->trajectory_buffer[1].timestamp)
    {
      const double lerp = (timestamp - imp_->trajectory_buffer[0].timestamp) / (imp_->trajectory_buffer[1].timestamp - imp_->trajectory_buffer[0].timestamp);
      position = imp_->trajectory_buffer[0].origin + lerp * (imp_->trajectory_buffer[1].origin - imp_->trajectory_buffer[0].origin);
      orientation = glm::slerp(imp_->trajectory_buffer[0].orientation, imp_->trajectory_buffer[1].orientation, lerp);
      return true;
    }
  }
  return false;
}
