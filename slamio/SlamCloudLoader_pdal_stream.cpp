//
// author Kazys Stepanas
//
#include "SlamCloudLoader.h"

#ifdef _MSC_VER
// Disable PDAL warnings:
// - 4996: strerror not secure
// - 4250: inheritance via dominance.
#pragma warning(disable : 4996 4250)
#endif  // _MSC_VER

#include <ohmutil/SafeIO.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BpfReader.hpp>
#include <pdal/io/LasReader.hpp>

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

  class PointStream : public pdal::StreamPointTable
  {
  public:
    PointStream(size_t buffer_capacity)
      : pdal::StreamPointTable(_layout, buffer_capacity)
    {
      // Register for the data we are interested in
      _buffers[0].reserve(buffer_capacity);
      _buffers[1].reserve(buffer_capacity);
      // Start with flipping allowed.
      _flip_wait.notify_one();
    }

    /// Called when execute() is started.  Typically used to set buffer size
    /// when all dimensions are known.
    void finalize() override
    {
      if (!_layout.finalized())
      {
        // Validate the dimensions.
        bool have_required_dimensions = true;
        if (!_layout.hasDim(pdal::Dimension::Id::X) || !_layout.hasDim(pdal::Dimension::Id::Y) ||
            !_layout.hasDim(pdal::Dimension::Id::Z))
        {
          have_required_dimensions = false;
        }

        // Resolve time dimension.
        auto dim = pdal::Dimension::Id::Unknown;
        // First try resolve by name.
        const std::string time_dim_names[] = { "time", "timestamp" };
        for (const auto &time_name : time_dim_names)
        {
          dim = _layout.findDim(time_name);
          if (dim != pdal::Dimension::Id::Unknown)
          {
            break;
          }
        }

        if (dim == pdal::Dimension::Id::Unknown)
        {
          // Not found by name. Try resolve by Dimension ID
          // Not found by name.
          const pdal::Dimension::Id time_ids[] = { pdal::Dimension::Id::GpsTime, pdal::Dimension::Id::InternalTime,
                                                   pdal::Dimension::Id::OffsetTime };
          for (const auto &time_dim : time_ids)
          {
            if (_layout.hasDim(time_dim))
            {
              dim = time_dim;
              break;
            }
          }
        }

        if (dim != pdal::Dimension::Id::Unknown)
        {
          // Resolved time field.
          _time_dimension = dim;
        }
        else
        {
          have_required_dimensions = false;
        }

        _valid_dimensions = have_required_dimensions;

        _layout.finalize();
        pdal::StreamPointTable::finalize();
      }
    }

    bool nextPoint(glm::dvec4 *point)
    {
      std::unique_lock<std::mutex> guard(_buffer_mutex);
      const int read_buffer = 1 - _write_index;
      const unsigned read_index = _next_read;
      bool have_read = false;
      if (_next_read < _buffers[read_buffer].size())
      {
        *point = _buffers[read_buffer][read_index];
        ++_next_read;
        have_read = true;
      }
      guard.unlock();
      _flip_wait.notify_one();
      return have_read;
    }

    inline bool isValid() const { return _valid_dimensions; }

    /// True once we have data available for reading.
    inline bool haveData() const { return _have_data; }

    /// True once loading has been completed (@c markLoadComplete() called) and the read index is at the end of the
    /// read buffer.
    inline bool done()
    {
      if (_loading_complete)
      {
        std::unique_lock<std::mutex> guard(_buffer_mutex);
        return _next_read >= _buffers[1 - _write_index].size();
      }
      return false;
    }

    /// Mark for abort. No more data points are stored.
    inline void abort()
    {
      _abort = true;
      _flip_wait.notify_all();
    }

    /// Mark loading as done: only to be called from the loading thread.
    inline void markLoadComplete() { _loading_complete = true; }

  protected:
    // Not supported
    char *getPoint(pdal::PointId /* idx */) override { return nullptr; }

    void setFieldInternal(pdal::Dimension::Id dim, pdal::PointId idx, const void *val) override
    {
      if (!_abort)
      {
        auto &buffer = _buffers[_write_index];
        while (buffer.size() <= idx)
        {
          buffer.emplace_back();
        }

        auto &point = buffer[idx];

        switch (dim)
        {
        case pdal::Dimension::Id::X:
          point.x = *static_cast<const double *>(val);
          break;
        case pdal::Dimension::Id::Y:
          point.y = *static_cast<const double *>(val);
          break;
        case pdal::Dimension::Id::Z:
          point.z = *static_cast<const double *>(val);
          break;
        default:
          if (dim == _time_dimension)
          {
            point.w = *static_cast<const double *>(val);
          }
          break;
        }
      }
    }

    /// Called whenever the buffer capacity is filled before starting on the next block.
    void reset() override
    {
      if (!_abort)
      {
        std::unique_lock<std::mutex> guard(_buffer_mutex);
        const int read_buffer = 1 - _write_index;
        _flip_wait.wait(guard, [this, read_buffer]() { return _abort || _next_read >= _buffers[read_buffer].size(); });
        _write_index = read_buffer;
        _buffers[read_buffer].clear();
        _next_read = 0;
        _have_data = true;
      }
    }

  private:
    // Double buffer to allow background thread streaming.
    // Use w channel for timetstamp
    std::vector<glm::dvec4> _buffers[2];
    pdal::Dimension::Id _time_dimension{ pdal::Dimension::Id::GpsTime };
    std::atomic_uint _next_read{ 0 };
    std::atomic_int _write_index{ 0 };
    pdal::PointLayout _layout;
    std::mutex _buffer_mutex;
    std::condition_variable _flip_wait;
    std::atomic_bool _have_data{ false };
    std::atomic_bool _loading_complete{ false };
    std::atomic_bool _abort{ false };
    std::atomic_bool _valid_dimensions{ false };
  };
}  // namespace

struct SlamCloudLoaderDetail
{
  std::unique_ptr<pdal::StageFactory> pdal_factory = std::make_unique<pdal::StageFactory>();
  std::shared_ptr<pdal::Streamable> sample_reader;
  std::shared_ptr<pdal::Streamable> trajectory_reader;

  std::unique_ptr<PointStream> sample_stream;
  std::unique_ptr<PointStream> traj_stream;

  std::thread sample_thread;
  std::thread traj_thread;

  pdal::point_count_t sample_count{ 0 };
  pdal::point_count_t read_samples{ 0 };

  std::string sample_file_path;
  std::string trajectory_file_path;
  std::function<bool(TrajectoryPoint &point)> read_trajectory_point;
  std::ifstream trajectory_file;
  std::string traj_line;
  glm::dvec3 trajectory_to_sensor_offset = glm::dvec3(0);
  TrajectoryPoint trajectory_buffer[2] = { TrajectoryPoint{}, TrajectoryPoint{} };

  std::vector<SamplePoint> preload_points;
  SamplePoint next_sample = SamplePoint{};
  pdal::point_count_t read_count = 0;
  pdal::point_count_t preload_index = 0;

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

  std::shared_ptr<pdal::Streamable> createReader(pdal::StageFactory &factory,  // NOLINT(google-runtime-references)
                                                 const std::string &file_name)
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

    if (!reader)
    {
      std::cerr << "PDAL reader for " << reader_type << " not available" << std::endl;
      return nullptr;
    }

    if (!reader->pipelineStreamable())
    {
      std::cout << "PDAL reader for " << reader_type << " does not support streaming" << std::endl;
      return nullptr;
    }

    auto streamable_reader = std::dynamic_pointer_cast<pdal::Streamable>(reader);

    if (streamable_reader)
    {
      options.add("filename", file_name);
      reader->setOptions(options);
    }

    return streamable_reader;
  }


  pdal::point_count_t pointCount(std::shared_ptr<pdal::Streamable> reader)
  {
    // Doesn't seem to be a consistent way to read the point count. `Reader::count()` didn't work with las, but casting
    // and calling `LasReader::getNumPoints()` does.
#define TRY_PDAL_POINT_COUNT(T, func)                        \
  if (T *r = dynamic_cast<T *>(reader.get()))                \
  {                                                          \
    const auto count = r->func();                            \
    if (count < std::numeric_limits<decltype(count)>::max()) \
    {                                                        \
      return r->func();                                      \
    }                                                        \
  }
    TRY_PDAL_POINT_COUNT(pdal::BpfReader, numPoints);
    TRY_PDAL_POINT_COUNT(pdal::LasReader, getNumPoints);
    TRY_PDAL_POINT_COUNT(pdal::Reader, count);

    return 0;
  }
}  // namespace

SlamCloudLoader::SlamCloudLoader(bool real_time_mode)
  : imp_(new SlamCloudLoaderDetail)
{
  imp_->real_time_mode = real_time_mode;
}


SlamCloudLoader::~SlamCloudLoader()
{
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

  imp_->sample_stream = std::make_unique<PointStream>(10000);
  imp_->sample_reader->prepare(*imp_->sample_stream);
  imp_->sample_count = pointCount(imp_->sample_reader);

  imp_->sample_stream->finalize();
  if (!imp_->sample_stream->isValid())
  {
    std::cerr << "Unable to resolve time field in " << imp_->sample_file_path << std::endl;
    std::cerr << "Require point X, Y, Z and time fields" << std::endl;
    close();
    return false;
  }

  bool using_trajectory = false;
  if (text_trajectory)
  {
    using_trajectory = true;
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

      return true;
    };
  }
  else if (imp_->trajectory_reader)
  {
    using_trajectory = true;
    imp_->traj_stream = std::make_unique<PointStream>(5000);
    imp_->trajectory_reader->prepare(*imp_->traj_stream);

    imp_->traj_stream->finalize();
    if (!imp_->traj_stream->isValid())
    {
      std::cerr << "Unable to resolve required fields in " << imp_->trajectory_file_path << std::endl;
      std::cerr << "Require point X, Y, Z and time fields" << std::endl;
      close();
      return false;
    }

    imp_->read_trajectory_point = [this](TrajectoryPoint &point) -> bool {
      glm::dvec4 pt;
      if (imp_->traj_stream->nextPoint(&pt))
      {
        point.origin.x = pt.x;
        point.origin.y = pt.y;
        point.origin.z = pt.z;
        point.timestamp = pt.w;
        return true;
      }

      return false;
    };

    imp_->traj_thread = std::thread([this]() {  //
      imp_->trajectory_reader->execute(*imp_->traj_stream);
      imp_->traj_stream->markLoadComplete();
    });
  }

  imp_->sample_thread = std::thread([this]() {  //
    imp_->sample_reader->execute(*imp_->sample_stream);
    imp_->sample_stream->markLoadComplete();
  });

  if (using_trajectory)
  {
    // Prime the trajectory buffer.
    if (!text_trajectory && imp_->traj_stream)
    {
      // Wait for first trajectory data.
      for (int i = 0; i < 10000 && !imp_->traj_stream->haveData(); ++i)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

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
  }

  return true;
}


void SlamCloudLoader::close()
{
  if (imp_->sample_stream)
  {
    imp_->sample_stream->abort();
  }
  if (imp_->traj_stream)
  {
    imp_->traj_stream->abort();
  }

  if (imp_->sample_reader)
  {
    if (imp_->sample_thread.joinable())
    {
      imp_->sample_thread.join();
    }
    imp_->sample_reader = nullptr;
  }
  if (imp_->trajectory_reader)
  {
    if (imp_->traj_thread.joinable())
    {
      imp_->traj_thread.join();
    }
    imp_->trajectory_reader = nullptr;
  }

  imp_->sample_stream.release();
  imp_->traj_stream.release();

  imp_->sample_file_path.clear();
  imp_->trajectory_file_path.clear();
  imp_->read_trajectory_point = std::function<bool(TrajectoryPoint &)>();
  imp_->trajectory_file.close();
  // imp_->next_sample_read_index = imp_->samples_view_index = imp_->traj_view_index = 0u;
  imp_->read_count = imp_->sample_count = 0;
}


size_t SlamCloudLoader::numberOfPoints() const
{
  return imp_->sample_count;
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


void SlamCloudLoader::preload(size_t point_count)
{
  if (!imp_->sample_stream)
  {
    return;
  }

  if (point_count && numberOfPoints())
  {
    point_count = std::min(point_count, numberOfPoints());
  }

  const bool load_all = point_count == 0 || numberOfPoints() && point_count >= numberOfPoints();

  std::vector<SamplePoint> preload_data;
  pdal::point_count_t initial_read_count = imp_->read_count;
  preload_data.reserve(point_count);

  bool ok = true;
  SamplePoint sample;
  while (!imp_->sample_stream->done() && (preload_data.size() < point_count || load_all))
  {
    ok = nextPoint(sample.sample, &sample.origin, &sample.timestamp);
    preload_data.emplace_back(sample);
  }

  if (!ok)
  {
    // Not ok: remove the failed point.
    preload_data.pop_back();
  }

  std::swap(imp_->preload_points, preload_data);
  imp_->preload_index = 0;
  imp_->read_count = initial_read_count;
}


bool SlamCloudLoader::nextPoint(glm::dvec3 &sample, glm::dvec3 *origin, double *timestamp_out)
{
  if (loadPoint())
  {
    ++imp_->read_count;
    imp_->sample_count = std::max(imp_->sample_count, imp_->read_count);

    // Read next sample.
    const SamplePoint sample_point = imp_->next_sample;
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
  if (imp_->preload_index < imp_->preload_points.size())
  {
    imp_->next_sample = imp_->preload_points[imp_->preload_index++];
    return true;
  }
  else if (!imp_->preload_points.empty())
  {
    // Preload done. Release the memory for preload_points
    imp_->preload_points = std::vector<SamplePoint>();
    imp_->preload_index = 0;
  }

  if (!imp_->sample_stream->done())
  {
    SamplePoint sample_data{};

    glm::dvec4 sample{ 0 };

    bool have_read = imp_->sample_stream->nextPoint(&sample);
    while (!imp_->sample_stream->done() && !have_read)
    {
      std::this_thread::yield();
      have_read = imp_->sample_stream->nextPoint(&sample);
    }

    if (have_read)
    {
      sample_data.sample = glm::dvec3(sample);
      sample_data.timestamp = sample.w;
      sample_data.origin = glm::dvec3(0);

      sampleTrajectory(sample_data.origin, sample_data.sample, sample_data.timestamp);
      imp_->next_sample = sample_data;

      if (imp_->first_sample_timestamp < 0)
      {
        imp_->first_sample_timestamp = sample_data.timestamp;
        imp_->first_sample_read_time = Clock::now();
      }
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
  else
  {
    // Handle no trajectory file.
    position = sample;
  }

  return false;
}
