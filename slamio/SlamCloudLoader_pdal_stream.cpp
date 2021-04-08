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

#include <array>
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
const size_t kCloudStreamBufferSize = 10000u;
const size_t kTrajectoryStreamBufferSize = 10000u;
struct TrajectoryPoint
{
  double timestamp;
  glm::dvec3 origin;
};

using Clock = std::chrono::high_resolution_clock;

class PointStream : public pdal::StreamPointTable
{
public:
  explicit PointStream(size_t buffer_capacity, bool ignore_missing_timefield = false)
    : pdal::StreamPointTable(layout_, buffer_capacity)
    , ignore_missing_timefield_(ignore_missing_timefield)
  {
    // Register for the data we are interested in
    buffers_[0].reserve(buffer_capacity);
    buffers_[1].reserve(buffer_capacity);
    // Start with flipping allowed.
    flip_wait_.notify_one();
  }

  /// Called when execute() is started.  Typically used to set buffer size
  /// when all dimensions are known.
  void finalize() override
  {
    if (!layout_.finalized())
    {
      // Validate the dimensions.
      bool have_required_dimensions = true;

      for (int i = 0; i < 3; ++i)
      {
        const auto *pos_dim = layout_.dimDetail(pdal::Dimension::Id(i + int(pdal::Dimension::Id::X)));
        if (pos_dim)
        {
          position_channel_types_[i] = pos_dim->type();
          have_required_dimensions =
            have_required_dimensions && (position_channel_types_[i] == pdal::Dimension::Type::Double ||
                                         position_channel_types_[i] == pdal::Dimension::Type::Float);
        }
        else
        {
          have_required_dimensions = false;
        }
      }

      have_colour_ = true;

      for (int i = 0; i < 3; ++i)
      {
        const auto *colour_dim = layout_.dimDetail(pdal::Dimension::Id(i + int(pdal::Dimension::Id::Red)));
        if (colour_dim)
        {
          colour_channel_types_[i] = colour_dim->type();
          have_colour_ = have_colour_ && (colour_channel_types_[i] == pdal::Dimension::Type::Unsigned8 ||
                                          colour_channel_types_[i] == pdal::Dimension::Type::Unsigned16 ||
                                          colour_channel_types_[i] == pdal::Dimension::Type::Unsigned32);
        }
        else
        {
          have_colour_ = false;
        }
      }

      const auto *intensity_dim = layout_.dimDetail(pdal::Dimension::Id::Intensity);
      if (intensity_dim)
      {
        intensity_channel_type_ = intensity_dim->type();
        have_intensity_ = (intensity_channel_type_ == pdal::Dimension::Type::Unsigned8 ||
                           intensity_channel_type_ == pdal::Dimension::Type::Unsigned16 ||
                           intensity_channel_type_ == pdal::Dimension::Type::Unsigned32 ||
                           intensity_channel_type_ == pdal::Dimension::Type::Double ||
                           intensity_channel_type_ == pdal::Dimension::Type::Float);
      }

      // Resolve time dimension.
      auto dim = pdal::Dimension::Id::Unknown;
      // First try resolve by Dimension ID
      const std::array<pdal::Dimension::Id, 3> time_ids = { pdal::Dimension::Id::GpsTime,
                                                            pdal::Dimension::Id::InternalTime,
                                                            pdal::Dimension::Id::OffsetTime };
      for (const auto &time_dim : time_ids)
      {
        if (layout_.hasDim(time_dim))
        {
          dim = time_dim;
          break;
        }
      }

      // Not found. Try resolve by name.
      if (dim == pdal::Dimension::Id::Unknown)
      {
        const std::array<const std::string, 2> time_dim_names = { "time", "timestamp" };
        for (const auto &time_name : time_dim_names)
        {
          dim = layout_.findDim(time_name);
          if (dim != pdal::Dimension::Id::Unknown)
          {
            break;
          }
        }
      }

      if (dim != pdal::Dimension::Id::Unknown)
      {
        have_timefield_ = true;
        time_dimension_ = dim;
        time_channel_type_ = layout_.dimDetail(dim)->type();
      }
      else
      {
        have_timefield_ = false;
        if (!ignore_missing_timefield_)
        {
          have_required_dimensions = false;
        }
      }

      valid_dimensions_ = have_required_dimensions;

      layout_.finalize();
      pdal::StreamPointTable::finalize();
    }
  }

  bool nextPoint(SamplePoint *sample)
  {
    std::unique_lock<std::mutex> guard(buffer_mutex_);
    const int read_buffer = 1 - write_index_;
    const unsigned read_index = next_read_;
    // Ensure colour pointer is valid, pointing to a stack variable if null.
    bool have_read = false;
    if (next_read_ < buffers_[read_buffer].size())
    {
      *sample = buffers_[read_buffer][read_index];
      ++next_read_;
      have_read = true;
    }
    guard.unlock();
    flip_wait_.notify_one();
    return have_read;
  }

  inline bool isValid() const { return valid_dimensions_; }

  /// True once we have data available for reading.
  inline bool haveData() const { return have_data_; }

  /// True once loading has been completed (@c markLoadComplete() called) and the read index is at the end of the
  /// read buffer.
  inline bool done()
  {
    if (loading_complete_)
    {
      std::unique_lock<std::mutex> guard(buffer_mutex_);
      return next_read_ >= buffers_[1 - write_index_].size();
    }
    return false;
  }

  /// Mark for abort. No more data points are stored.
  inline void abort()
  {
    abort_ = true;
    flip_wait_.notify_all();
  }

  /// Mark loading as done: only to be called from the loading thread.
  inline void markLoadComplete() { loading_complete_ = true; }

  inline bool hasTimestamp() const { return have_timefield_; }
  inline bool hasColour() const { return have_colour_; }
  inline bool hasIntensity() const { return have_intensity_; }

protected:
  // Not supported
  char *getPoint(pdal::PointId /* idx */) override { return nullptr; }

  void setFieldInternal(pdal::Dimension::Id dim, pdal::PointId idx, const void *val) override
  {
    if (!abort_)
    {
      auto &point_buffer = buffers_[write_index_];
      while (point_buffer.size() <= idx)
      {
        point_buffer.emplace_back(SamplePoint{});
      }

      auto &sample = point_buffer[idx];

      switch (dim)
      {
      case pdal::Dimension::Id::X:
      case pdal::Dimension::Id::Y:
      case pdal::Dimension::Id::Z: {
        const int pos_index = int(dim) - int(pdal::Dimension::Id::X);
        if (position_channel_types_[pos_index] == pdal::Dimension::Type::Double)
        {
          sample.sample[pos_index] = *static_cast<const double *>(val);
        }
        else
        {
          sample.sample[pos_index] = double(*static_cast<const float *>(val));
        }
      }
      break;
      case pdal::Dimension::Id::Red:
      case pdal::Dimension::Id::Green:
      case pdal::Dimension::Id::Blue: {
        const int colour_index = int(dim) - int(pdal::Dimension::Id::Red);
        switch (colour_channel_types_[colour_index])
        {
        case pdal::Dimension::Type::Signed8:  // Ignore sign
        case pdal::Dimension::Type::Unsigned8:
          sample.colour[colour_index] = *static_cast<const uint8_t *>(val);
          break;
        case pdal::Dimension::Type::Signed16:  // Ignore sign
        case pdal::Dimension::Type::Unsigned16:
          sample.colour[colour_index] =
            uint8_t(255.0 * double(*static_cast<const uint16_t *>(val)) / double(std::numeric_limits<uint16_t>::max()));
          break;
        case pdal::Dimension::Type::Signed32:  // Ignore sign
        case pdal::Dimension::Type::Unsigned32:
          sample.colour[colour_index] =
            uint8_t(255.0 * double(*static_cast<const uint32_t *>(val)) / double(std::numeric_limits<uint32_t>::max()));
          break;
        default:
          break;
        }
      }
      break;
      case pdal::Dimension::Id::Intensity:
        switch (intensity_channel_type_)
        {
        case pdal::Dimension::Type::Unsigned8:
          sample.intensity = float(*static_cast<const uint8_t *>(val));
          break;
        case pdal::Dimension::Type::Unsigned16:
          sample.intensity = float(*static_cast<const uint16_t *>(val));
          break;
        case pdal::Dimension::Type::Unsigned32:
          sample.intensity = float(*static_cast<const uint32_t *>(val));
          break;
        case pdal::Dimension::Type::Float:
          sample.intensity = *static_cast<const float *>(val);
          break;
        case pdal::Dimension::Type::Double:
          sample.intensity = float(*static_cast<const double *>(val));
          break;
        default:
          break;
        }
        break;
      default:
        if (have_timefield_ && dim == time_dimension_)
        {
          if (time_channel_type_ == pdal::Dimension::Type::Double)
          {
            sample.timestamp = *static_cast<const double *>(val);
          }
          else
          {
            sample.timestamp = double(*static_cast<const float *>(val));
          }
        }
        break;
      }
    }
  }

  /// Called whenever the buffer capacity is filled before starting on the next block.
  void reset() override
  {
    if (!abort_)
    {
      std::unique_lock<std::mutex> guard(buffer_mutex_);
      const int read_buffer = 1 - write_index_;
      flip_wait_.wait(guard, [this, read_buffer]() { return abort_ || next_read_ >= buffers_[read_buffer].size(); });
      write_index_ = read_buffer;
      buffers_[read_buffer].clear();
      next_read_ = 0;
      have_data_ = true;
    }
  }

private:
  // Double buffer to allow background thread streaming.
  // Use w channel for timetstamp
  std::array<std::vector<SamplePoint>, 2> buffers_;
  std::array<pdal::Dimension::Type, 3> position_channel_types_{};
  std::array<pdal::Dimension::Type, 3> colour_channel_types_{};
  pdal::Dimension::Id time_dimension_{ pdal::Dimension::Id::GpsTime };
  pdal::Dimension::Type time_channel_type_{};
  pdal::Dimension::Id intensity_dimension_{ pdal::Dimension::Id::GpsTime };
  pdal::Dimension::Type intensity_channel_type_{};
  std::atomic_uint next_read_{ 0 };
  std::atomic_int write_index_{ 0 };
  pdal::PointLayout layout_;
  std::mutex buffer_mutex_;
  std::condition_variable flip_wait_;
  std::atomic_bool have_data_{ false };
  std::atomic_bool loading_complete_{ false };
  std::atomic_bool abort_{ false };
  std::atomic_bool valid_dimensions_{ false };
  bool have_timefield_{ false };
  bool have_intensity_{ false };
  bool have_colour_{ false };
  bool ignore_missing_timefield_{ false };
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
  std::array<TrajectoryPoint, 2> trajectory_buffer = { TrajectoryPoint{}, TrajectoryPoint{} };

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

std::shared_ptr<pdal::Streamable> createReader(pdal::StageFactory &factory, const std::string &file_name)
{
  const std::string ext = getFileExtension(file_name);
  std::string reader_type;
  pdal::Options options;

  reader_type = ext;

  if (ext == "laz")
  {
    reader_type = "las";
    options.add("compression", "EITHER");
  }
  else if (ext == "xyz")
  {
    reader_type = "text";
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

template <typename T, typename Func>
bool tryPdalPointCount(std::shared_ptr<pdal::Streamable> reader, Func func, pdal::point_count_t &point_count)
{
  if (T *r = dynamic_cast<T *>(reader.get()))
  {
    const auto count = (r->*func)();
    if (count < std::numeric_limits<decltype(count)>::max())
    {
      point_count = count;
      return true;
    }
  }
  return false;
}


pdal::point_count_t pointCount(std::shared_ptr<pdal::Streamable> reader)
{
  // Doesn't seem to be a consistent way to read the point count. `Reader::count()` didn't work with las, but casting
  // and calling `LasReader::getNumPoints()` does.

  pdal::point_count_t point_count = 0;
  if (tryPdalPointCount<pdal::BpfReader>(reader, &pdal::BpfReader::numPoints, point_count))
  {
    return point_count;
  }

  if (tryPdalPointCount<pdal::LasReader>(reader, &pdal::LasReader::getNumPoints, point_count))
  {
    return point_count;
  }

  if (tryPdalPointCount<pdal::Reader>(reader, &pdal::Reader::count, point_count))
  {
    return point_count;
  }

  return point_count;
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
    if (traj_ext == "txt")
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

  const bool allow_missing_time_field = trajectory_file_path == nullptr || !trajectory_file_path[0];
  imp_->sample_stream = std::make_unique<PointStream>(kCloudStreamBufferSize, allow_missing_time_field);
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
      const int read_item_count = 8;
      // NOLINTNEXTLINE(cert-err34-c)
      return sscanf_s(imp_->traj_line.c_str(), "%lg %lg %lg %lg %lg %lg %lg %lg", &point.timestamp, &point.origin.x,
                      &point.origin.y, &point.origin.z, &orientation.w, &orientation.x, &orientation.y,
                      &orientation.z) == read_item_count;
    };
  }
  else if (imp_->trajectory_reader)
  {
    using_trajectory = true;
    imp_->traj_stream = std::make_unique<PointStream>(kTrajectoryStreamBufferSize);
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
      SamplePoint sample;
      if (imp_->traj_stream->nextPoint(&sample))
      {
        point.origin = sample.sample;
        point.timestamp = sample.timestamp;
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
      const int wait_loop_limit = 10000;
      for (int i = 0; i < wait_loop_limit && !imp_->traj_stream->haveData(); ++i)
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

  imp_->sample_stream.reset();
  imp_->traj_stream.reset();

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


bool SlamCloudLoader::hasTimestamp() const
{
  return imp_->sample_stream && imp_->sample_stream->hasTimestamp() &&
         (!imp_->traj_stream || imp_->traj_stream->hasTimestamp());
}


bool SlamCloudLoader::hasOrigin() const
{
  return trajectoryFileIsOpen();
}


bool SlamCloudLoader::hasIntensity() const
{
  return imp_->sample_stream && imp_->sample_stream->hasIntensity();
}


bool SlamCloudLoader::hasColour() const
{
  return imp_->sample_stream && imp_->sample_stream->hasColour();
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
  SamplePoint sample{};
  while (!imp_->sample_stream->done() && (preload_data.size() < point_count || load_all))
  {
    ok = nextPoint(sample);
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


bool SlamCloudLoader::nextPoint(SamplePoint &sample)
{
  if (loadPoint())
  {
    ++imp_->read_count;
    imp_->sample_count = std::max(imp_->sample_count, imp_->read_count);

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


bool SlamCloudLoader::loadPoint()
{
  if (imp_->preload_index < imp_->preload_points.size())
  {
    imp_->next_sample = imp_->preload_points[imp_->preload_index++];
    return true;
  }
  if (!imp_->preload_points.empty())
  {
    // Preload done. Release the memory for preload_points
    imp_->preload_points = std::vector<SamplePoint>();
    imp_->preload_index = 0;
  }

  if (!imp_->sample_stream->done())
  {
    SamplePoint sample_data{};

    bool have_read = imp_->sample_stream->nextPoint(&sample_data);
    while (!imp_->sample_stream->done() && !have_read)
    {
      std::this_thread::yield();
      have_read = imp_->sample_stream->nextPoint(&sample_data);
    }

    if (have_read)
    {
      sampleTrajectory(sample_data.origin, sample_data.sample, sample_data.timestamp);
      sample_data.origin += imp_->trajectory_to_sensor_offset;
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
