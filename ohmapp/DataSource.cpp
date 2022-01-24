// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "DataSource.h"

#include <ohm/Logger.h>

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

using namespace ohm;

namespace ohmapp
{
DataSource::Options::~Options() = default;

void DataSource::Options::configure(cxxopts::Options &parser)
{
  cxxopts::OptionAdder adder = parser.add_options("Input");
  configure(adder);
}


void DataSource::Options::configure(cxxopts::OptionAdder &adder)
{
  // clang-format off
  adder
    ("point-limit", "Limit the number of points loaded.", optVal(point_limit))
    ("start-time", "Only process points time stamped later than the specified time.", optVal(start_time))
    ("time-limit", "Limit the elapsed time in the LIDAR data to process (seconds). Measured relative to the first data sample.", optVal(time_limit))
    ("stats", "Stats collection mode: [off,console,csv]", optVal(stats_mode)->implicit_value("console"))
    ;
  // clang-format on
}


void DataSource::Options::print(std::ostream &out)
{
  if (point_limit)
  {
    out << "Maximum point: " << point_limit << '\n';
  }

  if (start_time > 0)
  {
    out << "Process from timestamp: " << start_time << '\n';
  }

  if (time_limit > 0)
  {
    out << "Process to timestamp: " << time_limit << '\n';
  }
}


DataSource::DataSource(std::unique_ptr<Options> &&options)
  : options_(std::move(options))
{}


void DataSource::configure(cxxopts::Options &parser)
{
  options_->configure(parser);
}


DataSource::~DataSource() = default;


std::string DataSource::getFileExtension(const std::string &file)
{
  const size_t last_dot = file.find_last_of('.');
  if (last_dot != std::string::npos)
  {
    return file.substr(last_dot + 1);
  }

  return "";
}


void DataSource::setSamplesOnly(bool samples_only)
{
  samples_only_ = samples_only;
}


int DataSource::validateOptions()
{
  return 0;
}


void DataSource::printConfiguration(std::ostream &out)
{
  options_->print(out);
}


unsigned DataSource::addBatchStats(const Stats &stats, Stats &global_stats, std::vector<Stats> &windowed_stats_buffer,
                                   unsigned windowed_stats_buffer_size, unsigned windowed_stats_buffer_next)
{
  // Update global stats.
  global_stats.data_time_start = std::min(stats.data_time_start, global_stats.data_time_start);
  global_stats.data_time_end = std::max(stats.data_time_end, global_stats.data_time_end);
  global_stats.process_time_start = std::min(stats.process_time_start, global_stats.process_time_start);
  global_stats.process_time_end = std::max(stats.process_time_end, global_stats.process_time_end);
  global_stats.ray_length_minimum = std::min(stats.ray_length_minimum, global_stats.ray_length_minimum);
  global_stats.ray_length_maximum = std::max(stats.ray_length_maximum, global_stats.ray_length_maximum);
  global_stats.ray_length_total += stats.ray_length_total;
  global_stats.ray_count += stats.ray_count;

  // Update window
  if (windowed_stats_buffer.size() >= windowed_stats_buffer_size)
  {
    windowed_stats_buffer[windowed_stats_buffer_next] = stats;
    windowed_stats_buffer_next = (windowed_stats_buffer_next + 1) % windowed_stats_buffer_size;
  }
  else
  {
    windowed_stats_buffer.emplace_back(stats);
    ++windowed_stats_buffer_next;
  }

  return windowed_stats_buffer_next;
}


std::ostream &DataSource::Stats::writeCsvHeader(std::ostream &out)
{
  const char delim = ',';
  out << "process_time_start" << delim << "process_time_end" << delim << "data_time_start" << delim << "data_time_end"
      << delim << "ray_count" << delim << "ray_length_minimum" << delim << "ray_length_maximum" << delim
      << "ray_length_average" << delim << "rays_per_second_data" << delim << "rays_per_second_process\n";
  return out;
}
}  // namespace ohmapp


std::ostream &operator<<(std::ostream &out, const ohmapp::DataSource::Stats &stats)
{
  const char delim = ',';
  const auto precision = out.precision();
  out.precision(std::numeric_limits<double>::max_digits10);
  out << stats.process_time_start << delim << stats.process_time_end << delim << stats.data_time_start << delim
      << stats.data_time_end << delim << stats.ray_count << delim << stats.ray_length_minimum << delim
      << stats.ray_length_maximum << delim << stats.rayLengthAverage() << delim << stats.dataRaysPerSecond() << delim
      << stats.processRaysPerSecond();
  out.precision(precision);
  return out;
}


std::ostream &operator<<(std::ostream &out, ohmapp::DataSource::StatsMode mode)
{
  using StatsMode = ohmapp::DataSource::StatsMode;
  switch (mode)
  {
  case StatsMode::Off:
    out << "off";
    break;
  case StatsMode::Console:
    out << "console";
    break;
  case StatsMode::Csv:
    out << "csv";
    break;
  default:
    out << "unknown";
    break;
  }

  return out;
}


std::istream &operator>>(std::istream &in, ohmapp::DataSource::StatsMode &mode)
{
  using StatsMode = ohmapp::DataSource::StatsMode;
  std::string mode_str;
  in >> mode_str;
  if (mode_str == "console")
  {
    mode = StatsMode::Console;
  }
  else if (mode_str == "csv")
  {
    mode = StatsMode::Csv;
  }
  else if (mode_str == "off")
  {
    mode = StatsMode::Off;
  }
  else
  {
    in.setstate(std::istream::failbit);
  }

  return in;
}
