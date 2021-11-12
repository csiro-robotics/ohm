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


int DataSource::validateOptions()
{
  return 0;
}


void DataSource::printConfiguration(std::ostream &out)
{
  options_->print(out);
}
}  // namespace ohmapp
