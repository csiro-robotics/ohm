// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapHarness.h"

#include "DataSource.h"

#include <ohmutil/OhmUtil.h>

#include <glm/vec4.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

using namespace ohm;

namespace ohmapp
{
using Clock = std::chrono::high_resolution_clock;


MapHarness::OutputOptions::OutputOptions() = default;
MapHarness::OutputOptions::~OutputOptions() = default;


void MapHarness::OutputOptions::configure(cxxopts::Options &parser)
{
  cxxopts::OptionAdder adder = parser.add_options("Output");
  configure(adder);
}


void MapHarness::OutputOptions::configure(cxxopts::OptionAdder &adder)
{
  // clang-format off
  adder
    ("cloud-colour", "Colour for points in the saved cloud (if saving).", optVal(cloud_colour))
    ("output", "Output base name. Saved file paths are derived from this string adding appropriate file extensions.", optVal(base_name))
    ("q,quiet", "Run in quiet mode. Suppresses progress messages.", optVal(quiet))
    ("save-cloud", "Save a point cloud after population?", optVal(save_cloud))
    ("save-info", "Write information on how the map was generated to text file?", optVal(save_info))
    ("save-map", "Save the map object after population?", optVal(save_map))
    ("trace", "Enable debug tracing to the given file name to generate a 3es file. High performance impact.", optVal(trace)->implicit_value("trace"))
    ("trace-final", "Only output final map in trace.", optVal(trace_final))
  ;
  // clang-format on
}


void MapHarness::OutputOptions::print(std::ostream &out)
{
  out << "Output base path: " << base_name << '\n';
  std::string save_items;
  if (save_map)
  {
    save_items += " map";
  }
  if (save_cloud)
  {
    save_items += " cloud";
    if (glm::all(glm::greaterThanEqual(cloud_colour, glm::vec3(0.0f))))
    {
      out << "Save cloud colour: " << cloud_colour << '\n';
    }
  }
  if (!save_items.empty())
  {
    out << "Will save: " << save_items << '\n';
  }
  if (!trace.empty())
  {
#ifdef TES_ENABLE
    out << "3es trace file: " << trace << (trace_final ? "(final only)\n" : "\n");
#else   //
    out << "3es trace ignore (not compiled)\n";
#endif  // TES_ENABLE
  }
}


MapHarness::MapOptions::~MapOptions() = default;


void MapHarness::MapOptions::configure(cxxopts::Options &parser)
{
  cxxopts::OptionAdder adder = parser.add_options("Map");
  configure(adder);
}


void MapHarness::MapOptions::configure(cxxopts::OptionAdder &adder)
{
  // clang-format off
  adder
    ("resolution", "The voxel resolution of the generated map.", optVal(resolution))
  ;
  // clang-format on
}


void MapHarness::MapOptions::print(std::ostream &out)
{
  out << "Map resolution: " << resolution << '\n';
}


MapHarness::Options::Options()
{
  output_ = std::make_unique<MapHarness::OutputOptions>();
  map_ = std::make_unique<MapHarness::MapOptions>();
}


MapHarness::Options::~Options() = default;


void MapHarness::Options::configure(cxxopts::Options &parser)
{
  parser.add_options()("h,help", "Display this help.");
  output_->configure(parser);
  map_->configure(parser);

  if (!positional_args.empty())
  {
    parser.parse_positional(positional_args);
  }
}


void MapHarness::Options::print(std::ostream &out)
{
  out.imbue(std::locale(""));
  out << std::boolalpha;
  output_->print(out);
  map_->print(out);
}


MapHarness::MapHarness(std::unique_ptr<Options> &&options, std::shared_ptr<DataSource> data_source)
  : options_(std::move(options))
  , data_source_(data_source)
{}


MapHarness::~MapHarness() = default;


int MapHarness::parseCommandLineOptions(int argc, const char *const *argv)
{
  cxxopts::Options parser(argv[0]);
  configureOptions(parser);
  if (on_configure_options_callback_)
  {
    on_configure_options_callback_();
  }

  int result = 0;
  try
  {
    cxxopts::ParseResult parsed = parser.parse(argc, argv);
    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << parser.help(options_->default_help_sections) << std::endl;
      quit_level_ = maxQuitLevel();
      return 0;
    }

    result = validateOptions(parsed);
  }
  catch (const cxxopts::OptionException &e)
  {
    std::cerr << "Argument error\n" << e.what() << std::endl;
    result = -1;
  }

  return result;
}


int MapHarness::validateOptions(const cxxopts::ParseResult &parsed)
{
  (void)parsed;
  data_source_->validateOptions();

  // Generate output name based on input if not specified.
  if (options_->output().base_name.empty())
  {
    options_->output().base_name = data_source_->sourceName();
  }

#ifdef TES_ENABLE
  if (!options().output().trace.empty())
  {
    if (DataSource::getFileExtension(options().output().trace) != "3es")
    {
      options().output().trace += ".3es";
    }
  }
#endif  // TES_ENABLE

  return 0;
}


int MapHarness::run()
{
  int result_code = 0;

  if (quit_level_)
  {
    // Nothing to do.
    return result_code;
  }

#ifdef TES_ENABLE
  if (!options().output().trace.empty())
  {
    trace_ = std::make_unique<ohm::Trace>(options().output().trace.c_str());
  }
#endif  //  TES_ENABLE

  progress_.setDisplayFunction(
    std::bind(&MapHarness::displayProgress, this, std::placeholders::_1, std::placeholders::_2));

  std::ofstream info_file_out;
  std::array<std::ostream *, 2> info_streams = { nullptr };

  if (!quiet())
  {
    info_streams[0] = &std::cout;
  }

  if (options_->output().save_info)
  {
    const std::string output_txt = options_->output().base_name + ".txt";
    info_file_out.open(output_txt.c_str());
    info_streams[1] = &info_file_out;
  }

  result_code = prepareForRun();
  if (result_code)
  {
    tearDown();
    return result_code;
  }

  for (auto *out : info_streams)
  {
    if (out)
    {
      data_source_->options().print(*out);
      options_->print(*out);
    }
  }

  uint64_t predicted_point_count = 0;
  data_source_->prepareForRun(predicted_point_count, options_->output().base_name);

  if (on_start_callback_)
  {
    on_start_callback_();
  }

  logutil::info("Populating map\n");
  display_stats_in_progress_ = data_source_->options().stats_mode != DataSource::StatsMode::Off;
  progress_.beginProgress(ProgressMonitor::Info(predicted_point_count));
  progress_.startThread();

  const Clock::time_point start_time = Clock::now();
  data_source_->run(
    std::bind(&MapHarness::processBatch, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5, std::placeholders::_6),
    quitLevelPtr());
  progress_.endProgress();
  progress_.pause();
  display_stats_in_progress_ = false;
  finaliseMap();
  const Clock::time_point end_time = Clock::now();

  const double time_range = data_source_->processedTimeRange();
  const uint64_t processed_count = data_source_->processedPointCount();
  const double processing_time_sec =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() * 1e-3;

  if (quit_level_ < 2)
  {
    for (auto *out : info_streams)
    {
      if (out)
      {
        const double rtf_inv = (processing_time_sec > 0 && time_range > 0) ? time_range / processing_time_sec : 0.0;
        const double rtf = (rtf_inv) ? 1.0 / rtf_inv : 0;
        *out << "Sample count: " << processed_count << '\n';
        *out << "Data time: " << time_range << '\n';
        *out << "Total processing time: " << end_time - start_time << '\n';
        *out << "Realtime Factor: " << rtf << '\n';
        *out << "RTF inverse: " << rtf_inv << '\n';
        *out << "Average samples/sec: "
             << unsigned((processing_time_sec > 0) ? processed_count / processing_time_sec : 0.0) << '\n';
        // *out << "Memory (approx): " << map.calculateApproximateMemory() / (1024.0 * 1024.0) << " MiB\n";
        *out << std::flush;

        if (data_source_->options().stats_mode != DataSource::StatsMode::Off)
        {
          *out << "Ray length minimum: " << data_source_->globalStats().ray_length_minimum << '\n';
          *out << "Ray length maximum: " << data_source_->globalStats().ray_length_maximum << '\n';
          *out << "Ray length average: " << data_source_->globalStats().rayLengthAverage() << '\n';
        }
      }
    }
  }

  result_code = serialise();
  progress_.joinThread();
  tearDown();

  return 0;
}


void MapHarness::configureOptions(cxxopts::Options &parser)
{
  data_source_->configure(parser);
  options_->configure(parser);
}


int MapHarness::serialise()
{
  int result_code = 0;
  if (options_->output().save_map && quit_level_ < 2)
  {
    result_code = saveMap(options_->output().base_name);
    if (result_code)
    {
      return result_code;
    }
  }

  if (options_->output().save_cloud && quit_level_ < 2)
  {
    const std::string output_ply = options_->output().base_name + "_cloud.ply";
    result_code = saveCloud(output_ply);
    if (result_code)
    {
      return result_code;
    }
  }

  return result_code;
}


void MapHarness::displayProgress(const ProgressMonitor::Progress &progress, bool final)
{
  if (!quiet())
  {
    const double elapsed_sec = data_source_->processedTimeRange();
    const auto sec = unsigned(std::trunc(elapsed_sec));
    const auto ms = unsigned((elapsed_sec - std::trunc(elapsed_sec)) * 1000.0);

    std::ostringstream out;
    out.imbue(std::locale(""));
    out << '\r';

    if (!progress.info.info.empty())
    {
      out << progress.info.info << " : ";
    }

    out << std::setfill(' ') << std::setw(6) << sec << '.' << std::setfill('0') << std::setw(3) << ms << "s : ";

    const int fill_padding = 3;
    const auto fill_width = std::min(std::numeric_limits<decltype(progress.progress)>::digits10,
                                     std::numeric_limits<uint32_t>::digits10 + fill_padding);
    out << std::setfill(' ') << std::setw(fill_width) << progress.progress;
    if (progress.info.total)
    {
      out << " / " << std::setfill(' ') << std::setw(fill_width) << progress.info.total;
    }

    if (display_stats_in_progress_)
    {
      out << " ";
      // Show stats
      const DataSource::Stats &stats = data_source_->windowedStats();
      const double rays_per_second = stats.processRaysPerSecond();
      const uint64_t integer_rays_per_second = uint32_t(std::round(rays_per_second));
      out << std::setfill(' ') << std::setw(fill_width) << integer_rays_per_second << " rays/s ";
      out.precision(3);
      out << "avg len: " << std::fixed << stats.rayLengthAverage() << "     ";
    }

    if (final)
    {
      out << '\n';
    }
    logutil::info(out.str());
  }
}
}  // namespace ohmapp
