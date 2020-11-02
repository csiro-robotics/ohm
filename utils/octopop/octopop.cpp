// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
//
// This is the program file for populating an Octomap occupancy map using SLAM data.
// The main entry point is near the bottom of the file with additional option parsing and support functions above that.
//
#include "OctoPopConfig.h"

#include <glm/glm.hpp>

#include <slamio/SlamCloudLoader.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/Options.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/ProgressMonitor.h>
#include <ohmutil/ScopedTimeDisplay.h>

#include <octomap/octomap.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <thread>

namespace
{
/// The clock used to report process timing.
using Clock = std::chrono::high_resolution_clock;

/// Quit level/flag. Initial quit will just stop populating, but still save out. Multiple increments will quit saving.
int g_quit = 0;

/// Control-C capture.
void onSignal(int arg)
{
  if (arg == SIGINT || arg == SIGTERM)
  {
    ++g_quit;
  }
}

/// Parsed command line options.
struct Options
{
  std::string cloud_file;
  std::string trajectory_file;
  std::string output_base_name;
  glm::dvec3 sensor_offset = glm::dvec3(0.0);
  uint64_t point_limit = 0;
  int64_t preload_count = 0;
  double start_time = 0;
  double time_limit = 0;
  double resolution = 0.25;
  float prob_hit = 0.9f;
  float prob_miss = 0.49f;
  float prob_thresh = 0.5f;
  glm::vec2 prob_range = glm::vec2(0, 0);
  glm::vec3 cloud_colour = glm::vec3(0);
  bool serialise = true;
  bool save_info = false;
  bool non_lazy_eval = false;
  bool collapse = false;
  bool quiet = false;

  /// A helper to print configured options to (multiple) output stream.
  void print(std::ostream **out) const;
};


void Options::print(std::ostream **out) const
{
  while (*out)
  {
    **out << "Cloud: " << cloud_file;
    if (!trajectory_file.empty())
    {
      **out << " + " << trajectory_file << '\n';
    }
    else
    {
      **out << " (no trajectory)\n";
    }
    if (preload_count)
    {
      **out << "Preload: ";
      if (preload_count < 0)
      {
        **out << "all";
      }
      else
      {
        **out << preload_count;
      }
      **out << '\n';
    }

    **out << "lazy eval: " << ((!non_lazy_eval) ? "true" : "false") << '\n';
    **out << "collapse (post): " << ((collapse) ? "true" : "false") << '\n';

    if (point_limit)
    {
      **out << "Maximum point: " << point_limit << '\n';
    }

    if (start_time > 0)
    {
      **out << "Process from timestamp: " << start_time << '\n';
    }

    if (time_limit > 0)
    {
      **out << "Process to timestamp: " << time_limit << '\n';
    }

    // std::string mem_size_string;
    // util::makeMemoryDisplayString(mem_size_string, ohm::OccupancyMap::voxelMemoryPerRegion(region_voxel_dim));
    **out << "Map resolution: " << resolution << '\n';
    // **out << "Map region memory: " << mem_size_string << '\n';
    **out << "Hit probability: " << prob_hit << '\n';
    **out << "Miss probability: " << prob_miss << '\n';
    **out << std::flush;
    ++out;
  }
}

/// Map saving control flags.
enum SaveFlags : unsigned
{
  kSaveMap = (1 << 0),   ///< Save the occupancy map
  kSaveCloud = (1 << 1)  ///< Save a point cloud from the occupancy map
};

/// Save the Octomap to file and optional to point cloud. The map file is set as `base_name + ".bt"` and the point
/// cloud as `base_name + ".ply"`
void saveMap(const Options &opt, octomap::OcTree *map, const std::string &base_name, unsigned save_flags = kSaveMap)
{
  if (g_quit >= 2)
  {
    return;
  }

  if (save_flags & kSaveMap)
  {
    std::string output_file = base_name + ".bt";
    std::cout << "Saving map to " << output_file.c_str() << std::endl;

    bool ok = map->writeBinary(output_file);
    if (!ok)
    {
      std::cerr << "Failed to save map" << std::endl;
    }
  }

  if (save_flags & kSaveCloud)
  {
    // Save a cloud representation. Need to walk the tree leaves.
    std::cout << "Converting to point cloud." << std::endl;
    ohm::PlyMesh ply;
    glm::vec3 v;
    std::uint64_t point_count = 0;
    const auto map_end_iter = map->end_leafs();

    // float to byte colour channel conversion.
    const auto colour_channel_f = [](float cf) -> uint8_t  //
    {
      cf = 255.0f * std::max(cf, 0.0f);
      unsigned cu = unsigned(cf);
      return uint8_t(std::min(cu, 255u));
    };
    const bool use_colour = opt.cloud_colour.r > 0 || opt.cloud_colour.g > 0 || opt.cloud_colour.b > 0;
    const ohm::Colour c(colour_channel_f(opt.cloud_colour.r), colour_channel_f(opt.cloud_colour.g),
                        colour_channel_f(opt.cloud_colour.b));

    for (auto iter = map->begin_leafs(); iter != map_end_iter && g_quit < 2; ++iter)
    {
      const auto occupancy = iter->getLogOdds();
      if (occupancy >= map->getOccupancyThresLog())
      {
        const auto coord = iter.getCoordinate();
        const auto v = glm::vec3(coord.x(), coord.y(), coord.z());
        if (use_colour)
        {
          ply.addVertex(v, c);
        }
        else
        {
          ply.addVertex(v);
        }
        ++point_count;
      }
    }

    if (g_quit < 2)
    {
      std::string output_file = base_name + ".ply";
      std::cout << "Saving point cloud to " << output_file.c_str() << std::endl;
      ply.save(output_file.c_str(), true);
    }
  }
}
}  // namespace


/// Main look used to populate and serialise an Octomap.
int populateMap(const Options &opt)
{
  ohm::ScopedTimeDisplay time_display("Execution time");
  if (opt.quiet)
  {
    time_display.disable();
  }

  std::cout << "Loading points from " << opt.cloud_file << " with trajectory " << opt.trajectory_file << std::endl;

  SlamCloudLoader loader;
  if (!loader.open(opt.cloud_file.c_str(), opt.trajectory_file.c_str()))
  {
    fprintf(stderr, "Error loading cloud %s with trajectory %s \n", opt.cloud_file.c_str(),
            opt.trajectory_file.c_str());
    return -2;
  }

  std::atomic<uint64_t> elapsed_ms(0);
  ProgressMonitor prog(10);

  prog.setDisplayFunction([&elapsed_ms, &opt](const ProgressMonitor::Progress &prog) {
    if (!opt.quiet)
    {
      const uint64_t elapsed_ms_local = elapsed_ms;
      const uint64_t sec = elapsed_ms_local / 1000u;
      const unsigned ms = unsigned(elapsed_ms_local - sec * 1000);

      std::ostringstream out;
      out.imbue(std::locale(""));
      out << '\r';

      if (!prog.info.info.empty())
      {
        out << prog.info.info << " : ";
      }

      out << sec << '.' << std::setfill('0') << std::setw(3) << ms << "s : ";

      out << std::setfill(' ') << std::setw(12) << prog.progress;
      if (prog.info.total)
      {
        out << " / " << std::setfill(' ') << std::setw(12) << prog.info.total;
      }
      out << "    ";
      std::cout << out.str() << std::flush;
    }
  });

  octomap::OcTree map(opt.resolution);
  octomap::OcTreeKey key;
  glm::dvec3 origin, sample;
  // glm::vec3 voxel, ext(opt.resolution);
  double timestamp;
  uint64_t point_count = 0;
  // Update map visualisation every N samples.
  double timebase = -1;
  double first_timestamp = -1;
  double last_timestamp = -1;
  double first_batch_timestamp = -1;
  Clock::time_point start_time, end_time;
  Clock::time_point collapse_start_time, collapse_end_time;

  map.setProbHit(opt.prob_hit);
  map.setOccupancyThres(opt.prob_thresh);
  map.setProbMiss(opt.prob_miss);
  if (opt.prob_range[0] > 0)
  {
    map.setClampingThresMin(opt.prob_range[0]);
  }
  if (opt.prob_range[1] > 0)
  {
    map.setClampingThresMax(opt.prob_range[1]);
  }

  std::ostream *streams[] = { &std::cout, nullptr, nullptr };
  std::ofstream info_stream;
  if (opt.save_info)
  {
    streams[1] = &info_stream;
    std::string output_file = opt.output_base_name + ".txt";
    std::ofstream out(output_file.c_str());
    info_stream.open(output_file.c_str());
  }

  opt.print(streams);

  if (opt.preload_count)
  {
    int64_t preload_count = opt.preload_count;
    if (preload_count < 0 && opt.point_limit)
    {
      preload_count = opt.point_limit;
    }

    std::cout << "Preloading points";

    start_time = Clock::now();
    if (preload_count < 0)
    {
      std::cout << std::endl;
      loader.preload();
    }
    else
    {
      std::cout << " " << preload_count << std::endl;
      loader.preload(preload_count);
    }
    end_time = Clock::now();
    const double preload_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() * 1e-3;
    std::cout << "Preload completed over " << preload_time << " seconds." << std::endl;
  }

  start_time = Clock::now();
  std::cout << "Populating map" << std::endl;

  prog.beginProgress(ProgressMonitor::Info((point_count && timebase == 0) ?
                                             std::min<uint64_t>(point_count, loader.numberOfPoints()) :
                                             loader.numberOfPoints()));
  prog.startThread();

  //------------------------------------
  // Population loop.
  //------------------------------------
  // mapper.start();
  origin = glm::vec3(0, 0, 0);
  while ((point_count < opt.point_limit || opt.point_limit == 0) &&
         (last_timestamp - timebase < opt.time_limit || opt.time_limit == 0) &&
         loader.nextPoint(sample, &origin, &timestamp))
  {
    if (timebase < 0)
    {
      timebase = timestamp;
    }

    if (timestamp - timebase < opt.start_time)
    {
      continue;
    }

    if (last_timestamp < 0)
    {
      last_timestamp = timestamp;
    }

    if (first_timestamp < 0)
    {
      first_timestamp = timestamp;
    }

    ++point_count;
    if (first_batch_timestamp < 0)
    {
      first_batch_timestamp = timestamp;
    }

    map.insertRay(octomap::point3d{ float(origin.x), float(origin.y), float(origin.z) },
                  octomap::point3d{ float(sample.x), float(sample.y), float(sample.z) }, -1.0, !opt.non_lazy_eval);
    prog.incrementProgress();
    elapsed_ms = uint64_t((last_timestamp - timebase) * 1e3);

    if (opt.point_limit && point_count >= opt.point_limit ||
        opt.time_limit > 0 && last_timestamp - timebase >= opt.time_limit || g_quit)
    {
      break;
    }
  }

  prog.endProgress();
  prog.pause();

  if (!opt.quiet)
  {
    std::cout << std::endl;
  }

  // Collapse the map.
  if (opt.collapse)
  {
    if (!opt.non_lazy_eval)
    {
      if (!opt.quiet)
      {
        std::cout << "collapsing map" << std::endl;
      }
      collapse_start_time = Clock::now();
      map.updateInnerOccupancy();
      collapse_end_time = Clock::now();
    }
    else if (!opt.quiet)
    {
      std::cout << "skipping map collapse due to non-lazy insert (progressive collapse)" << std::endl;
    }
  }

  end_time = Clock::now();

  std::ostream **out = streams;
  while (*out)
  {
    const double time_range = last_timestamp - first_timestamp;
    const double processing_time_sec =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() * 1e-3;

    **out << "Point count: " << point_count << '\n';
    **out << "Data time: " << time_range << '\n';
    **out << "Total processing time: " << end_time - start_time << '\n';
    if (opt.collapse)
    {
      **out << "Collapse time: " << collapse_end_time - collapse_start_time << '\n';
    }
    **out << "Efficiency: " << ((processing_time_sec > 0 && time_range > 0) ? time_range / processing_time_sec : 0.0)
          << '\n';
    **out << "Points/sec: " << unsigned((processing_time_sec > 0) ? point_count / processing_time_sec : 0.0) << '\n';
    // **out << "Memory (approx): " << map.calculateApproximateMemory() / (1024.0 * 1024.0) << " MiB\n";
    **out << std::flush;
    ++out;
  }

  if (opt.serialise)
  {
    saveMap(opt, &map, opt.output_base_name, kSaveMap | kSaveCloud);
  }

  prog.joinThread();

  return 0;
}


int parseOptions(Options *opt, int argc, char *argv[])
{
  cxxopts::Options opt_parse(argv[0],
                             "Generate an occupancy map from a LAS/LAZ based point cloud and accompanying "
                             "trajectory file using GPU. The trajectory marks the scanner trajectory with timestamps "
                             "loosely corresponding to cloud point timestamps. Trajectory points are "
                             "interpolated for each cloud point based on corresponding times in the "
                             "trajectory.");
  opt_parse.positional_help("<cloud.laz> <_traj.txt> [output-base]");

  try
  {
    // clang-format off
    opt_parse.add_options()
      ("help", "Show help.")
      ("i,cloud", "The input cloud (las/laz) to load.", cxxopts::value(opt->cloud_file))
      ("o,output","Output base name", optVal(opt->output_base_name))
      ("p,point-limit", "Limit the number of points loaded.", optVal(opt->point_limit))
      ("preload", "Preload this number of points before starting processing. Zero for all. May be used for separating processing and loading time.", optVal(opt->preload_count)->default_value("0"))
      ("q,quiet", "Run in quiet mode. Suppresses progress messages.", optVal(opt->quiet))
      ("sensor", "Offset from the trajectory to the sensor position. Helps correct trajectory to the sensor centre for better rays.", optVal(opt->sensor_offset))
      ("s,start-time", "Only process points time stamped later than the specified time.", optVal(opt->start_time))
      ("serialise", "Serialise the results? This option is intended for skipping saving during performance analysis.", optVal(opt->serialise))
      ("save-info", "Save timing information to text based on the output file name.", optVal(opt->save_info))
      ("t,time-limit", "Limit the elapsed time in the LIDAR data to process (seconds). Measured relative to the first data sample.", optVal(opt->time_limit))
      ("trajectory", "The trajectory (text) file to load.", cxxopts::value(opt->trajectory_file))
      ("cloud-colour", "Colour for points in the saved cloud (if saving).", optVal(opt->cloud_colour))
      ;

    opt_parse.add_options("Map")
      ("clamp", "Set probability clamping to the given min/max.", optVal(opt->prob_range))
      ("h,hit", "The occupancy probability due to a hit. Must be >= 0.5.", optVal(opt->prob_hit))
      ("m,miss", "The occupancy probability due to a miss. Must be < 0.5.", optVal(opt->prob_miss))
      ("r,resolution", "The voxel resolution of the generated map.", optVal(opt->resolution))
      ("threshold", "Sets the occupancy threshold assigned when exporting the map to a cloud.", optVal(opt->prob_thresh)->implicit_value(optStr(opt->prob_thresh)))
      ("non-lazy", "Use non-lazy node insertion. Map may collapse at every step.", optVal(opt->non_lazy_eval))
      ("collapse", "Collapse octree nodes where possible after map generation. Redundant '--non-lazy' is used.", optVal(opt->collapse))
      ;

    // clang-format on

    opt_parse.parse_positional({ "cloud", "trajectory", "output" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help({ "", "Map" }) << std::endl;
      return 1;
    }

    if (opt->cloud_file.empty())
    {
      std::cerr << "Missing input cloud" << std::endl;
      return -1;
    }
    if (opt->trajectory_file.empty())
    {
      std::cerr << "Missing trajectory file" << std::endl;
      return -1;
    }
  }
  catch (const cxxopts::OptionException &e)
  {
    std::cerr << "Argument error\n" << e.what() << std::endl;
    return -1;
  }

  return 0;
}

/// entry point
int main(int argc, char *argv[])
{
  Options opt;

  std::cout.imbue(std::locale(""));

  int res = parseOptions(&opt, argc, argv);

  if (res)
  {
    return res;
  }

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  // Generate output name based on input if not specified.
  if (opt.output_base_name.empty())
  {
    const auto extension_start = opt.cloud_file.find_last_of('.');
    if (extension_start != std::string::npos)
    {
      opt.output_base_name = opt.cloud_file.substr(0, extension_start);
    }
    else
    {
      opt.output_base_name = opt.cloud_file;
    }
  }

  if (res)
  {
    return res;
  }

  res = populateMap(opt);

  return res;
}
