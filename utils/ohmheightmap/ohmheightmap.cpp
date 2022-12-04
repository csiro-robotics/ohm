//
// author Kazys Stepanas
//
// Utility for generating an ohm heightmap from an ohm occupancy map.
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Trace.h>
#include <ohm/Voxel.h>

#include <ohmheightmap/Heightmap.h>
#include <ohmheightmap/HeightmapVoxel.h>

#include <logutil/LogUtil.h>
#include <ohmutil/GlmStream.h>
#include <ohmutil/ProgressMonitor.h>
#include <ohmutil/SafeIO.h>
#include <ohmutil/ScopedTimeDisplay.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <unordered_set>

#include <3esservermacros.h>

/// Declare a function for throwing exceptions for bad arguments. The exception isn't included yet due to cxxoptions
/// oddities.
void badArgParse(const std::string &info);

namespace ohm
{
// Custom option parsing. Must come before we include Options.h/cxxopt.hpp
std::istream &operator>>(std::istream &in, HeightmapMode &mode)
{
  std::string mode_str;
  in >> mode_str;
  bool valid_mode = false;
  mode = heightmapModeFromString(mode_str, &valid_mode);
  if (!valid_mode)
  {
    badArgParse(mode_str);
  }
  return in;
}

std::ostream &operator<<(std::ostream &out, const HeightmapMode mode)
{
  std::string mode_str = heightmapModeToString(mode);
  out << mode_str;
  return out;
}


std::istream &operator>>(std::istream &in, UpAxis &up)
{
  std::string up_str;
  in >> up_str;
  if (up_str == "x")
  {
    up = UpAxis::kX;
  }
  else if (up_str == "y")
  {
    up = UpAxis::kY;
  }
  else if (up_str == "z")
  {
    up = UpAxis::kZ;
  }
  else if (up_str == "-x")
  {
    up = UpAxis::kNegX;
  }
  else if (up_str == "-y")
  {
    up = UpAxis::kNegY;
  }
  else if (up_str == "-z")
  {
    up = UpAxis::kNegZ;
  }
  else
  {
    badArgParse(up_str);
  }
  return in;
}

std::ostream &operator<<(std::ostream &out, const UpAxis up)
{
  switch (up)
  {
  case UpAxis::kX:
    out << "x";
    break;
  case UpAxis::kY:
    out << "y";
    break;
  case UpAxis::kZ:
    out << "z";
    break;
  case UpAxis::kNegX:
    out << "-x";
    break;
  case UpAxis::kNegY:
    out << "-y";
    break;
  case UpAxis::kNegZ:
    out << "-z";
    break;
  default:
    out << "<unknown>";
    break;
  }
  return out;
}
}  // namespace ohm

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

void badArgParse(const std::string &info)
{
  throw cxxopts::invalid_option_format_error(info);
}


namespace
{
using Clock = std::chrono::high_resolution_clock;

int g_quit = 0;

void onSignal(int arg)
{
  if (arg == SIGINT || arg == SIGTERM)
  {
    ++g_quit;
  }
}

struct Options
{
  std::string map_file;
  std::string heightmap_file;
  std::string trace;  // 3es trace file (if available)
  ohm::UpAxis axis_id = ohm::UpAxis::kZ;
  ohm::HeightmapMode mode = ohm::HeightmapMode::kSimpleFill;
  glm::dvec3 seed_pos{ 0, 0, 0 };
  double clearance = 2.0;
  double floor = -1;
  double ceiling = -1;
  unsigned virtual_surface_filter_threshold = 0;
  bool virtual_surfaces = false;
  bool no_voxel_mean = false;
};


class LoadMapProgress : public ohm::SerialiseProgress
{
public:
  explicit LoadMapProgress(ProgressMonitor &monitor)
    : monitor_(monitor)
  {}

  bool quit() const override { return ::g_quit > 1; }

  void setTargetProgress(unsigned target) override { monitor_.beginProgress(ProgressMonitor::Info(target)); }
  void incrementProgress(unsigned inc) override { monitor_.incrementProgressBy(inc); }

private:
  ProgressMonitor &monitor_;
};
}  // namespace


int parseOptions(Options *opt, int argc, char *argv[])  // NOLINT(modernize-avoid-c-arrays)
{
  cxxopts::Options opt_parse(argv[0], "\nCreate a heightmap from an occupancy map.\n");
  opt_parse.positional_help("<map.ohm> <heightmap.ohm>");

  try
  {
    std::ostringstream mode_help;
    mode_help << "Heightmap expansion mode (";
    std::string append_str = "";
    for (ohm::HeightmapMode mode = ohm::HeightmapMode::kFirst; mode <= ohm::HeightmapMode::kLast;
         mode = ohm::HeightmapMode(int(mode) + 1))
    {
      mode_help << append_str << ohm::heightmapModeToString(mode);
      append_str = ", ";
    }
    mode_help << ").";

    opt_parse.add_options()("help", "Show help.")("i", "The input map file (ohm).", cxxopts::value(opt->map_file))  //
      ("o", "The output heightmap file (ohm).", cxxopts::value(opt->heightmap_file))                                //
      ("ceiling",
       "Ceiling applied to ground voxel searches. Limits how far up to search. Non-negative to enable. Affected by "
       "'--mode'.",
       optVal(opt->ceiling))                                                                                 //
      ("clearance", "The required height clearance for a heightmap surface voxel.", optVal(opt->clearance))  //
      ("floor",
       "Floor applied to ground voxel searches. Limits how far down to search. Non-negative to enable. Affected by "
       "'--mode'.",
       optVal(opt->floor))                                                                           //
      ("mode", mode_help.str(), optVal(opt->mode))                                                   //
      ("no-voxel-mean", "Ignore voxel mean positioning if available?.", optVal(opt->no_voxel_mean))  //
      ("seed", "Seed position from which to build the heightmap. Specified as a 3 component vector such as '0,0,1'.",
       optVal(opt->seed_pos))                                                        //
      ("up", "Specifies the up axis {x,y,z,-x,-y,-z}.", optVal(opt->axis_id))        //
      ("virtual", "Allow virtual surfaces?", cxxopts::value(opt->virtual_surfaces))  //
      ("virtual-filter-threshold",
       "Remove virtual surface voxels with fewer than this number of occupied neighbours. Layered modes only.",
       optVal(opt->virtual_surface_filter_threshold))  //
      ;

    if (ohm::trace::available())
    {
      opt_parse.add_options()(
        "trace", "Enable debug tracing to the given file name to generate a 3es file. High performance impact.",
        cxxopts::value(opt->trace));
    }

    opt_parse.parse_positional({ "i", "o" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help({ "", "Group" }) << std::endl;
      return 1;
    }

    if (opt->map_file.empty())
    {
      std::cerr << "Missing input map" << std::endl;
      return -1;
    }

    if (opt->heightmap_file.empty())
    {
      std::cerr << "Missing output name" << std::endl;
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

int main(int argc, char *argv[])
{
  const auto start_time = Clock::now();
  Options opt;

  std::cout.imbue(std::locale(""));

  int res = 0;
  res = parseOptions(&opt, argc, argv);

  if (res)
  {
    return res;
  }

  // Initialise 3ES
  std::unique_ptr<ohm::Trace> trace;
  if (!opt.trace.empty())
  {
    trace = std::make_unique<ohm::Trace>(opt.trace.c_str());
  }

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  printf("Loading map %s\n", opt.map_file.c_str());
  ProgressMonitor prog(10);
  LoadMapProgress load_progress(prog);
  ohm::OccupancyMap map(1.0);
  ohm::MapVersion version;

  prog.setDisplayFunction([](const ProgressMonitor::Progress &prog, bool final) {
    std::ostringstream str;
    str << '\r';
    str << prog.progress;
    if (prog.info.total)
    {
      str << " / " << prog.info.total;
    }
    str << "      ";
    if (final)
    {
      str << '\n';
    }
    std::cout << str.str() << std::flush;
  });

  prog.startThread();
  res = ohm::load(opt.map_file.c_str(), map, &load_progress, &version);
  prog.endProgress();

  std::cout << std::endl;

  if (res != 0)
  {
    std::cerr << "Failed to load map. Error(" << res << "): " << ohm::serialiseErrorCodeString(res) << std::endl;
    return res;
  }

  std::cout << "Loaded in " << (Clock::now() - start_time) << std::endl;


  std::cout << "Generating heightmap" << std::endl;

  const auto heightmap_start_time = Clock::now();

  ohm::Heightmap heightmap(map.resolution(), opt.clearance, opt.axis_id);
  heightmap.setMode(opt.mode);
  heightmap.setOccupancyMap(&map);
  heightmap.heightmap().setOrigin(map.origin());

  heightmap.setCeiling(opt.ceiling >= 0 ? opt.ceiling : heightmap.ceiling());
  heightmap.setFloor(opt.floor >= 0 ? opt.floor : heightmap.floor());
  heightmap.setIgnoreVoxelMean(opt.no_voxel_mean);
  heightmap.setGenerateVirtualSurface(opt.virtual_surfaces);
  heightmap.setVirtualSurfaceFilterThreshold(opt.virtual_surface_filter_threshold);

  heightmap.buildHeightmap(opt.seed_pos);
  heightmap.checkForBaseLayerDuplicates(std::cerr);

  const auto heightmap_end_time = Clock::now();

  std::cout << "Heightmap generated in " << (heightmap_end_time - heightmap_start_time) << std::endl;

  std::cout << "Saving " << opt.heightmap_file << std::endl;
  ohm::save(opt.heightmap_file.c_str(), heightmap.heightmap(), nullptr);

  return res;
}
