//
// author Kazys Stepanas
//
#include <ohm/Heightmap.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Trace.h>
#include <ohm/Voxel.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/ProgressMonitor.h>
#include <ohmutil/SafeIO.h>
#include <ohmutil/ScopedTimeDisplay.h>

#include <ohmutil/Options.h>

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

namespace
{
  using Clock = std::chrono::high_resolution_clock;

  int quit = 0;

  void onSignal(int arg)
  {
    if (arg == SIGINT || arg == SIGTERM)
    {
      ++quit;
    }
  }

  struct Options
  {
    std::string map_file;
    std::string heightmap_file;
    ohm::UpAxis axis_id = ohm::UpAxis::kZ;
    double base_height = 0;
    double clearance = 2.0;
    double floor = 0;
    double ceiling = 0;
    bool no_voxel_mean = false;
  };


  class LoadMapProgress : public ohm::SerialiseProgress
  {
  public:
    LoadMapProgress(ProgressMonitor &monitor)  // NOLINT(google-runtime-references)
      : monitor_(monitor)
    {}

    bool quit() const override { return ::quit > 1; }

    void setTargetProgress(unsigned target) override { monitor_.beginProgress(ProgressMonitor::Info(target)); }
    void incrementProgress(unsigned inc) override { monitor_.incrementProgressBy(inc); }

  private:
    ProgressMonitor &monitor_;
  };
}  // namespace


int parseOptions(Options *opt, int argc, char *argv[])
{
  cxxopts::Options opt_parse(argv[0], "\nCreate a heightmap from an occupancy map.\n");
  opt_parse.positional_help("<map.ohm> <heightmap.ohm>");

  try
  {
    opt_parse.add_options()("help", "Show help.")("i", "The input map file (ohm).", cxxopts::value(opt->map_file))  //
      ("o", "The output heightmap file (ohm).", cxxopts::value(opt->heightmap_file))                                //
      ("base", "Base height: heightmap values are stored relative to this height.", optVal(opt->base_height))       //
      ("clearance", "The required height clearance for a heightmap surface voxel.", optVal(opt->clearance))         //
      ("floor", "Heightmap excludes voxels below this (positive) value below the --base height. Positive to enable.",
       optVal(opt->floor))  //
      ("ceiling", "Heightmap excludes voxels above this (positive) value above the --base height. Positive to enable.",
       optVal(opt->ceiling))                                                                         //
      ("no-voxel-mean", "Ignore voxel mean positioning if available?.", optVal(opt->no_voxel_mean))  //
      ;

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

  // Initialise TES
  ohm::Trace trace("ohmheightmap.3es");

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  printf("Loading map %s\n", opt.map_file.c_str());
  ProgressMonitor prog(10);
  LoadMapProgress load_progress(prog);
  ohm::OccupancyMap map(1.0);
  ohm::MapVersion version;

  prog.setDisplayFunction([](const ProgressMonitor::Progress &prog) {
    std::ostringstream str;
    str << '\r';
    str << prog.progress;
    if (prog.info.total)
    {
      str << " / " << prog.info.total;
    }
    str << "      ";
    std::cout << str.str() << std::flush;
  });

  prog.startThread();
  res = ohm::load(opt.map_file.c_str(), map, &load_progress, &version);
  prog.endProgress();

  std::cout << std::endl;

  if (res != 0)
  {
    std::cerr << "Failed to load map. Error(" << res << "): " << ohm::errorCodeString(res) << std::endl;
    return res;
  }

  std::cout << "Loaded in " << (Clock::now() - start_time) << std::endl;


  std::cout << "Generating heightmap" << std::endl;

  const auto heightmap_start_time = Clock::now();

  ohm::Heightmap heightmap(map.resolution(), opt.clearance, opt.axis_id);
  heightmap.setUseFloodFill(true);  // For better surface following.
  heightmap.setOccupancyMap(&map);

  heightmap.setIgnoreVoxelMean(opt.no_voxel_mean);

  heightmap.buildHeightmap(opt.base_height * heightmap.upAxisNormal());

  const auto heightmap_end_time = Clock::now();

  std::cout << "Heightmap generated in " << (heightmap_end_time - heightmap_start_time) << std::endl;

  std::cout << "Saving " << opt.heightmap_file << std::endl;
  ohm::save(opt.heightmap_file.c_str(), heightmap.heightmap(), nullptr);

  return res;
}
