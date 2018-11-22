//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include <ohm/Heightmap.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/OccupancyMap.h>
#include <ohm/MapSerialise.h>
#include <ohm/Voxel.h>
#include <ohmutil/OhmUtil.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/ProgressMonitor.h>
#include <ohmutil/SafeIO.h>
#include <ohmutil/ScopedTimeDisplay.h>

#include <ohmutil/Options.h>

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

namespace
{
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
    ohm::Heightmap::Axis axis_id = ohm::Heightmap::AxisZ;
    double base_height = 0;
    double clearance = 2.0;
    int blur = 0;
  };


  class LoadMapProgress : public ohm::SerialiseProgress
  {
  public:
    LoadMapProgress(ProgressMonitor &monitor) : monitor_(monitor) {}

    bool quit() const override { return ::quit > 1; }

    void setTargetProgress(unsigned target) override { monitor_.beginProgress(ProgressMonitor::Info(target)); }
    void incrementProgress(unsigned inc = 1) override { monitor_.incrementProgressBy(inc); }

  private:
    ProgressMonitor &monitor_;
  };
}


int parseOptions(Options &opt, int argc,  char *argv[])
{
  cxxopts::Options optParse(argv[0],
  "\nCreate a heightmap from an occupancy map.\n"
  );
  optParse.positional_help("<map.ohm> <heightmap.ohm>");

  try
  {
    optParse.add_options()
      ("help", "Show help.")
      ("i", "The input map file (ohm).", cxxopts::value(opt.map_file))
      ("o", "The output heightmap file (ohm).", cxxopts::value(opt.heightmap_file))
      ("base", "Base height: heightmap values are stored relative to this height.", optVal(opt.base_height))
      ("blur", "Blur factor in generating the heightmap. Must be >= 0, recommended < 5.", optVal(opt.blur))
      ("clearance", "The required height clearance for a heightmap surface voxel.", optVal(opt.clearance))
      ;

    optParse.parse_positional({ "i", "o" });

    cxxopts::ParseResult parsed = optParse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << optParse.help({ "", "Group" }) << std::endl;
      return 1;
    }

    if (opt.map_file.empty())
    {
      std::cerr << "Missing input map" << std::endl;
      return -1;
    }

    if (opt.heightmap_file.empty())
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
  Options opt;

  std::cout.imbue(std::locale(""));

  int res = 0;
  res = parseOptions(opt, argc, argv);

  if (res)
  {
    return res;
  }

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  printf("Loading map %s\n", opt.map_file.c_str());
  ProgressMonitor prog(10);
  LoadMapProgress load_progress(prog);
  ohm::OccupancyMap map(1.0);
  ohm::MapVersion version;

  prog.setDisplayFunction([&opt](const ProgressMonitor::Progress &prog)
  {
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

  std::cout << std::endl;;

  if (res != 0)
  {
    fprintf(stderr, "Failed to load map. Error code: %d\n", res);
    return res;
  }

  std::cout << "Generating heightmap" << std::endl;

  ohm::Heightmap heightmap(map.resolution(), opt.clearance, opt.axis_id);
  heightmap.setOccupancyMap(&map);
  if (opt.blur >= 0)
  {
    heightmap.setBlurLevel(opt.blur);
  }

  heightmap.update(opt.base_height);

  std::cout << "Saving " << opt.heightmap_file << std::endl;
  ohm::save(opt.heightmap_file.c_str(), heightmap.heightmap(), nullptr);
  return res;
}
