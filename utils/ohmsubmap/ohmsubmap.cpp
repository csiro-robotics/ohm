//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include <ohm/Aabb.h>
#include <ohm/MapLayer.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>

#include <logutil/LogUtil.h>

#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>

#include <ohmutil/Options.h>

namespace
{
int g_quit = 0;

void onSignal(int arg)
{
  if (arg == SIGINT || arg == SIGTERM)
  {
    ++g_quit;
  }
}

struct V3Arg
{
  double x = 0;
  double y = 0;
  double z = 0;
};

struct Options
{
  std::string map_in;
  std::string map_out;
  ohm::Aabb box = ohm::Aabb(0);
  glm::dvec3 centre = glm::dvec3(0);
  glm::dvec3 extents = glm::dvec3(0);
};
}  // namespace


int parseOptions(Options *opt, int argc, char *argv[])  // NOLINT(modernize-avoid-c-arrays)
{
  cxxopts::Options opt_parse(argv[0], "\nExtract a submap from an existing map.\n");
  opt_parse.positional_help("<map-in.ohm> <map-out.ohm>");

  try
  {
    opt_parse.add_options()("help", "Show help.")                                                                 //
      ("i,map", "The input map file (ohm) to load.", cxxopts::value(opt->map_in))                                 //
      ("o", "The input map file (ohm) to load.", cxxopts::value(opt->map_out))                                    //
      ("box", "Specify the submap extents as six floats; minx,miny,minz,maxx,maxy,maxz", optVal(opt->box))        //
      ("centre", "Centre of the extents specified extents as three floats; x,y,z", optVal(opt->centre))           //
      ("extents", "Specify the submap extents diagonal as three floats (positive); x,y,z", optVal(opt->extents))  //
      ;

    opt_parse.parse_positional({ "i", "o" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help({ "", "Group" }) << std::endl;
      return 1;
    }

    if (opt->map_in.empty())
    {
      std::cerr << "Missing input map file name" << std::endl;
      return -1;
    }

    if (opt->map_out.empty())
    {
      std::cerr << "Missing output map file name" << std::endl;
      return -1;
    }

    const double epsilon = 1e-6;
    if (glm::length(opt->box.diagonal()) < epsilon)
    {
      // No box. Try centre/extents setup.
      opt->box = ohm::Aabb::fromCentreHalfExtents(opt->centre, 0.5 * opt->extents);

      if (glm::length(opt->box.diagonal()) < epsilon)
      {
        // Still bad.
        std::cerr << "Extents not specified or too small" << std::endl;
        return -1;
      }
    }

    std::cout << "Extents: (" << opt->box << ")" << std::endl;

    if (opt->box.halfExtents().x <= 0 || opt->box.halfExtents().y <= 0 && opt->box.halfExtents().z <= 0)
    {
      std::cerr << "Zero or negative extents" << std::endl;
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
  res = parseOptions(&opt, argc, argv);

  if (res)
  {
    return res;
  }

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  ohm::OccupancyMap map(1.0f);

  std::cout << "Loading" << std::flush;
  res = ohm::load(opt.map_in.c_str(), map);
  std::cout << std::endl;

  if (res != 0)
  {
    std::cerr << "Failed to load map. Error(" << res << "): " << ohm::serialiseErrorCodeString(res) << std::endl;
    return res;
  }

  std::cout << "Cloning" << std::flush;
  // Clone the map
  const std::unique_ptr<ohm::OccupancyMap> map_copy(map.clone(opt.box.minExtents(), opt.box.maxExtents()));
  std::cout << std::endl;

  std::cout << "Saving" << std::flush;
  res = ohm::save(opt.map_out.c_str(), *map_copy);
  std::cout << std::endl;

  return res;
}
