//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include <ohm/MapCache.h>
#include <ohm/Key.h>
#include <ohm/OccupancyMap.h>
#include <ohm/MapSerialise.h>
#include <ohm/Voxel.h>
#include <ohm/OccupancyType.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/ProgressMonitor.h>
#include <ohmutil/Options.h>

#include <liblas/liblas.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iostream>
#include <locale>
#include <sstream>
#include <unordered_set>

#define PROFILING 1
#include <ohmutil/Profile.h>

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
    std::string map_file_in;
    std::string cloud_file_in;
    std::string cloud_file_out;
    float occupancy_threshold = 0;
  };


  class MapProgress : public ohm::SerialiseProgress
  {
  public:
    MapProgress(ProgressMonitor &monitor) : monitor_(monitor) {}

    bool quit() const override { return ::quit > 1; }

    void setTargetProgress(unsigned target) override { monitor_.beginProgress(ProgressMonitor::Info("Loading", target)); }
    void incrementProgress(unsigned inc = 1) override { monitor_.incrementProgressBy(inc); }

  private:
    ProgressMonitor &monitor_;
  };
}


int parseOptions(Options &opt, int argc, char *argv[])
{
  cxxopts::Options opt_parse(argv[0],
    "\nFilters a cloud against an occupancy map, removing points in free voxels."
  );
  opt_parse.positional_help("<map-in.ohm> <cloud-in.laz> <cloud-out.laz>");

  try
  {
    // clang-format off
    opt_parse.add_options()
        ("cloud-in", "Input cloud to filter", cxxopts::value(opt.cloud_file_in))
        ("cloud-out", "Clout to write filtered results to", cxxopts::value(opt.cloud_file_out))
        ("map", "Input occupancy map", cxxopts::value(opt.map_file_in))
        ("threshold", "Changes the occupancy probability threshold from that used to generate the map.", optVal(opt.occupancy_threshold), "[0, 1]")
      ;
    // clang-format on

    opt_parse.parse_positional({ "map", "cloud-in", "cloud-out" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help({ "", "Group" }) << std::endl;
      return 1;
    }

    if (opt.map_file_in.empty())
    {
      std::cerr << "Missing input map" << std::endl;
      return -1;
    }
    if (opt.cloud_file_in.empty())
    {
      std::cerr << "Missing input cloud" << std::endl;
      return -1;
    }
    if (opt.cloud_file_out.empty())
    {
      std::cerr << "Missing output cloud" << std::endl;
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


size_t filterCloud(ProgressMonitor &prog, const ohm::OccupancyMap &map, const Options &opt)
{
  size_t exported = 0;
  size_t number_of_points = 0;

  std::ifstream ifs;
  std::ofstream ofs;

  ifs.open(opt.cloud_file_in.c_str(), std::ios::in | std::ios::binary);
  ofs.open(opt.cloud_file_out.c_str(), std::ios::out | std::ios::binary);

  bool ok = true;

  if (!ifs.is_open())
  {
    ok = false;
    std::cerr << "Unable to read " << opt.cloud_file_in << std::endl;
  }

  if (!ofs.is_open())
  {
    ok = false;
    std::cerr << "Unable to write " << opt.cloud_file_out << std::endl;
  }

  if (!ok)
  {
    return false;
  }

  std::string extension_str = opt.cloud_file_out;
  bool compress = false;

  if (extension_str.size() >= 4)
  {
    extension_str = extension_str.substr(extension_str.size() - 4);
    std::transform(extension_str.begin(), extension_str.end(), extension_str.begin(), &tolower);
    if (extension_str.compare(".laz") == 0)
    {
      compress = true;
    }
  }

  liblas::ReaderFactory reader_factor;
  try
  {
    liblas::Reader reader(reader_factor.CreateWithStream(ifs));
    liblas::Header header = reader.GetHeader();
    header.SetCompressed(compress);
    liblas::Writer writer(ofs, header);
    number_of_points = reader.GetHeader().GetPointRecordsCount();

    prog.beginProgress(ProgressMonitor::Info("Filtering", reader.GetHeader().GetPointRecordsCount()));

    ohm::VoxelConst voxel;
    ohm::MapCache cache;
    glm::dvec3 pos;
    while (reader.ReadNextPoint())
    {
      // Get the voxel for the point.
      auto &&pt = reader.GetPoint();
      pos = glm::dvec3(pt.GetX(), pt.GetY(), pt.GetZ());
      voxel = map.voxel(map.voxelKey(pos), &cache);
      if (voxel.isOccupied())
      {
        writer.WritePoint(reader.GetPoint());
        ++exported;
      }
      prog.incrementProgress();
    }
  }
  catch (std::exception const& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  prog.endProgress();
  std::cout << std::endl;

  std::cout << "Exported " << exported << " / " <<  number_of_points << " point(s)" << std::endl;

  return exported;
}


int main(int argc, char *argv[])
{
  Options opt;

  std::cout.imbue(std::locale(""));

  int res = parseOptions(opt, argc, argv);

  if (res)
  {
    return res;
  }

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  printf("Loading map %s\n", opt.map_file_in.c_str());
  ProgressMonitor prog(10);
  MapProgress load_progress(prog);
  ohm::OccupancyMap map(1.0f);

  prog.setDisplayFunction([](const ProgressMonitor::Progress &prog)
  {
    std::ostringstream out;
    out.imbue(std::locale(""));
    out << '\r';

    if (prog.info.info)
    {
      out << prog.info.info << " : ";
    }

    out << std::setfill(' ') << std::setw(12) << prog.progress;
    if (prog.info.total)
    {
      out << " / " << std::setfill(' ') << std::setw(12) << prog.info.total;
    }
    out << "    ";
    std::cout << out.str() << std::flush;
  });

  prog.startThread();
  res = ohm::load(opt.map_file_in.c_str(), map, &load_progress);
  prog.endProgress();

  std::cout << std::endl;

  if (res != 0)
  {
    fprintf(stderr, "Failed to load map. Error code: %d\n", res);
    return res;
  }

  printf("\n");

  if (opt.occupancy_threshold > 0)
  {
    map.setOccupancyThresholdProbability(opt.occupancy_threshold);
  }

  // Start filtering the cloud.
  filterCloud(prog, map, opt);

  prog.joinThread();

  return res;
}
