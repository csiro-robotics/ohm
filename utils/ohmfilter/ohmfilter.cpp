//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include <ohm/CovarianceVoxel.h>
#include <ohm/Key.h>
#include <ohm/MapLayout.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelData.h>

#include <ohmutil/PlyPointStream.h>
#include <ohmutil/ProgressMonitor.h>

#include <slamio/SlamCloudLoader.h>

#include <glm/gtc/matrix_access.hpp>

#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

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

using FilterFunction = std::function<bool(double, const glm::dvec3 &, const ohm::Key &)>;

struct Options
{
  std::string map_file;
  std::string cloud_in;
  std::string traj_in;
  std::string cloud_out;
  double expected_value_tolerance = -1;
  bool occupancy_only = false;
  bool quiet = false;
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

bool filterPointByCovariance(const glm::dvec3 &point, const glm::dvec3 &mean, const glm::dmat3 &cov_sqrt,
                             double threshold)
{
  // Transient point assessement vs NDT:
  // With:
  //  P:covariance
  //  S:covariance square root
  //  a:test value
  // a = (point - mean)^T P^{-1} (point - mean)
  // where P=covariance
  // expected value is dimensionality (3)
  // so to pass, we need a < 3.
  //
  // Using CovarianceVoxel we have the square-root of P:
  // P = S S^T
  // so:
  // v = S^{-1} (point - mean)
  // a = v^T v

  const glm::dvec3 divergence_from_mean = point - mean;
  const auto v = glm::inverse(cov_sqrt) * divergence_from_mean;
  const double value = glm::dot(v, v);
  const double expected_value = 3.0;  // In R3 => expected value is 3.
  return std::abs(value) < expected_value + threshold;
}
}  // namespace

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

int parseOptions(Options *opt, int argc, char *argv[])  // NOLINT(modernize-avoid-c-arrays)
{
  cxxopts::Options opt_parse(argv[0], "Convert an occupancy map to a point cloud. Defaults to generate a positional "
                                      "point cloud, but can generate a clearance cloud as well.");
  opt_parse.positional_help("<map.ohm> <cloud.ply>");

  try
  {
    // Build GPU options set.
    // clang-format off
    opt_parse.add_options()
      ("help", "Show help.")
      ("map", "The input map file (ohm).", cxxopts::value(opt->map_file))
      ("cloud-in", "The input cloud file.", cxxopts::value(opt->cloud_in))
      ("traj", "Optional trajectory file input.", cxxopts::value(opt->traj_in))
      ("cloud-out", "The output cloud file (ply).", cxxopts::value(opt->cloud_out))
      ("occupancy-only", "Force using only occupancy information, even if NDT is present.", optVal(opt->occupancy_only))
      ("tolerance", "Additional tolerance above the expected value. A zero value enabled the direct comparison of each "
                    "point against it's voxel's covariance while a positive value expands the tolerance beyond the "
                    "expected value. A negative value disables this check.",
        cxxopts::value(opt->expected_value_tolerance))
      ("quiet", "Limited log output.", optVal(opt->quiet))
      ;
    // clang-format on

    opt_parse.parse_positional({ "map", "cloud-in", "cloud-out" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help({ "" }) << std::endl;
      return 1;
    }

    if (opt->map_file.empty())
    {
      std::cerr << "Missing input map file name" << std::endl;
      return -1;
    }
    if (opt->cloud_in.empty())
    {
      std::cerr << "Missing input cloud file name" << std::endl;
      return -1;
    }
    if (opt->cloud_out.empty())
    {
      std::cerr << "Missing output file name" << std::endl;
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

bool filterCloud(const Options &opt, const ohm::OccupancyMap &map, ProgressMonitor *prog)
{
  // Use the SlamCloudLoader with no trajectory specified to load the cloud - it's just convenient.
  slamio::SlamCloudLoader cloud_loader;
  cloud_loader.setErrorLog([](const char *msg) { std::cerr << msg << std::flush; });

  if (!cloud_loader.openWithTrajectory(opt.cloud_in.c_str(), opt.traj_in.c_str()))
  {
    std::cerr << "Error: Unable to load cloud file " << opt.cloud_in << std::endl;
    return false;
  }

  prog->beginProgress(ProgressMonitor::Info("filtering", cloud_loader.numberOfPoints()));
  prog->unpause();

  // Prepare required data layers.
  ohm::Voxel<const ohm::CovarianceVoxel> voxel_cov(&map, map.layout().covarianceLayer());
  ohm::Voxel<const ohm::VoxelMean> voxel_mean(&map, map.layout().meanLayer());
  ohm::Voxel<const float> voxel_occ(&map, map.layout().occupancyLayer());

  if (!voxel_occ.isLayerValid())
  {
    std::cerr << "Error: Map missing occupancy layer" << std::endl;
    return false;
  }

  // Select the filter function depending on available layers.
  FilterFunction filter;
  glm::dmat3 cov_sqrt{};
  if (!opt.occupancy_only && voxel_cov.isLayerValid() && voxel_mean.isLayerValid())
  {
    if (!opt.quiet)
    {
      std::cout << "Filtering with NDT information" << std::endl;
    }
    filter = [&](double /* timestamp */, const glm::dvec3 &point, const ohm::Key &key) -> bool {
      ohm::setVoxelKey(key, voxel_occ, voxel_mean, voxel_cov);
      if (ohm::isOccupied(voxel_occ))
      {
        const glm::dvec3 mean = ohm::positionUnsafe(voxel_mean);
        ohm::CovarianceVoxel cov_data;
        voxel_cov.read(&cov_data);
        cov_sqrt = ohm::covarianceSqrtMatrix(&cov_data);
        if (opt.expected_value_tolerance >= 0)
        {
          return filterPointByCovariance(point, mean, cov_sqrt, opt.expected_value_tolerance);
        }
        return true;
      }

      return false;
    };
  }
  else
  {
    if (!opt.quiet)
    {
      std::cout << "Filtering using occupancy only" << std::endl;
    }
    filter = [&](double /* timestamp */, const glm::dvec3 & /* point */, const ohm::Key &key) -> bool {
      voxel_occ.setKey(key);
      return ohm::isOccupied(voxel_occ);
    };
  }

  std::ofstream out(opt.cloud_out.c_str(), std::ios::binary);
  using Property = ohm::PlyPointStream::Property;
  using Type = ohm::PlyPointStream::Type;
  ohm::PlyPointStream ply({ Property{ "x", Type::kFloat64 }, Property{ "y", Type::kFloat64 },
                            Property{ "z", Type::kFloat64 }, Property{ "time", Type::kFloat64 } },
                          out);

  if (!ply.isOpen())
  {
    std::cerr << "Error: Unable to write to " << opt.cloud_out << std::endl;
    return false;
  }
  if (!opt.quiet)
  {
    std::cout << "Exporting to " << opt.cloud_out << std::endl;
  }
  ohm::Key key;
  std::uint64_t point_count = 0;
  std::uint64_t export_count = 0;
  const bool with_trajectory = !opt.traj_in.empty();
  slamio::SamplePoint sample{};
  while (cloud_loader.nextSample(sample))
  {
    key = map.voxelKey(sample.sample);
    if (filter(sample.timestamp, sample.sample, key))
    {
      ply.setPointPosition(sample.sample);
      if (with_trajectory)
      {
        ply.setPointNormal(sample.sample - sample.origin);
      }
      ply.setPointTimestamp(sample.timestamp);
      ply.writePoint();
      ++export_count;
    }
    prog->incrementProgress();
    ++point_count;
  }
  prog->endProgress();
  prog->pause();
  if (!opt.quiet)
  {
    std::cout << std::endl;
  }

  cloud_loader.close();
  ply.close();
  out.close();

  if (!opt.quiet)
  {
    std::cout << "Removed: " << point_count - export_count << std::endl;
    std::cout << "Exported: " << export_count << "/" << point_count << std::endl;
  }

  return true;
}


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

  if (!opt.quiet)
  {
    std::cout << "Loading map " << opt.map_file.c_str() << std::endl;
  }
  ProgressMonitor prog(10);
  LoadMapProgress load_progress(prog);
  if (!opt.quiet)
  {
    prog.startThread(true);
  }

  prog.setDisplayFunction([&opt](const ProgressMonitor::Progress &prog) {
    if (!opt.quiet)
    {
      std::ostringstream out;
      out.imbue(std::locale(""));
      out << '\r';

      if (!prog.info.info.empty())
      {
        out << prog.info.info << " : ";
      }

      const auto fill_width = std::numeric_limits<decltype(prog.progress)>::digits10;
      out << std::setfill(' ') << std::setw(fill_width) << prog.progress;
      if (prog.info.total)
      {
        out << " / " << std::setfill(' ') << std::setw(fill_width) << prog.info.total;
      }
      out << "    ";
      std::cout << out.str() << std::flush;
    }
  });

  ohm::OccupancyMap map;
  res = ohm::load(opt.map_file.c_str(), map, &load_progress);

  prog.endProgress();
  prog.pause();
  if (!opt.quiet)
  {
    std::cout << std::endl;
  }

  if (res == 0)
  {
    if (!filterCloud(opt, map, &prog))
    {
      res = -1;
    }
  }
  else
  {
    std::cerr << "Error: Unable to read map '" << opt.map_file << "' : " << ohm::serialiseErrorCodeString(res)
              << std::endl;
  }

  prog.joinThread();

  return res;
}
