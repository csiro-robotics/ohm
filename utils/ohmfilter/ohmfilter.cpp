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

#include <ohmutil/PlyMesh.h>
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
  int quit = 0;

  void onSignal(int arg)
  {
    if (arg == SIGINT || arg == SIGTERM)
    {
      ++quit;
    }
  }

  using FilterFunction = std::function<bool(double, const glm::dvec3 &, const ohm::Key &)>;

  struct Options
  {
    std::string map_file;
    std::string cloud_in;
    std::string cloud_out;
    double expected_value_threshold = 0.2;
    bool occupancy_only = false;
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

  bool filterPointByCovariance(const glm::dvec3 &point, const glm::dvec3 &mean, const glm::dmat3 &cov_sqrt,
                               double threshold)
  {
    // transient point assessement vs NDT:
    // (point - mean)^T cov^{-1} (point - mean)
    // result is scalar
    // expected value is ~= dimensionality (3)
    //
    // Using CovarianceVoxel instead:
    // we have S S^T = P
    // v = S^{-1} (point - mean)
    // answer = v^T v
    // S: Square root of the covariance
    // P: covariance matrix
    // C===S (alternative naming)

    const glm::dvec3 divergence_from_mean = point - mean;
    const auto v = glm::inverse(cov_sqrt) * divergence_from_mean;
    const double value = glm::dot(v, v);
    const double expected_value = 3.0;  // In R3 => expected value is 3.
    return std::abs(value) < expected_value + threshold;
  }
}  // namespace

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

int parseOptions(Options *opt, int argc, char *argv[])
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
      ("cloud-out", "The output cloud file (ply).", cxxopts::value(opt->cloud_out))
      ("occupancy-only", "Force using only occupancy information, even if NDT is presnet.", optVal(opt->occupancy_only))
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
  SlamCloudLoader cloud_loader;

  if (!cloud_loader.open(opt.cloud_in.c_str(), ""))
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
    std::cout << "Filtering with NDT information" << std::endl;
    filter = [&](double timestamp, const glm::dvec3 &point, const ohm::Key &key) -> bool {
      ohm::setVoxelKey(key, voxel_occ, voxel_mean, voxel_cov);
      if (ohm::isOccupied(voxel_occ))
      {
        const glm::dvec3 mean = ohm::positionUnsafe(voxel_mean);
        const ohm::CovarianceVoxel *cov_data = voxel_cov.dataPtr();
        cov_sqrt = ohm::covarianceSqrtMatrix(cov_data);
        return filterPointByCovariance(point, mean, cov_sqrt, opt.expected_value_threshold);
      }

      return false;
    };
  }
  else
  {
    std::cout << "Filtering using occupancy only" << std::endl;
    filter = [&](double timestamp, const glm::dvec3 &point, const ohm::Key &key) -> bool {
      voxel_occ.setKey(key);
      return ohm::isOccupied(voxel_occ);
    };
  }

  ohm::PlyMesh ply;
  glm::dvec3 point(0);
  double timestamp = 0;
  ohm::Key key;
  std::uint64_t point_count = 0, export_count = 0;
  while (cloud_loader.nextPoint(point, nullptr, &timestamp))
  {
    key = map.voxelKey(point);
    if (filter(timestamp, point, key))
    {
      ply.addVertex(point);
      ++export_count;
    }
    prog->incrementProgress();
    ++point_count;
  }
  prog->endProgress();
  prog->pause();
  std::cout << std::endl;

  cloud_loader.close();

  std::cout << "Removed: " << point_count - export_count << std::endl;
  std::cout << "Exporting to " << opt.cloud_out << std::endl;
  ply.save(opt.cloud_out.c_str(), true);
  std::cout << "Exported: " << export_count << "/" << point_count << std::endl;

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

  std::cout << "Loading map " << opt.map_file.c_str() << std::endl;
  ProgressMonitor prog(10);
  LoadMapProgress load_progress(prog);

  prog.setDisplayFunction([](const ProgressMonitor::Progress &prog) {
    // if (!opt.quiet)
    {
      std::ostringstream out;
      out.imbue(std::locale(""));
      out << '\r';

      if (prog.info.info && prog.info.info[0])
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
    }
  });

  ohm::OccupancyMap map;
  res = ohm::load(opt.map_file.c_str(), map, &load_progress);

  prog.endProgress();
  prog.pause();
  std::cout << std::endl;

  if (res == 0)
  {
    if (!filterCloud(opt, map, &prog))
    {
      res = -1;
    }
  }
  else
  {
    std::cerr << "Error: Unable to read map '" << opt.map_file << "' : " << ohm::errorCodeString(res) << std::endl;
  }


  prog.joinThread();

  return res;
}
