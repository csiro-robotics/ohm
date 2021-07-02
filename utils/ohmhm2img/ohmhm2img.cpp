//
// author Kazys Stepanas
//

#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Voxel.h>

#include <ohmheightmap/Heightmap.h>
#include <ohmheightmap/HeightmapMesh.h>
#include <ohmheightmap/HeightmapSerialise.h>
#include <ohmheightmap/HeightmapVoxel.h>

#include <ohmheightmapimage/HeightmapImage.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/ProgressMonitor.h>
#include <ohmutil/SafeIO.h>
#include <ohmutil/ScopedTimeDisplay.h>

#include <png.h>

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

enum ExportMode
{
  kNormals16 = ohm::HeightmapImage::kImageNormals,
  kNormals8 = ohm::HeightmapImage::kImageNormals888,
  kHeights = ohm::HeightmapImage::kImageHeights,
  kTraversability
};

enum ExportImageType
{
  kExportError = -1,
  kExportRGB8,
  kExportRGB16,
  kExportGrey8,
  kExportGrey16
};

struct Options
{
  std::string map_file;
  std::string image_file;
  ExportMode image_mode = kNormals16;
  ohm::HeightmapMesh::NormalsMode normals_mode = ohm::HeightmapMesh::kNormalsAverage;
  double traverse_angle = 45.0;  // NOLINT(readability-magic-numbers)

  ohm::HeightmapImage::ImageType imageType() const
  {
    switch (image_mode)
    {
    case kNormals16:
      return ohm::HeightmapImage::kImageNormals;
    case kNormals8:
      return ohm::HeightmapImage::kImageNormals888;
    case kHeights:
      return ohm::HeightmapImage::kImageHeights;
    case kTraversability:
      return ohm::HeightmapImage::kImageNormals;
    default:
      break;
    }

    return ohm::HeightmapImage::kImageNormals;
  }
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

ExportImageType convertImage(std::vector<uint8_t> &export_pixels, const uint8_t *raw,
                             const ohm::HeightmapImage::BitmapInfo &info, const Options &opt)
{
  if (info.type == ohm::HeightmapImage::kImageVertexColours888)
  {
    export_pixels.resize(size_t(info.image_width) * size_t(info.image_height) * 3u);
    memcpy(export_pixels.data(), raw, export_pixels.size());
    return kExportRGB8;
  }

  if (opt.image_mode == kNormals16 && info.type == ohm::HeightmapImage::kImageNormals)
  {
    // Need to convert float colour to u16
    export_pixels.clear();
    export_pixels.reserve(info.image_width * info.image_height * 3 * sizeof(uint16_t));

    float red;
    float green;
    float blue;
    uint16_t red16;
    uint16_t green16;
    uint16_t blue16;

    const auto convert_colour = [](float c) -> uint16_t {
      return uint16_t(c * float(std::numeric_limits<uint16_t>::max()));
    };

    const auto push_channel = [](std::vector<uint8_t> &out, uint16_t c) {
      const size_t insert_index = out.size();
      // Push the stride in bytes.
      for (size_t i = 0; i < sizeof(uint16_t); ++i)
      {
        out.push_back(0);
      }

      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      *reinterpret_cast<uint16_t *>(&out[insert_index]) = c;
    };

    for (size_t i = 0; i < size_t(info.image_width) * size_t(info.image_height) * size_t(info.bpp);
         i += 3 * sizeof(float))
    {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      red = *reinterpret_cast<const float *>(raw + i);
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      green = *reinterpret_cast<const float *>(raw + i + sizeof(float));
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      blue = *reinterpret_cast<const float *>(raw + i + 2 * sizeof(float));

      red16 = convert_colour(red);
      green16 = convert_colour(green);
      blue16 = convert_colour(blue);

      // No data: black
      if (red * red + green * green + blue * blue < 0.5f)
      {
        red16 = green16 = blue16 = 0;
      }

      push_channel(export_pixels, red16);
      push_channel(export_pixels, green16);
      push_channel(export_pixels, blue16);
    }

    return kExportRGB16;
  }

  if (opt.image_mode == kNormals8 && info.type == ohm::HeightmapImage::kImageNormals888)
  {
    export_pixels.resize(size_t(info.image_width) * size_t(info.image_height) * 3u);
    memcpy(export_pixels.data(), raw, export_pixels.size());
    return kExportRGB8;
  }

  if (opt.image_mode == kHeights && info.bpp == sizeof(float))
  {
    export_pixels.resize(info.image_width * info.image_height * sizeof(uint16_t));
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    const auto *depth_pixels = reinterpret_cast<const float *>(raw);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto *depth_out = reinterpret_cast<uint16_t *>(export_pixels.data());

    for (size_t i = 0; i < size_t(info.image_width) * size_t(info.image_height); ++i)
    {
      depth_out[i] = uint16_t(1.0f - depth_pixels[i] * float(std::numeric_limits<uint16_t>::max()));
    }

    return kExportGrey16;
  }

  if (opt.image_mode == kTraversability && info.type == ohm::HeightmapImage::kImageNormals)
  {
    static_assert(sizeof(glm::vec3) == sizeof(float) * 3, "glm::vec3 mismatch");
    export_pixels.resize(size_t(info.image_width) * size_t(info.image_height));

    const uint8_t c_unknown = 127u;
    const uint8_t c_blocked = 0u;
    const uint8_t c_free = 255u;
    glm::vec3 normal{};
    const glm::vec3 flat(0, 0, 1);
    float dot;
    const auto free_threshold = float(std::cos(M_PI * opt.traverse_angle / 180.0));

    for (size_t i = 0; i < size_t(info.image_width) * size_t(info.image_height); ++i)
    {
      memcpy(&normal, &raw[i * sizeof(normal)], sizeof(normal));
      if (glm::dot(normal, normal) > 0.5f * 0.5f)
      {
        normal = 2.0f * normal - glm::vec3(1.0f);
        normal = glm::normalize(normal);
        dot = glm::dot(normal, flat);
        if (dot >= free_threshold)
        {
          export_pixels[i] = c_free;
        }
        else
        {
          export_pixels[i] = c_blocked;
        }
      }
      else
      {
        // No data.
        export_pixels[i] = c_unknown;
      }
    }

    return kExportGrey8;
  }

  return kExportError;
}


bool savePng(const char *filename, const std::vector<uint8_t> &raw, ExportImageType type, unsigned w, unsigned h)
{
  png_image image;

  // Initialize the 'png_image' structure.
  memset(&image, 0, sizeof(image));
  image.version = PNG_IMAGE_VERSION;
  image.width = int(w);
  image.height = int(h);
  image.flags = 0;
  image.colormap_entries = 0;

  int row_stride = int(w);
  switch (type)
  {
  case kExportRGB8:
    image.format = PNG_FORMAT_RGB;
    row_stride = -int(w * 3);
    break;
  case kExportRGB16:
    image.format = PNG_FORMAT_RGB | PNG_IMAGE_FLAG_16BIT_sRGB;  // NOLINT(hicpp-signed-bitwise)
    row_stride = -int(w * 3);
    break;
  case kExportGrey8:
    image.format = PNG_FORMAT_GRAY;
    row_stride = -int(w);
    break;
  case kExportGrey16:
    image.format = PNG_FORMAT_LINEAR_Y;
    row_stride = -int(w);
    break;
  default:
    image.format = PNG_FORMAT_GRAY;
    row_stride = -int(w);
    break;
  }

  // Negative row stride to flip the image.
  return png_image_write_to_file(&image, filename, false,  // convert_to_8bit,
                                 raw.data(),
                                 row_stride,  // row_stride
                                 nullptr      // colormap
  );
}
}  // namespace


// Custom option parsing. Must come before we include Options.h/cxxopt.hpp
std::istream &operator>>(std::istream &in, ExportMode &mode)
{
  std::string mode_str;
  in >> mode_str;
  if (mode_str == "norm8")
  {
    mode = kNormals8;
  }
  else if (mode_str == "norm16")
  {
    mode = kNormals16;
  }
  else if (mode_str == "height")
  {
    mode = kHeights;
  }
  else if (mode_str == "traverse")
  {
    mode = kTraversability;
  }
  // else
  // {
  //   throw cxxopts::invalid_option_format_error(modeStr);
  // }
  return in;
}

std::ostream &operator<<(std::ostream &out, const ExportMode mode)
{
  switch (mode)
  {
  case kNormals8:
    out << "norm8";
    break;
  case kNormals16:
    out << "norm16";
    break;
  case kHeights:
    out << "height";
    break;
  case kTraversability:
    out << "traverse";
    break;
  }
  return out;
}

std::istream &operator>>(std::istream &in, ohm::HeightmapMesh::NormalsMode &mode)
{
  std::string mode_str;
  in >> mode_str;
  if (mode_str == "average" || mode_str == "avg")
  {
    mode = ohm::HeightmapMesh::kNormalsAverage;
  }
  else if (mode_str == "worst")
  {
    mode = ohm::HeightmapMesh::kNormalsWorst;
  }
  // else
  // {
  //   throw cxxopts::invalid_option_format_error(modeStr);
  // }
  return in;
}

std::ostream &operator<<(std::ostream &out, const ohm::HeightmapMesh::NormalsMode mode)
{
  switch (mode)
  {
  case ohm::HeightmapMesh::kNormalsAverage:
    out << "average";
    break;
  case ohm::HeightmapMesh::kNormalsWorst:
    out << "worst";
    break;
  }
  return out;
}
// Must be after argument streaming operators.
#include <ohmutil/Options.h>

int parseOptions(Options *opt, int argc, char *argv[])  // NOLINT(modernize-avoid-c-arrays)
{
  cxxopts::Options opt_parse(argv[0], "\nCreate a heightmap from an occupancy map.\n");
  opt_parse.positional_help("<heightmap.ohm> <image.png>");

  try
  {
    opt_parse.add_options()("help", "Show help.")                                       //
      ("i", "The input heightmap file (ohm).", cxxopts::value(opt->map_file))           //
      ("o", "The output heightmap image file (png).", cxxopts::value(opt->image_file))  //
      ("m,mode",
       "The image output mode [norm8, norm16, height, traverse]. norm8 exports a normal map image with 8 bits per "
       "pixel. norm16 "
       "uses 16 bits per pixel. height is a greyscale image where the colour is the relative heights. traverse colours "
       "by traversability black (non-traversable), white (traversable), grey (unknown) based on the --traverse-angle "
       "argument.",
       cxxopts::value(opt->image_mode)->default_value(optStr(opt->image_mode)))  //
      ("traverse-angle", "The maximum traversable angle (degrees) for use with mode=traverse.",
       cxxopts::value(opt->traverse_angle)->default_value(optStr(opt->traverse_angle)))  //
      ("normals",
       "Defines how vertex normals are calculated: [average/avg, worst]. average averages triangle normals, worst "
       "selects the least horizontal triangle normal for a vertex.",
       cxxopts::value(opt->normals_mode)->default_value(optStr(opt->normals_mode)))  //
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

    if (opt->image_file.empty())
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


std::string generateYamlName(const std::string &image_file)
{
  // Find replace the extension with .yaml
  auto last_dot_pos = image_file.find_last_of('.');
  std::string yaml_name;
  if (last_dot_pos != std::string::npos)
  {
    yaml_name = image_file.substr(0, last_dot_pos);
  }
  else
  {
    yaml_name = image_file;
  }

  yaml_name += ".yaml";
  return yaml_name;
}


bool saveMetaData(const std::string &yaml_file, const Options &opt, ohm::Heightmap &heightmap,
                  const ohm::HeightmapImage::BitmapInfo &info, ExportImageType image_format)
{
  std::ofstream out(yaml_file.c_str());

  if (!out.is_open())
  {
    return false;
  }

  // Set high precision output.
  out << std::setprecision(std::numeric_limits<double>::max_digits10);

  int white_colour = std::numeric_limits<uint8_t>::max();
  const int black_colour = 0;

  // Path to the heightmap image.
  out << "image: " << opt.image_file << '\n';
  // Describe how the image was generated.
  out << "image_mode: " << opt.image_mode << "\n";
  out << "image_format: ";
  switch (image_format)
  {
  case kExportError:
    out << "error";
    break;
  case kExportRGB8:
    white_colour = std::numeric_limits<uint8_t>::max();
    out << "RGB8";
    break;
  case kExportRGB16:
    white_colour = std::numeric_limits<uint16_t>::max();
    out << "RGB16";
    break;
  case kExportGrey8:
    white_colour = std::numeric_limits<uint8_t>::max();
    out << "mono8";
    break;
  case kExportGrey16:
    white_colour = std::numeric_limits<uint16_t>::max();
    out << "mono16";
    break;
  default:
    out << "unknown";
    break;
  }
  out << "\n";

  // Convert the bitmap image global min extents into a 2D value.
  glm::dvec3 min_ext_2d =
    info.image_extents.minExtents() - glm::dot(heightmap.upAxisNormal(), info.image_extents.minExtents());
  glm::dvec3 origin(0.0);
  origin.x = glm::dot(min_ext_2d, heightmap.surfaceAxisA());
  origin.y = glm::dot(min_ext_2d, heightmap.surfaceAxisB());
  origin.z = 0;  // Yaw

  // Image resolution in units/pixel. Units are the same as those used to generate the heightmap, generally metres.
  out << "resolution: " << heightmap.heightmap().resolution() << '\n';
  // The 2D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise rotation (degrees)
  // yaw = 0 means no rotation).
  out << "origin: [" << origin.x << ", " << origin.y << ", " << origin.z << "]\n";

  // Mode specific information
  switch (opt.image_mode)
  {
  case kTraversability:
    // Pixels with this colour value are considered obstructed.
    out << "obstructed_value: " << black_colour << "\n";  // Black
    // Pixels with this colour value are considered free/traversable.
    out << "free_value: " << white_colour << "\n";  // White
    // Traversability angle (degrees)
    out << "traverse_angle: " << opt.traverse_angle << "\n";
    break;
  default:
    break;
  }

  out << "up: [" << heightmap.upAxisNormal().x << ", " << heightmap.upAxisNormal().y << ", "
      << heightmap.upAxisNormal().z << "]\n";

  out << std::flush;
  out.close();

  return true;
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

  printf("Loading map %s\n", opt.map_file.c_str());
  ProgressMonitor prog(10);
  LoadMapProgress load_progress(prog);
  ohm::Heightmap heightmap;
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
  res = ohm::load(opt.map_file.c_str(), heightmap, &load_progress, &version);
  prog.endProgress();

  std::cout << std::endl;

  if (res != 0)
  {
    std::cerr << "Failed to load heightmap. Error(" << res << "): " << ohm::serialiseErrorCodeString(res) << std::endl;
    return res;
  }

  ohm::HeightmapImage hm_image(opt.imageType());
  ohm::HeightmapImage::BitmapInfo info;
  ohm::HeightmapMesh mesh_builder(opt.normals_mode);

  mesh_builder.buildMesh(heightmap);

  hm_image.generateBitmap(mesh_builder, heightmap.upAxis());
  const uint8_t *image = hm_image.bitmap(&info);
  if (!image)
  {
    std::cerr << "Failed to generate heightmap image" << std::endl;
    return 1;
  }

  std::vector<uint8_t> export_pixels;
  ExportImageType export_type = convertImage(export_pixels, image, info, opt);

  std::cout << "Saving " << opt.image_file << std::endl;
  if (!savePng(opt.image_file.c_str(), export_pixels, export_type, info.image_width, info.image_height))
  {
    std::cerr << "Failed to save heightmap image" << std::endl;
    return 1;
  }

  std::string info_yaml_name = generateYamlName(opt.image_file);
  if (!saveMetaData(info_yaml_name, opt, heightmap, info, export_type))
  {
    std::cerr << "Failed to save yaml" << std::endl;
    return 1;
  }

  return res;
}
