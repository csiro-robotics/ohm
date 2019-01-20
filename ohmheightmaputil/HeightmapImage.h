// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPIMAGE_H
#define HEIGHTMAPIMAGE_H

#include "OhmHeightmapUtilConfig.h"

#include <ohm/Aabb.h>

#include <glm/fwd.hpp>

#include <memory>

namespace ohm
{
  class Heightmap;
  class HeightmapMesh;

  struct HeightmapImageDetail;

  /// Experimental conversion of a @c Heightmap into an image by rendering to an OpenGL FGO.
  ///
  /// The resulting image may be either an RGB image where the RGB values map the local surface normals (like a
  /// normal map in rendering) or a grey scale depth value, relative to the min/max for the input data set.
  /// The RGB image may be provided either as a pure RGB uint8 format (1 byte per channel) using @c kImageNormals888
  /// or as 32-bit floating point colour channels ranging [0, 1] using @c kImageNormals. Using @c kImageNormals, the
  /// colour channels need to be remapped from [0, 1] to [-1, 1] by a shift and scale (2 * c - 1).
  ///
  /// Surface normals are derived from the local vertex normals and interpolated across the surface. The vertex normals
  /// are calculated according to @c NormalsMode.
  ///
  /// Typical usage is:
  /// - Generate a @c Heightmap.
  /// - Construct a @c HeightmapImage around the @c Heightmap.
  /// - Call @c generateBitmap()
  /// - Fetch the results using @c bitmap().
  /// - Dispose of the @c HeightmapImage before the @c Heightmap.
  class ohmheightmaputil_API HeightmapImage
  {
  public:
    enum ImageType
    {
      /// Extracted image consists of three floats per pixel. A normal can be derived from each pixel by taking the
      /// RGB float values and converting each channel by multiplying it by 2 and subtracting 1. This is to convert from
      /// the range [0, 1] to the range [-1, 1]. This is shown below.
      /// @code
      /// glm::vec3 convertColourToNormal(const glm::vec3 &c)
      /// {
      ///   return 2.0f * c - glm::vec3(1.0f);
      /// }
      /// @endcode
      kImageNormals,
      /// Extract RBG 888 image where colours represent surface normals.
      kImageNormals888,
      /// Extract a depth image with 4-byte float values for each pixel (depth).
      kImageHeights
    };

    struct BitmapInfo
    {
      unsigned image_width;
      unsigned image_height;
      unsigned bpp;
      ImageType type;
      size_t byte_count;
      Aabb image_extents;
    };

    HeightmapImage(const Heightmap &heightmap, ImageType type = kImageNormals, unsigned pixels_per_voxel = 1);
    ~HeightmapImage();

    /// Access the mesh building class. May be used to configure mesh building.
    /// @return The heightmap mesh builder use.
    HeightmapMesh &meshBuilder();

    /// Access the mesh building class.
    /// @return The heightmap mesh builder use.
    const HeightmapMesh &meshBuilder() const;

    ImageType imageType();
    void setImageType(ImageType type);
    unsigned pixelsPerVoxel();
    void setPixelsPerVoxel(unsigned ppv);

    /// Query the number of bytes required to extract the heightmap into a bitmap using @c extractBitmap().
    /// @return The number of bytes required for the bitmap.
    const uint8_t *bitmap(BitmapInfo *info) const;

    /// Extract the heightmap into a bitmap buffer. The bitmap format is a grey scale, 1 byte per pixel bitmap image.
    ///
    /// Pixels are coloured as heightmap surface normals here RGB pixels map directly to XYZ normal components. The
    /// normals are mapped such that the normal (0, 0, 1) is directly out of the heightmap surface (horizontal surface)
    /// regardless of the original heightmap orientation.
    ///
    bool generateBitmap();

  private:
    bool renderHeightMesh(const glm::dvec3 &min_ext_spatial, const glm::dvec3 &max_ext_spatial, ImageType type,
                          double voxel_resolution);

    std::unique_ptr<HeightmapImageDetail> imp_;
  };
}  // namespace ohm

#endif  // HEIGHTMAPIMAGE_H
