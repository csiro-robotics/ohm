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

  struct HeightmapImageDetail;

  class ohmheightmaputil_API HeightmapImage
  {
  public:
    enum ImageType
    {
      /// Extract RBG 888 image where colours represent surface normals.
      /// TODO(KS): improve the normal resolution by converting into
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

    HeightmapImage(const Heightmap &heightmap, ImageType type = kImageNormals888, unsigned pixels_per_voxel = 1);
    ~HeightmapImage();

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
    void triangulate();
    bool renderHeightMesh(const glm::dvec3 &min_ext_spatial, const glm::dvec3 &max_ext_spatial,
                          ImageType type, double voxel_resolution);

    std::unique_ptr<HeightmapImageDetail> imp_;
  };
}  // namespace ohm

#endif  // HEIGHTMAPIMAGE_H
