// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPIMAGE_H
#define HEIGHTMAPIMAGE_H

#include "OhmHeightmapUtilConfig.h"

#include <ohm/Aabb.h>
#include <ohm/UpAxis.h>

#include <glm/fwd.hpp>

#include <memory>

namespace ohm
{
  struct Colour;
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
      kImageHeights,
      /// Image has been generated using vertex colours. This is not be to used in the constructor, but may be returned
      /// as @c generatedImageType() after @c generateBitmap().
      kImageVertexColours888
    };

    /// Information about the generated bitmap.
    struct BitmapInfo
    {
      /// Image pixel width.
      unsigned image_width;
      /// Image pixel height.
      unsigned image_height;
      /// Number of bytes per pixel.
      unsigned bpp;
      /// Details on the pixel format.
      ImageType type;
      /// Number of bytes in the image.
      size_t byte_count;
      /// Image spatial extents.
      Aabb image_extents;
    };

    HeightmapImage(ImageType type = kImageNormals, unsigned pixels_per_voxel = 1);
    ~HeightmapImage();

    ImageType desiredImageType();
    void setDesiredImageType(ImageType type);

    unsigned pixelsPerVoxel();
    void setPixelsPerVoxel(unsigned ppv);

    bool showWindow() const;
    void setShowWindow(bool show_window);

    bool blockOnShowWindow() const;
    void setBlockOnShowWindow(bool block);

    /// Query the number of bytes required to extract the heightmap into a bitmap using @c extractBitmap().
    ///
    /// Note the @c ImageType in @p info will differ from @c desiredImageType() when vertex colours were provided in
    /// generation. In this case the @c ImageType will be @c kImageVertexColours888.
    ///
    /// @return The number of bytes required for the bitmap.
    const uint8_t *bitmap(BitmapInfo *info) const;

    /// Render an image from the given heightmap mesh.
    ///
    /// Pixels are coloured according to @c desiredImageType() or using the provided @c colours. When @p colours are
    /// provided, the image is rendered using vertex colouring, interpolated across the mesh. The colour format is RGB24
    /// using 3 bytes per pixel using 1 byte each for RGB colour channels.
    ///
    /// Withour explicit @c colours, both @c kImageNormals888 and @c kImageNormals the colours represent the surface
    /// normal. A horizontal surface maps to the colour (0.5, 0.5, 1.0) and deviations from horizontal decrease the
    /// blue channel towards 0.5, while increasing the red and green channels. The surface normal is essentially
    /// recovered by remapping each channel from the range [0, 1] to the range [-1, 1].
    ///
    /// The bitmap format for @c kImageNormals888 is RGB24 as described above. When using @c kImageNormals the colour
    /// format is RGB float 32, where each colour channel is represented by a 32-bit float.
    ///
    /// In @p kImageHeights, each pixel is a 32-bit float representing the relative depth.
    ///
    /// Note the generated image format is only attainable by calling @c bitmap() and reading the @c BitmapInfo.
    ///
    ///
    /// @param extents Defines the axis aligned bounding box enclosing the mesh to render. This may be larger than that
    ///   required, in which case the image will feature a black border.
    /// @param resolution The spatial resolution of each vertex. This maps to the resolution fo the voxel map used to
    ///   generate the mesh and is used to resolve @p pixelsPerVoxel().
    /// @param vertices The vertex array.
    /// @param vertex_count Number of elements in @p vertices.
    /// @param indices The triangle index array, three indices per triangle.
    /// @param index_count Number of indices; must be a multiple of 3.
    /// @param vertex_normals The normal at each vertex. Required for @c kImageNormals and kImagenormals888.
    /// @param colours Per vertex colours override. Forces the generated image to be @c kImageVertexColours888.
    ///     Various overloads support different input colour formats.
    /// @param up_axis Identifies the up axis.
    bool generateBitmap(const Aabb &extents, double resolution,
                        const glm::dvec3 *vertices, size_t vertex_count,
                        const unsigned *indices, size_t index_count,
                        const glm::dvec3 *vertex_normals = nullptr,
                        const Colour *colours = nullptr,
                        UpAxis up_axis = UpAxis::Z);

    /// @overload
    bool generateBitmap(const Aabb &extents, double resolution,
                        const glm::dvec3 *vertices, size_t vertex_count,
                        const unsigned *indices, size_t index_count,
                        const glm::dvec3 *vertex_normals,
                        const glm::vec3 *colours,
                        UpAxis up_axis = UpAxis::Z);

    /// @overload
    bool generateBitmap(const Aabb &extents, double resolution,
                        const glm::dvec3 *vertices, size_t vertex_count,
                        const unsigned *indices, size_t index_count,
                        const glm::dvec3 *vertex_normals,
                        const glm::dvec3 *colours,
                        UpAxis up_axis = UpAxis::Z);

    /// Generate a bitmap image from a @c HeightmapMesh. Does not support @c kImageVertexColours888 image mode.
    /// @param mesh The mesh to generate an image of.
    /// @param up_axis Identifies the up axis.
    bool generateBitmap(const HeightmapMesh &mesh, UpAxis up_axis = UpAxis::Z);

  private:
    template <typename NORMAL_VEC3, typename COLOUR_VEC>
    bool renderHeightMesh(ImageType type, const Aabb &spatial_extents,
                          double voxel_resolution,
                          const glm::dvec3 *vertices, size_t vertex_count,
                          const unsigned *indices, size_t index_count,
                          const NORMAL_VEC3 *vertex_normals,
                          const COLOUR_VEC *colours,
                          UpAxis up_axis);

    std::unique_ptr<HeightmapImageDetail> imp_;
  };
}  // namespace ohm

#endif  // HEIGHTMAPIMAGE_H
