// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAP_H
#define HEIGHTMAP_H

#include "OhmConfig.h"

#include <memory>

#include <glm/fwd.hpp>

namespace ohm
{
  struct HeightmapDetail;
  class MapInfo;
  class OccupancyMap;

  /// A 2D voxel map variant which calculate a heightmap surface from another @c OccupancyMap.
  ///
  /// The heightmap is built from an @c OccupancyMap and forms an axis aligned collapse of that map. The up axis may be
  /// specified on construction of the heightmap, but must be aligned to a primary axis. The heightmap is build in
  /// its own @c OccupancyMap, which consists of a single layer of voxels. The @c MapLayout for the heightmap is
  /// two layers:
  /// - **occupancy** layer
  ///   - float occupancy
  /// - *heightmap* layer (named from @c HeightmapVoxel::kHeightmapLayer)
  ///   - @c HeightmapVoxel
  ///
  /// The height specifies the absolute height of the surface, while clearance denotes how much room there is above
  /// the surface voxel before the next obstruction. Note that the height values always increase going up, so the
  /// height value will be inverted when using any @c AxisNegN @c Axis value. Similarly, the clearance is always
  /// positive unless there are no further voxels above the surface, in which case the clearance is zero
  /// (no information).
  ///
  /// The height for each heightmap voxel is simplistically calculated by finding the its corresponding voxel column in
  /// the source map. The algorithm then walks up this column. By default, there is only one column per heightmap voxel.
  /// This may create incorrect results when the two maps use different voxel resolutions.
  ///
  /// A @c blurLevel() may be set to expand the column. Each blur factor expands the search space into a square around
  /// the original column. A blur level of 1 tests 9 voxels per source map layer to populate a heightmap voxel, blur 2
  /// uses 25, 3 uses 49, etc.
  ///
  /// The resulting heightmap may be accessed via @c heightmap().
  ///
  /// The @c OccupancyMap used to represent the heightmap has additional meta data stored in it's @c MapInfo:
  /// - <b>heightmap</b> - Present and true if this is a heightmap.
  /// - <b>heightmap-axis</b> - The up axis ID for a heightmap.
  /// - <b>heightmap-axis-x</b> - The up axis X value for a heightmap.
  /// - <b>heightmap-axis-y</b> - The up axis Y value for a heightmap.
  /// - <b>heightmap-axis-z</b> - The up axis Z value for a heightmap.
  /// - <b>heightmap-blur</b> - The blur value used to generate the heightamp.
  /// - <b>heightmap-clearance</b> - The clearance value used to generate the heightamp.
  ///
  /// @todo Is this a @c Query?
  class Heightmap
  {
  public:
    /// Possible values for setting the @c upAxis() in @c update().
    enum Axis : int
    {
      AxisNegZ = -3,
      AxisNegY = -2,
      AxisNegX = -1,
      AxisX,
      AxisY,
      AxisZ,
    };

    static const unsigned kDefaultRegionSize = 128;

    /// Construct a default initialised heightmap.
    Heightmap();

    /// Construct a new heightmap optionally tied to a specific @p map.
    /// @param grid_resolution The grid resolution for the heightmap. Should match the source map for best results.
    /// @param min_clearance The minimum clearance value expected above each surface voxel.
    /// @param up_axis Identifies the up axis for the map.
    /// @param region_size Grid size of each region in the heightmap.
    Heightmap(double grid_resolution, double min_clearance, Axis up_axis = AxisZ, unsigned region_size = 0);

    /// Destructor.
    ~Heightmap();

    /// Set the occupancy map on which to base the heightmap. The heightmap does not takes no ownership of the pointer
    /// so the @p map must persist until @c update() is called.
    void setOccupancyMap(OccupancyMap *map);

    /// Access the current source occupancy map.
    OccupancyMap *occupancyMap() const;

    /// Access the currently generated heightmap.
    OccupancyMap &heightmap() const;

    /// Set the minimum clearance required above a voxel in order to consider it a heightmap voxel.
    /// @param clearance The new clearance value.
    void setMinClearance(double clearance);

    /// Get the minimum clearance required above a voxel in order to consider it a heightmap voxel.
    /// @return The height clearance value.
    double minClearance() const;

    /// Sets the blur level. See class comments.
    void setBlurLevel(int blur);

    /// Gets the blur level. See class comments.
    int blurLevel() const;

    /// The layer number which contains @c HeightmapVoxel structures.
    unsigned heightmapVoxelLayer() const;

    /// Get the up axis identifier used to generate the heightmap.
    Axis upAxis() const;

    /// Get the up axis index [0, 2] marking XYZ respectively. Ignores direction.
    int upAxisIndex() const;

    /// Get the normal vector for the up axis used to last @c update().
    const glm::dvec3 &upAxisNormal() const;

    /// Static resolution of @c Axis to a normal.
    /// @param id The @c Axis ID.
    static const glm::dvec3 &upAxisNormal(int axis_id);

    /// The last base height value given to @p update().
    double baseHeight() const;

    /// Update the heightmap from the source @c occupancyMap(). This clears the existing content first.
    ///
    /// Voxels are project onto this plane to calculate each voxels' closest two clusters to the plane.
    ///
    /// @param base_height The base heightmap value. All heights are relative to this value. This helps reduce floating
    ///   point error with heights being stored in single precision.
    /// @return true on success.
    bool update(double base_height);

    //-------------------------------------------------------
    // Internal
    //-------------------------------------------------------
    /// @internal
    inline HeightmapDetail *detail() { return imp_.get(); }
    /// @internal
    inline const HeightmapDetail *detail() const { return imp_.get(); }

    /// Update @c info to reflect the details of how the heightmap is generated. See class comments.
    /// @param info The info object to update.
    void updateMapInfo(MapInfo &info) const;

  private:
    std::unique_ptr<HeightmapDetail> imp_;
  };
}  // namespace ohm

#endif  // HEIGHTMAP_H
