// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Jason Williams
#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include "OhmConfig.h"

#include "Aabb.h"
#include "HeightmapVoxelType.h"
#include "UpAxis.h"

#include <memory>

#include <glm/fwd.hpp>

#include <vector>

namespace ohm
{
  class Key;
  class MapInfo;
  class OccupancyMap;
  class VoxelConst;
  class SnapshotDetail;
  class MapChunk;
  class MapCache;

  /// Extract a simplified snapshot of a 3D occupancy map.
  ///

  class Snapshot
  {
  public:
    enum State : uint8_t
    {
      kFree = 0,
      kOccupied = 1,
      kUnknown = 2,
      kNonLeaf = 3
    };

    struct SnapshotNode
    {
      SnapshotNode();
      SnapshotNode(const MapChunk &ch, const OccupancyMap &map, MapCache &map_cache);
      /// Make a snapshot chunk out of this part of region. Valid indices are min_ext.x .. max_ext.x-1, etc
      SnapshotNode(const glm::i16vec3 &region_key, const OccupancyMap &map, MapCache &map_cache, glm::u8vec3 min_ext,
                   glm::u8vec3 max_ext);
      ~SnapshotNode();
      // Returns the number of free, occupied and unknown voxels underneath node
      std::tuple<uint16_t, uint16_t, uint16_t> simplify(float voxel_size);
      // Layer 0 = root (layer \in (0,..,treeHeight()-1))
      void getVoxels(size_t layer, float voxel_size, glm::f32vec3 centre,
                     std::vector<std::pair<uint8_t, glm::f32vec3>> &voxels) const;
      // Tree height ()
      size_t treeHeight() const;
      // Return approximate memory use
      size_t memoryUse() const;
      // Return total number of nodes (packed memory use is number of nodes divided by four)
      size_t numNode() const;

      SnapshotNode &operator=(SnapshotNode &&s);

    private:
      State state_;
      std::unique_ptr<std::array<SnapshotNode, 8>> children_;
    };

    struct SnapshotChunk
    {
      SnapshotChunk();
      SnapshotChunk(const MapChunk &ch, const OccupancyMap &map, MapCache &map_cache);
      /// Make a snapshot chunk out of this part of region. Valid indices are min_ext.x .. max_ext.x-1, etc
      SnapshotChunk(const glm::i16vec3 &region_key, const OccupancyMap &map, MapCache &map_cache, glm::u8vec3 min_ext,
                    glm::u8vec3 max_ext);
      ~SnapshotChunk();
      void simplify(float voxel_size);
      // Layer 0 = root (layer \in (0,..,treeHeight()-1))
      void getVoxels(size_t layer, float voxel_size, glm::f32vec3 centre,
                     std::vector<std::pair<uint8_t, glm::f32vec3>> &voxels) const;
      // Tree height ()
      size_t treeHeight() const;
      // Return approximate memory use
      size_t memoryUse() const;

      SnapshotChunk &operator=(SnapshotChunk &&s);

    private:
      uint8_t p_occ_, min_p_occ_, max_p_occ_; // node empty/invalid if min_p_occ_ > max_p_occ_
      std::unique_ptr<std::array<SnapshotChunk, 8>> children_;
    };


    /// Construct a default initialised snapshot object.
    Snapshot();

    /// Destructor.
    ~Snapshot();

    /// Generate the snapshot around a reference position. 
    ///
    /// @param cull_to The box (xy) to restrict snapshot to.
    /// @return a vector of chunk centres and mean occupancy.
    std::vector<std::pair<float, std::vector<std::pair<uint8_t, glm::f32vec3>>>> buildSnapshot(
      const ohm::Aabb &cull_to = ohm::Aabb(0.0));

    /// Set the (source) occupancy map. Does not take ownership of the pointer so the @p map must persist until 
    /// @c buildSnapshot() is called.
    void setOccupancyMap(OccupancyMap *map);

    /// Access the current source occupancy map.
    OccupancyMap *occupancyMap() const;

  private:
    std::unique_ptr<SnapshotDetail> imp_;
  };
}  // namespace ohm

#endif  // HEIGHTMAP_H
