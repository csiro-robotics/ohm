// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmtestutil.h"

#include <ohm/mapchunk.h>
#include <ohm/maplayer.h>
#include <ohm/maplayout.h>
#include <ohm/occupancymap.h>
#include <ohm/occupancynode.h>

#include <gtest/gtest.h>

using namespace ohm;

namespace ohmtestutil
{
  bool compareLayout(const OccupancyMap &map, const OccupancyMap &referenceMap)
  {
    const MapLayout &layout = map.layout();
    const MapLayout &refLayout = referenceMap.layout();
    bool layoutMatches = layout.layerCount() == refLayout.layerCount();
    EXPECT_EQ(layout.layerCount(), refLayout.layerCount());

    for (unsigned i = 0; layoutMatches && i < layout.layerCount(); ++i)
    {
      const MapLayer &layer = layout.layer(i);
      const MapLayer &refLayer = refLayout.layer(i);

      layoutMatches = layoutMatches && strcmp(layer.name(), refLayer.name()) == 0;
      EXPECT_STREQ(layer.name(), refLayer.name());
      layoutMatches = layoutMatches && layer.layerIndex() == refLayer.layerIndex();
      EXPECT_EQ(layer.layerIndex(), refLayer.layerIndex());
      layoutMatches = layoutMatches && layer.subsampling() == refLayer.subsampling();
      EXPECT_EQ(layer.subsampling(), refLayer.subsampling());
      layoutMatches = layoutMatches && layer.flags() == refLayer.flags();
      EXPECT_EQ(layer.flags(), refLayer.flags());
      layoutMatches = layoutMatches && layer.dimensions(map.regionVoxelDimensions()) ==
                                         refLayer.dimensions(referenceMap.regionVoxelDimensions());
      EXPECT_EQ(layer.dimensions(map.regionVoxelDimensions()),
                refLayer.dimensions(referenceMap.regionVoxelDimensions()));
      layoutMatches = layoutMatches && layer.layerByteSize(map.regionVoxelDimensions()) ==
                                         refLayer.layerByteSize(referenceMap.regionVoxelDimensions());
      EXPECT_EQ(layer.layerByteSize(map.regionVoxelDimensions()),
                refLayer.layerByteSize(referenceMap.regionVoxelDimensions()));

      VoxelLayoutConst voxelLayout = layer.voxelLayout();
      VoxelLayoutConst refVoxelLayout = refLayer.voxelLayout();

      layoutMatches = layoutMatches && voxelLayout.memberCount() == refVoxelLayout.memberCount();
      EXPECT_EQ(voxelLayout.memberCount(), refVoxelLayout.memberCount());
      layoutMatches = layoutMatches && voxelLayout.voxelByteSize() == refVoxelLayout.voxelByteSize();
      EXPECT_EQ(voxelLayout.voxelByteSize(), refVoxelLayout.voxelByteSize());

      for (unsigned v = 0; layoutMatches && v < voxelLayout.memberCount(); ++v)
      {
        layoutMatches = layoutMatches && strcmp(voxelLayout.memberName(v), refVoxelLayout.memberName(v)) == 0;
        EXPECT_STREQ(voxelLayout.memberName(v), refVoxelLayout.memberName(v));
        layoutMatches = layoutMatches && voxelLayout.memberType(v) == refVoxelLayout.memberType(v);
        EXPECT_EQ(voxelLayout.memberType(v), refVoxelLayout.memberType(v));
        layoutMatches = layoutMatches && voxelLayout.memberClearValue(v) == refVoxelLayout.memberClearValue(v);
        EXPECT_EQ(voxelLayout.memberClearValue(v), refVoxelLayout.memberClearValue(v));
      }
    }

    return layoutMatches;
  }


  void compareMaps(const OccupancyMap &map, const OccupancyMap &referenceMap, unsigned compareFlags)
  {
    compareMaps(map, referenceMap, glm::dvec3(-std::numeric_limits<double>::infinity()),
                glm::dvec3(std::numeric_limits<double>::infinity()), compareFlags);
  }


  void compareMaps(const OccupancyMap &map, const OccupancyMap &referenceMap, const glm::dvec3 &minExt,
                   const glm::dvec3 maxExt, unsigned compareFlags)
  {
    const bool fullExtents = glm::all(glm::equal(minExt, glm::dvec3(-std::numeric_limits<double>::infinity()))) &&
                             glm::all(glm::equal(maxExt, glm::dvec3(std::numeric_limits<double>::infinity())));

    if (compareFlags & CF_General)
    {
      EXPECT_EQ(map.resolution(), referenceMap.resolution());
      EXPECT_EQ(map.regionSpatialResolution(), referenceMap.regionSpatialResolution());
      EXPECT_EQ(map.regionVoxelDimensions(), referenceMap.regionVoxelDimensions());
      EXPECT_EQ(map.regionVoxelVolume(), referenceMap.regionVoxelVolume());
      EXPECT_EQ(map.origin(), referenceMap.origin());
      if (fullExtents)
      {
        EXPECT_EQ(map.regionCount(), referenceMap.regionCount());
      }
      EXPECT_EQ(map.hitValue(), referenceMap.hitValue());
      EXPECT_EQ(map.missValue(), referenceMap.missValue());
      EXPECT_EQ(map.occupancyThresholdValue(), referenceMap.occupancyThresholdValue());
    }

    // Layout
    // Start with layout not compared. Important so later we can assume layout is OK.
    bool layoutMatches = false;
    if (compareFlags & CF_Layout)
    {
      layoutMatches = compareLayout(map, referenceMap);
    }

    // Chunks
    if (compareFlags & CF_ChunksGeneral)
    {
      std::vector<const MapChunk *> refChunks;
      referenceMap.enumerateRegions(refChunks);

      for (const MapChunk *refChunk : refChunks)
      {
        const MapChunk *chunk = map.region(refChunk->region.coord);

        if (fullExtents || refChunk->overlapsExtents(minExt, maxExt, map.regionSpatialResolution()))
        {
          EXPECT_TRUE(chunk != nullptr);
        }

        if (!chunk)
        {
          continue;
        }

        EXPECT_EQ(chunk->region.centre, refChunk->region.centre);
        EXPECT_EQ(chunk->region.coord, refChunk->region.coord);
        EXPECT_EQ(chunk->region.hash, refChunk->region.hash);

        if (compareFlags & CF_ChunksFine)
        {
          EXPECT_EQ(chunk->firstValidIndex, refChunk->firstValidIndex);
          EXPECT_EQ(chunk->touchedTime, refChunk->touchedTime);
          EXPECT_EQ(chunk->dirtyStamp, refChunk->dirtyStamp);
          for (unsigned i = 0; i < chunk->layout->layerCount(); ++i)
          {
            EXPECT_EQ(chunk->touchedStamps[i], refChunk->touchedStamps[i]);
          }
          EXPECT_EQ(chunk->flags, refChunk->flags);
        }
      }
    }

    // Voxels
    if (compareFlags & (CF_Occupancy | CF_Clearance))
    {
      bool haveValidClerance = false;
      OccupancyNodeConst mapNode;
      OccupancyNodeConst refNode;
      for (auto iter = referenceMap.begin(); iter != referenceMap.end(); ++iter)
      {
        mapNode = *iter;
        refNode = map.node(iter.key());

        if (fullExtents || mapNode.chunk()->overlapsExtents(minExt, maxExt, map.regionSpatialResolution()))
        {
          ASSERT_TRUE(refNode.isValid());
        }

        if (!refNode.isValid())
        {
          continue;
        }

        ASSERT_EQ(refNode.key(), mapNode.key());
        if (compareFlags & CF_Occupancy)
        {
          ASSERT_EQ(refNode.occupancy(), mapNode.occupancy());
        }

        if (compareFlags & CF_Clearance)
        {
          ASSERT_EQ(refNode.clearance(), mapNode.clearance());
          haveValidClerance = haveValidClerance || refNode.clearance() >= 0.0f;
        }
      }

      if (compareFlags & CF_ExpectClearance)
      {
        EXPECT_TRUE(haveValidClerance);
      }
    }
  }
}
