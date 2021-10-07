// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMDEFAULTLAYER_H
#define OHMDEFAULTLAYER_H

#include "OhmConfig.h"

namespace ohm
{
namespace default_layer
{
/// Name of the occupancy layer.
/// @return "occupancy"
const char *ohm_API occupancyLayerName();
/// Name of the @c VoxelMean layer containing mean voxel coordinates.
/// @return "mean"
const char *ohm_API meanLayerName();
/// Name of the traversal layer.
/// @return "traversal"
const char *ohm_API traversalLayerName();
/// Name of the @c CovarianceVoxel layer containing packed voxel covariances.
/// @return "covariance"
const char *ohm_API covarianceLayerName();
/// Name of the voxel clearance layer.
/// @return "clearance"
const char *ohm_API clearanceLayerName();
/// Name of the voxel intensity layer.
/// @return "intensity"
const char *ohm_API intensityLayerName();
/// Name of the voxel hit miss count layer.
/// @return "hit_miss_count"
const char *ohm_API hitMissCountLayerName();
/// Name of the voxel touch time layer.
/// @return "touch_time"
const char *ohm_API touchTimeLayerName();
/// Name of the voxel incident layer.
/// @return "incident_normal"
const char *ohm_API incidentNormalLayerName();
}  // namespace default_layer

class MapLayout;
class MapLayer;

/// Add the occupancy layer to @p layout.
///
/// This ensures @p layout has a layer with a name matching @p occupancyLayerName() setup to hold @c float occupancy
/// data.
///
/// The function makes no changes if @p layout already has a layer named according to @c occupancyLayerName() , but no
/// validation is performed to ensure that the data contained in that layer matches @c float occupancy data .
///
/// @param layout The @p MapLayout to modify.
/// @return The map layer added or the pre-existing layer named according to @c occupancyLayerName() .
MapLayer *ohm_API addOccupancy(MapLayout &layout);

/// Add the @c VoxelMean layer to @p layout.
///
/// This ensures @p layout has a layer with a name matching @p meanLayerName() setup to hold @c VoxelMean data.
///
/// The function makes no changes if @p layout already has a layer named according to @c meanLayerName() , but no
/// validation is performed to ensure that the data contained in that layer matches @c VoxelMean .
///
/// @param layout The @p MapLayout to modify.
/// @return The map layer added or the pre-existing layer named according to @c meanLayerName() .
/// @see @c voxelmean
MapLayer *ohm_API addVoxelMean(MapLayout &layout);

/// Add the traversal layer to @p layout.
///
/// This ensures @p layout has a layer with a name matching @p traversalLayerName() setup to hold a single @c float per
/// voxel. The layer accumulates the total distance travelled all rays through each voxel.
///
/// The function makes no changes if @p layout already has a layer named according to @c traversalLayerName() , but no
/// validation is performed to ensure that the data contained in that layer matches the expected layout.
///
/// @param layout The @p MapLayout to modify.
/// @return The map layer added or the pre-existing layer named according to @c traversalLayerName() .
MapLayer *ohm_API addTraversal(MapLayout &layout);

/// Add the @c CovarianceVoxel layer to @p layout.
///
/// Similar to @c addVoxelMean() , this function adds @c CovarianceVoxel support using the @c covarianceLayerName() .
///
/// @param layout The @p MapLayout to modify.
/// @return The map layer added or the pre-existing layer named according to @c covarianceLayerName() .
MapLayer *ohm_API addCovariance(MapLayout &layout);

/// Add the voxel clearance layer to @p layout.
///
/// Similar to @c addVoxelMean() , this function adds voxel clearance using the @c clearanceLayerName() .
///
/// @param layout The @p MapLayout to modify.
/// @return The map layer added or the pre-existing layer named according to @c clearanceLayerName() .
MapLayer *ohm_API addClearance(MapLayout &layout);

/// Add the voxel intensity (mean and covariance) layer to @p layout.
///
/// Similar to @c addVoxelMean(), this function adds voxel intensity (mean and covariance) using the
/// @c intensityLayerName() .
///
/// @param layout The @p MapLayout to modify.
/// @return The map layer added or the pre-existing layer named according to @c intensityLayerName() .
MapLayer *ohm_API addIntensity(MapLayout &layout);

/// Add the voxel hit and miss counts layer to @p layout.
///
/// Similar to @c addVoxelMean(), this function adds voxel hit count and miss count using the
/// @c hitMissCountLayerName() . The hit count is slightly different to the count in the voxel mean, as it follows the
/// NDT-TM conventions of when to count hits.
///
/// @param layout The @p MapLayout to modify.
/// @return The map layer added or the pre-existing layer named according to @c hitMissCountLayerName() .
MapLayer *ohm_API addHitMissCount(MapLayout &layout);

/// Add the voxel touch time layer to @p layout.
///
/// Similar to @c addVoxelMean(), this function adds touch timestamp using the @c touchTimeLayerName() .
/// The touch time marks the most recent timestamp on which a voxel has been updated. The timestamp stored as a 32-bit
/// unsigned value. The ohm implementation derives this from TODO(KS)
///
/// @param layout The @p MapLayout to modify.
/// @return The map layer added or the pre-existing layer named according to @c touchTimeLayerName() .
MapLayer *ohm_API addTouchTime(MapLayout &layout);

/// Add the voxel incident normal layer to @p layout.
///
/// Similar to @c addVoxelMean(), this function adds voxel incident normals using the @c incidentNormalLayerName() .
/// An incident normal approximates the average incident ray from which sample updates occur. That is, the incident
/// averages all the incoming sample rays for the voxels in which the samples falls - i.e., it only the sample voxel is
/// updated. The normal is an approximate, compressed normal and uses a progressive average.
///
/// @note The incident normal points from the sample voxel back to the ray origin.
///
/// @param layout The @p MapLayout to modify.
/// @return The map layer added or the pre-existing layer named according to @c incidentNormalLayerName() .
MapLayer *ohm_API addIncidentNormal(MapLayout &layout);
}  // namespace ohm

#endif  // OHMDEFAULTLAYER_H
