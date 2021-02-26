// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_COMPARE_H
#define OHM_COMPARE_H

#include "OhmConfig.h"

#include "VoxelBlock.h"
#include "VoxelBuffer.h"
#include "VoxelLayout.h"

#include <functional>
#include <string>

#define OHM_CMP_FAIL(flags, ret)              \
  if (((flags)&ohm::compare::kContinue) == 0) \
  {                                           \
    return (ret);                             \
  }

namespace ohm
{
class Key;
class OccupancyMap;

/// An experimental set of functions for comparing maps.
namespace compare
{
/// Severity of a comparison message.
enum class Severity
{
  kInfo,
  kWarning,
  kError
};

/// Comparison flag values.
enum Flag : unsigned
{
  kZero = 0u,             ///< Zero value.
  kContinue = (1u << 0u)  ///< Continue on error.
};

/// Results on comparing voxels
struct ohm_API VoxelsResult
{
  size_t voxels_passed = 0;   ///< Number of voxels passed.
  size_t voxels_failed = 0;   ///< Number of voxels failed.
  bool layout_match = false;  ///< Results of layout check. Must pass to compare content.

  /// Conversion to bool value.
  explicit inline operator bool() const { return voxels_failed == 0 && layout_match; }
};

/// Logging function type.
using Log = std::function<void(Severity, const std::string &)>;

/// Empty/dummy logging function.
inline void ohm_API emptyLog(Severity /*severity*/, const std::string & /*msg*/){};

/// Compare the layout of a layer in two maps.
///
/// This first checks that the layer exists in both maps, then gradually delves down to validating the content.
/// Compares:
/// - Voxel byte size
/// - Number of voxel members.
/// - Member alignment.
/// - Member type.
/// - Member size.
///
/// @param eval_map The map to evaluate.
/// @param ref_map The reference map.
/// @param layer_name The name fo the layer to compare layouts.
/// @param flags See @c Flag values.
/// @param log Logging function.
/// @return False if any validation step fails.
bool ohm_API compareLayoutLayer(const OccupancyMap &eval_map, const OccupancyMap &ref_map,
                                const std::string &layer_name, unsigned flags = 0, Log log = emptyLog);


/// Compare the voxel identified by @p key in two voxel buffers. The presumptions are that the same key is referencing
/// equivalent voxels from two different maps.
///
/// This checks the content member by member.
///
/// @param key Identifies the voxel of interest. Must be valid in both buffers.
/// @param eval_buffer The voxel buffer containing the data to evaluate. May be null, otherwise the @p key be a valid
/// key into this buffer. A null buffer is a failure condition.
/// @param eval_voxel_layout The voxel layer associated with the @p eval_buffer.
/// @param ref_buffer The voxel buffer containing the reference data. May be null, otherwise the @p key be a valid
/// key into this buffer. A null buffer is a failure condition.
/// @param ref_voxel_layout The voxel layer associated with the @p ref_buffer.
/// @param log Logging function.
/// @return False if any validation step fails.
bool ohm_API compareVoxel(const Key &key, VoxelBuffer<const VoxelBlock> &eval_buffer,
                          VoxelLayoutConst &eval_voxel_layout, VoxelBuffer<const VoxelBlock> &ref_buffer,
                          VoxelLayoutConst &ref_voxel_layout, Log log = emptyLog);

/// Compare the layer content for all voxels in @p ref_map ensuring they exit in, and match in @c eval_map.
///
/// @param eval_map The map to evaluate.
/// @param ref_map The reference map.
/// @param flags See @c Flag values.
/// @param layer_name The name fo the layer to compare voxel content.
/// @param log Logging function.
/// @return False if any validation step fails.
VoxelsResult ohm_API compareVoxels(const OccupancyMap &eval_map, const OccupancyMap &ref_map,
                                   const std::string &layer_name, unsigned flags = 0, Log log = emptyLog);
}  // namespace compare
}  // namespace ohm

#endif  // OHM_COMPARE_H
