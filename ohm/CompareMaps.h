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
/// This checks the content member by member. A tolerance in value differences is supported via the @p toleranace
/// object. Whenever a member is matched for value comparison, we looking the @c tolerance for a member of the same
/// name (and type). When found, the @c VoxelLayoutT<T>::memberClearValue() is read and used as the allowed absolute
/// error difference between the eval map and the reference map.
///
/// For example, the following code configures an allowed deviation in occupancy value of:
///
/// @code
/// // Instantiate the tolerance object.
/// ohm::MapLayer occupancy_tolerance("occupancy");
/// auto voxel_layout = occupancy_tolerance.voxelLayout();
/// // Add an 'occupancy' member.
/// float tolerance = 1e-2f;
/// voxel_layout.addMember("occupancy", ohm::DataType::kFloat, *(uint64_t*)&tolerance);
/// @endcode
///
/// A convenience function, @c configureTolerance(), is provided to assist in configuring tolerances.
/// This changes the example above to:
///
/// @code
/// // Instantiate the tolerance object.
/// ohm::MapLayer occupancy_tolerance("occupancy");
/// ohm::compare::configureTolerance(occupancy_tolerance, "occupancy", 1e-2f);
/// @endcode
///
/// Hopefully this usage of @c MapLayer doesn't prove "too clever".
///
/// @param key Identifies the voxel of interest. Must be valid in both buffers.
/// @param eval_buffer The voxel buffer containing the data to evaluate. May be null, otherwise the @p key be a valid
/// key into this buffer. A null buffer is a failure condition.
/// @param eval_voxel_layout The voxel layer associated with the @p eval_buffer.
/// @param ref_buffer The voxel buffer containing the reference data. May be null, otherwise the @p key be a valid
/// key into this buffer. A null buffer is a failure condition.
/// @param ref_voxel_layout The voxel layer associated with the @p ref_buffer.
/// @param log Logging function.
/// @param tolerance A dummy @c MapLayer object which wraps the allowed tolerances for differences in voxel member
/// values.
/// @return False if any validation step fails.
bool ohm_API compareVoxel(const Key &key, VoxelBuffer<const VoxelBlock> &eval_buffer,
                          VoxelLayoutConst &eval_voxel_layout, VoxelBuffer<const VoxelBlock> &ref_buffer,
                          VoxelLayoutConst &ref_voxel_layout, const MapLayer *tolerance = nullptr, Log log = emptyLog);

/// Compare the layer content for all voxels in @p ref_map ensuring they exit in, and match in @c eval_map.
///
/// @param eval_map The map to evaluate.
/// @param ref_map The reference map.
/// @param flags See @c Flag values.
/// @param layer_name The name fo the layer to compare voxel content.
/// @param tolerance A dummy @c MapLayer object which wraps the allowed tolerances for differences in voxel member
/// values. See @c compareVoxel().
/// @param log Logging function.
/// @return False if any validation step fails.
VoxelsResult ohm_API compareVoxels(const OccupancyMap &eval_map, const OccupancyMap &ref_map,
                                   const std::string &layer_name, const MapLayer *tolerance = nullptr,
                                   unsigned flags = 0, Log log = emptyLog);


/// Configure a data tolerance value for @c member_name. The allowed absolute error tolerance is @p epsilon.
///
/// See @c compareVoxel() for intended usage.
/// @param layer The tolerance object to configure.
/// @param member_name The data member name to configure tolerance for.
/// @param epsilon The allowed error limit (absolute value is used).
void ohm_API configureTolerance(ohm::MapLayer &layer, const char *member_name, int8_t epsilon);
/// @overload
void ohm_API configureTolerance(ohm::MapLayer &layer, const char *member_name, uint8_t epsilon);
/// @overload
void ohm_API configureTolerance(ohm::MapLayer &layer, const char *member_name, int16_t epsilon);
/// @overload
void ohm_API configureTolerance(ohm::MapLayer &layer, const char *member_name, uint16_t epsilon);
/// @overload
void ohm_API configureTolerance(ohm::MapLayer &layer, const char *member_name, int32_t epsilon);
/// @overload
void ohm_API configureTolerance(ohm::MapLayer &layer, const char *member_name, uint32_t epsilon);
/// @overload
void ohm_API configureTolerance(ohm::MapLayer &layer, const char *member_name, int64_t epsilon);
/// @overload
void ohm_API configureTolerance(ohm::MapLayer &layer, const char *member_name, uint64_t epsilon);
/// @overload
void ohm_API configureTolerance(ohm::MapLayer &layer, const char *member_name, float epsilon);
/// @overload
void ohm_API configureTolerance(ohm::MapLayer &layer, const char *member_name, double epsilon);
}  // namespace compare
}  // namespace ohm

#endif  // OHM_COMPARE_H
