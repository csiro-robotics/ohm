// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_MAPPER_H
#define OHM_MAPPER_H

#include "OhmConfig.h"

#include <memory>

namespace ohm
{
class OccupancyMap;
struct MapperDetail;
class MappingProcess;

/// The @c Mapper class manages a set of @c MappingProcess objects to execute as the map changes.
///
/// The @c Mapper update is invoked interspersed with occupancy map generation in order to make
/// additional calculations supporting the occupancy map. For example, a @c Mapper instance may
/// contain a @c MappingProcess which updates map layers derived from the current occupancy layer.
/// More concretely, the @c ClearanceProcess uses the occupancy layer to generate a clearance layer
/// which specifies the range to the nearest obstacle for each voxel.
///
/// @c Mapper execution is designed to be limited to a specified time slice and each @c MappingProcess
/// should only process dirty map regions (see @c MappingProcess for more details). Calling of
/// @c MappingProcess updates is set to run each process round robin so no single process is starved,
/// until the allowed time slice is exceeded.
///
/// Intended usage is as follows:
/// - Instantiate an @c OccupancyMap for population.
/// - Instantiate an associated @c Mapper.
/// - Register a set of @c MappingProcess objects to run round robin with occupancy map generation.
/// - While new occupancy rays come in:
///   - Integrate new occupancy rays into the map.
///   - Update the @p Mapper, optionally with a time limit.
///
/// The @c Mapper is designed to be strongly associated with one @c OccupancyMap which does not change.
class ohm_API Mapper
{
public:
  /// Invalid return value for @c indexOfProcess()
  const unsigned npos = ~0u;

  /// Instantiate a mapper optionally providing the target @p map on construction.
  /// If @p map is null here then @c setMap() must be called before @c update().
  explicit Mapper(OccupancyMap *map = nullptr);

  /// Destructor deleting all the registered @c MappingProcesses.
  ~Mapper();

  /// Sets the map to operate on. Must be set either here or on construction before @c update().
  /// @param map The map to operate on.s
  void setMap(OccupancyMap *map);

  /// Gets the target map for the @p Mapper.
  /// @return The target @c OccupancyMap.
  OccupancyMap *map() const;

  /// Updates the @c Mapper giving each @c MappingProcess an opportunity to update.
  ///
  /// Each @c MappingProcess is updated in turn until the @c timeSliceSec is exceeded. This process tracks the last
  /// @c MappingProcess updated, so the next call to this @c update() will target the next @c MappingProcess in
  /// the list. This ensures each process has the same opportunity to run.
  ///
  /// It is not recommended that the set of @c MappingProcess objects change between calls, though the method is
  /// robust to such changes.
  ///
  /// A full update may be forced by setting @p timeSliceSec to zero.
  ///
  /// @param time_slice_sec The allowed time slice for this update (seconds). Zero for full update.
  /// @return The overall status. See @c MappingProcessResult.
  int update(double time_slice_sec);

  /// Adds @p process to the update list. The @c Mapper takes ownership of the pointer. The specific @p process
  /// must not already be registered in this or any other @c Mapper.
  ///
  /// @param process The new process to add.
  void addProcess(MappingProcess *process);

  /// Remove @p process from the update list. The @c Mapper relinquishes ownership of the pointer.
  ///
  /// @param process The process to remove.
  /// @return True if @p process was registered and has been removed.
  bool removeProcess(const MappingProcess *process);

  /// Checks whether @p process is in the update list or not.
  /// @param process The process to search for.
  /// @return True if @p process is being managed by this @c Mapper.
  bool hasProcess(const MappingProcess *process) const;

  /// Returns the index of @p process in the process list.
  /// @return The index of @p process or @c npos on failure.
  unsigned indexOfProcess(const MappingProcess *process) const;

  /// Queries the number of registered @p MappingProcess objects.
  /// @return The number of registered processes.
  unsigned processCount() const;

  /// Gets the @c MappingProcess at @p index.
  /// @param index The index of the process of interest. Must be in the range [0, @c processCount()).
  /// @return The registered process at @p index.
  MappingProcess *process(unsigned index);
  /// @overload
  const MappingProcess *process(unsigned index) const;

  /// Internal data access.
  /// @return Internal data.
  inline MapperDetail *detail() { return imp_.get(); }
  /// @overload
  inline const MapperDetail *detail() const { return imp_.get(); }

private:
  std::unique_ptr<MapperDetail> imp_;
};
}  // namespace ohm

#endif  // OHM_MAPPER_H
