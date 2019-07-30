// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_MAPPROCESS_H
#define OHM_MAPPROCESS_H

#include "OhmConfig.h"

namespace ohm
{
  class OccupancyMap;

  /// Return value for @c Mapper::update() and @c MappingProcess::update(). Values must be ordered
  /// such that more significant results come later. This supports a simple amalgomation of results
  /// in @c Mappper::update() by simply using the maximum value of the aggregate processes.
  enum MappingProcessResult
  {
    /// Everything up to date or nothing to do.
    kMprUpToDate,
    /// Some progress made. More to be done.
    kMprProgressing
  };

  /// Base class for processes to be added to the @p Mapper for processing during map update.
  ///
  /// @todo Features to consider:
  /// - Update period and scheduling
  /// - Update only on dirty?
  class ohm_API MappingProcess
  {
  public:
    MappingProcess();
    virtual ~MappingProcess();

    inline void pause(bool pause = true) { paused_ = pause; }
    inline bool paused() const { return paused_; }

    virtual void reset() = 0;

    virtual int update(OccupancyMap &map, double time_slice) = 0;  // NOLINT(google-runtime-references)

  private:
    bool paused_;
  };
}  // namespace ohm

#endif  // OHM_MAPPROCESS_H
