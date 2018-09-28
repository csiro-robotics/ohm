// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMMAPPROCESS_H_
#define OHMMAPPROCESS_H_

#include "ohmconfig.h"

namespace ohm
{
  class OccupancyMap;

  /// Return value for @c Mapper::update() and @c MappingProcess::update(). Values must be ordered
  /// such that more significant results come later. This supports a simple amalgomation of results
  /// in @c Mappper::update() by simply using the maximum value of the aggregate processes.
  enum MappingProcessResult
  {
    /// Everything up to date or nothing to do.
    MPR_UpToDate,
    /// Some progress made. More to be done.
    MPR_Progressing
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

    inline void pause(bool pause = true) { _paused = pause; }
    inline bool paused() const { return _paused; }

    virtual void reset() = 0;

    virtual int update(OccupancyMap &map, double timeSlice) = 0;

  private:
    bool _paused;
  };
}

#endif // OHMMAPPROCESS_H_
