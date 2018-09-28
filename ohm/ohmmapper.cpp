// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmmapper.h"

#include "ohmmappingprocess.h"

#include "private/mapperdetail.h"

#include <algorithm>
#include <chrono>

using namespace ohm;

Mapper::Mapper(OccupancyMap *map)
  : _detail(new MapperDetail)
{
  setMap(map);
}


Mapper::~Mapper()
{
  if (_detail)
  {
    for (MappingProcess *process : _detail->processes)
    {
      delete process;
    }
    delete _detail;
  }
}

void Mapper::setMap(OccupancyMap *map)
{
  _detail->map = map;
}


OccupancyMap *Mapper::map() const
{
  return _detail->map;
}


int Mapper::update(double timeSliceSec)
{
  using Clock = std::chrono::high_resolution_clock;
  Clock::time_point startTime = Clock::now();
  Clock::time_point curTime;

  OccupancyMap *map = this->map();

  int status = MPR_UpToDate;
  if (map && !_detail->processes.empty())
  {
    int processResult;
    double elpasedSec = 0;

    // Ensure first item is in range.
    _detail->nextProcess = _detail->nextProcess % unsigned(_detail->processes.size());
    const unsigned initialIndex = _detail->nextProcess;
    bool firstIteration = true;

    // Update until we consume the timeSliceSec or everything is up to date.
    while ((timeSliceSec == 0 || elpasedSec < timeSliceSec) && (firstIteration || _detail->nextProcess != initialIndex))
    {
      // Enforce range of _detail->nextProcess. This plus the increment ensures we run processes in a round robin,
      // but don't go out of range.
      MappingProcess *process = _detail->processes[_detail->nextProcess++];
      _detail->nextProcess = _detail->nextProcess % unsigned(_detail->processes.size());
      if (!process->paused())
      {
        processResult = process->update(*map, timeSliceSec);
        status = std::max(processResult, status);
      }
      curTime = Clock::now();
      elpasedSec = std::chrono::duration_cast<std::chrono::duration<double>>(curTime - startTime).count();
      firstIteration = false;
    }
  }

  return status;
}


void Mapper::addProcess(MappingProcess *process)
{
  // ASSERT(!hasProcess(process));
  // ASSERT(process);
  _detail->processes.push_back(process);
}


bool Mapper::removeProcess(const MappingProcess *process)
{
  auto search = std::find(_detail->processes.begin(), _detail->processes.end(), process);
  if (search != _detail->processes.end())
  {
    _detail->processes.erase(search);
    return true;
  }

  return false;
}


bool Mapper::hasProcess(const MappingProcess *process) const
{
  return indexOfProcess(process) != npos;
}


unsigned Mapper::indexOfProcess(const MappingProcess* process) const
{
  if (_detail)
  {
    const auto iter = std::find(_detail->processes.begin(), _detail->processes.end(), process);
    if (iter != _detail->processes.end())
    {
      return static_cast<unsigned>(iter - _detail->processes.begin());
    }
  }

  return npos;
}


unsigned Mapper::processCount() const
{
  return unsigned(_detail->processes.size());
}


MappingProcess *Mapper::process(unsigned index)
{
  return _detail->processes[index];
}


const MappingProcess *Mapper::process(unsigned index) const
{
  return _detail->processes[index];
}
