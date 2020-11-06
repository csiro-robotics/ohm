// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Mapper.h"

#include "MappingProcess.h"

#include "private/MapperDetail.h"

#include <algorithm>
#include <chrono>

namespace ohm
{
Mapper::Mapper(OccupancyMap *map)
  : imp_(std::make_unique<MapperDetail>())
{
  setMap(map);
}


Mapper::~Mapper()
{
  if (imp_)
  {
    for (MappingProcess *process : imp_->processes)
    {
      delete process;
    }
  }
}

void Mapper::setMap(OccupancyMap *map)
{
  imp_->map = map;
}


OccupancyMap *Mapper::map() const
{
  return imp_->map;
}


int Mapper::update(double time_slice_sec)
{
  using Clock = std::chrono::high_resolution_clock;
  const Clock::time_point start_time = Clock::now();
  Clock::time_point cur_time;

  OccupancyMap *map = this->map();

  int status = kMprUpToDate;
  if (map && !imp_->processes.empty())
  {
    int process_result;
    double elapsed_sec = 0;

    // Ensure first item is in range.
    imp_->next_process = imp_->next_process % unsigned(imp_->processes.size());
    const unsigned initial_index = imp_->next_process;
    bool first_iteration = true;

    // Update until we consume the timeSliceSec or everything is up to date.
    while ((time_slice_sec == 0 || elapsed_sec < time_slice_sec) &&
           (first_iteration || imp_->next_process != initial_index))
    {
      // Enforce range of imp_->next_process. This plus the increment ensures we run processes in a round robin,
      // but don't go out of range.
      MappingProcess *process = imp_->processes[imp_->next_process++];
      imp_->next_process = imp_->next_process % unsigned(imp_->processes.size());
      if (!process->paused())
      {
        process_result = process->update(*map, time_slice_sec);
        status = std::max(process_result, status);
      }
      cur_time = Clock::now();
      elapsed_sec = std::chrono::duration_cast<std::chrono::duration<double>>(cur_time - start_time).count();
      first_iteration = false;
    }
  }

  return status;
}


void Mapper::addProcess(MappingProcess *process)
{
  // ASSERT(!hasProcess(process));
  // ASSERT(process);
  imp_->processes.push_back(process);
}


bool Mapper::removeProcess(const MappingProcess *process)
{
  auto search = std::find(imp_->processes.begin(), imp_->processes.end(), process);
  if (search != imp_->processes.end())
  {
    imp_->processes.erase(search);
    return true;
  }

  return false;
}


bool Mapper::hasProcess(const MappingProcess *process) const
{
  return indexOfProcess(process) != npos;
}


unsigned Mapper::indexOfProcess(const MappingProcess *process) const
{
  if (imp_)
  {
    const auto iter = std::find(imp_->processes.begin(), imp_->processes.end(), process);
    if (iter != imp_->processes.end())
    {
      return static_cast<unsigned>(iter - imp_->processes.begin());
    }
  }

  return npos;
}


unsigned Mapper::processCount() const
{
  return unsigned(imp_->processes.size());
}


MappingProcess *Mapper::process(unsigned index)
{
  return imp_->processes[index];
}


const MappingProcess *Mapper::process(unsigned index) const
{
  return imp_->processes[index];
}
}  // namespace ohm
