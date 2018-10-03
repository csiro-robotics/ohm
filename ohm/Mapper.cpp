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

using namespace ohm;

Mapper::Mapper(OccupancyMap *map)
  : imp_(new MapperDetail)
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
    delete imp_;
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
    double elpased_sec = 0;

    // Ensure first item is in range.
    imp_->nextProcess = imp_->nextProcess % unsigned(imp_->processes.size());
    const unsigned initial_index = imp_->nextProcess;
    bool first_iteration = true;

    // Update until we consume the timeSliceSec or everything is up to date.
    while ((time_slice_sec == 0 || elpased_sec < time_slice_sec) && (first_iteration || imp_->nextProcess != initial_index))
    {
      // Enforce range of imp_->nextProcess. This plus the increment ensures we run processes in a round robin,
      // but don't go out of range.
      MappingProcess *process = imp_->processes[imp_->nextProcess++];
      imp_->nextProcess = imp_->nextProcess % unsigned(imp_->processes.size());
      if (!process->paused())
      {
        process_result = process->update(*map, time_slice_sec);
        status = std::max(process_result, status);
      }
      cur_time = Clock::now();
      elpased_sec = std::chrono::duration_cast<std::chrono::duration<double>>(cur_time - start_time).count();
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


unsigned Mapper::indexOfProcess(const MappingProcess* process) const
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
