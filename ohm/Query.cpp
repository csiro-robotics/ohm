// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Query.h"

#include "private/QueryDetail.h"

#include "QueryFlag.h"

namespace ohm
{
Query::Query(QueryDetail *detail)
  : imp_(detail)
{
  if (!imp_)
  {
    imp_ = new QueryDetail;
  }
}


Query::~Query()
{
  delete imp_;
  imp_ = nullptr;
}


const OccupancyMap *Query::map() const
{
  return imp_->map;
}


void Query::setMap(OccupancyMap *map)
{
  imp_->map = map;
  onSetMap();
}


unsigned Query::queryFlags() const
{
  return imp_->query_flags;
}


void Query::setQueryFlags(unsigned flags)
{
  imp_->query_flags = flags;
}


size_t Query::numberOfResults() const
{
  return imp_->number_of_results;
}


const Key *Query::intersectedVoxels() const
{
  return (!imp_->intersected_voxels.empty()) ? imp_->intersected_voxels.data() : nullptr;
}


const double *Query::ranges() const
{
  return (!imp_->ranges.empty()) ? imp_->ranges.data() : nullptr;
}


bool Query::execute()
{
  reset(false);
  return onExecute();
}


bool Query::executeAsync()
{
  return onExecuteAsync();
}


void Query::reset(bool hard_reset)
{
  wait();
  imp_->intersected_voxels.clear();
  imp_->ranges.clear();
  imp_->number_of_results = 0u;
  onReset(hard_reset);
}


bool Query::wait(unsigned timeout_ms)
{
  return onWaitAsync(timeout_ms);
}


bool Query::onWaitAsync(unsigned /*timeout_ms*/)
{
  return false;
}
}  // namespace ohm
