// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancyquery.h"

#include "private/occupancyquerydetail.h"

#include "occupancyqueryflag.h"

using namespace ohm;

Query::Query(QueryDetail *detail)
: _imp(detail)
{
  if (!_imp)
  {
    _imp = new QueryDetail;
  }
}


Query::~Query()
{
  delete _imp;
  _imp = nullptr;
}


const OccupancyMap *Query::map() const
{
  return _imp->map;
}


void Query::setMap(OccupancyMap *map)
{
  _imp->map = map;
  onSetMap();
}


unsigned Query::queryFlags() const
{
  return _imp->queryFlags;
}


void Query::setQueryFlags(unsigned flags)
{
  _imp->queryFlags = flags;
}


size_t Query::numberOfResults() const
{
  return _imp->numberOfResults;
}


const OccupancyKey *Query::intersectedVoxels() const
{
  return (!_imp->intersectedVoxels.empty()) ? _imp->intersectedVoxels.data() : nullptr;
}


const float *Query::ranges() const
{
  return (!_imp->ranges.empty()) ? _imp->ranges.data() : nullptr;
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


void Query::reset(bool hardReset)
{
  wait();
  _imp->intersectedVoxels.clear();
  _imp->ranges.clear();
  _imp->numberOfResults = 0u;
  onReset(hardReset);
}


bool Query::wait(unsigned timeoutMs)
{
  return onWaitAsync(timeoutMs);
}


bool Query::onWaitAsync(unsigned timeoutMs)
{
  return false;
}
