// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "LineKeysQuery.h"

#include "private/LineKeysQueryDetail.h"

#include "CalculateSegmentKeys.h"
#include "KeyList.h"
#include "OccupancyMap.h"
#include "OccupancyUtil.h"

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

namespace ohm
{
namespace
{
unsigned nextPow2(unsigned v)
{
  // compute the next highest power of 2 of 32-bit v
  v--;
  v |= v >> 1u;
  v |= v >> 2u;
  v |= v >> 4u;
  v |= v >> 8u;
  v |= v >> 16u;  // NOLINT(readability-magic-numbers)
  v++;
  return v;
}
}  // namespace

LineKeysQuery::LineKeysQuery(LineKeysQueryDetail *detail)
  : Query(detail)
{}


LineKeysQuery::LineKeysQuery(ohm::OccupancyMap &map, unsigned query_flags)
  : LineKeysQuery(query_flags)
{
  setMap(&map);
}


LineKeysQuery::LineKeysQuery(unsigned query_flags)
  : LineKeysQuery(new LineKeysQueryDetail)
{
  setQueryFlags(query_flags);
}


LineKeysQuery::~LineKeysQuery()
{
  auto *d = static_cast<LineKeysQueryDetail *>(imp_);
  delete d;
  imp_ = nullptr;
}


void LineKeysQuery::setRays(const glm::dvec3 *rays, size_t point_count)
{
  auto *d = static_cast<LineKeysQueryDetail *>(imp_);
  d->rays.resize(point_count);
  memcpy(d->rays.data(), rays, sizeof(*rays) * point_count);
}


const glm::dvec3 *LineKeysQuery::rays() const
{
  const auto *d = static_cast<LineKeysQueryDetail *>(imp_);
  return d->rays.data();
}


size_t LineKeysQuery::rayPointCount() const
{
  const auto *d = static_cast<LineKeysQueryDetail *>(imp_);
  return d->rays.size();
}


const size_t *LineKeysQuery::resultIndices() const
{
  auto *d = static_cast<LineKeysQueryDetail *>(imp_);
  return d->result_indices.data();
}


const size_t *LineKeysQuery::resultCounts() const
{
  auto *d = static_cast<LineKeysQueryDetail *>(imp_);
  return d->result_counts.data();
}


bool LineKeysQuery::onExecute()
{
  auto *d = static_cast<LineKeysQueryDetail *>(imp_);

  KeyList key_list;
  d->result_indices.resize(d->rays.size() / 2);
  d->result_counts.resize(d->rays.size() / 2);
  for (size_t i = 0; i < d->rays.size(); i += 2)
  {
    key_list.clear();
    calculateSegmentKeys(key_list, *d->map, d->rays[i + 0], d->rays[i + 1], true);
    d->result_indices[i / 2] = d->intersected_voxels.size();
    d->result_counts[i / 2] = key_list.size();
    for (auto &&key : key_list)
    {
      d->intersected_voxels.push_back(key);
    }
  }

  d->number_of_results = d->result_indices.size();

  return true;
}


bool LineKeysQuery::onExecuteAsync()
{
  auto *d = static_cast<LineKeysQueryDetail *>(imp_);

  if ((d->query_flags & kQfGpuEvaluate))
  {
    static bool once = false;
    if (!once)
    {
      once = true;
      std::cerr << "GPU unavailable for LineKeysQuery. Failing async call.\n" << std::flush;
    }
  }

  return false;
}


void LineKeysQuery::onReset(bool /*hard_reset*/)
{
  auto *d = static_cast<LineKeysQueryDetail *>(imp_);
  d->result_indices.clear();
  d->result_counts.clear();
}


LineKeysQueryDetail *LineKeysQuery::imp()
{
  return static_cast<LineKeysQueryDetail *>(imp_);
}


const LineKeysQueryDetail *LineKeysQuery::imp() const
{
  return static_cast<const LineKeysQueryDetail *>(imp_);
}
}  // namespace ohm
