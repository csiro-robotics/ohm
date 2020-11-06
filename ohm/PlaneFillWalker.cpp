// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PlaneFillWalker.h"

#include "ohm/Key.h"
#include "ohm/OccupancyMap.h"

#include <algorithm>

namespace ohm
{
PlaneFillWalker::PlaneFillWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key,
                                 UpAxis up_axis, bool auto_add_neighbours)
  : map(map)
  , min_ext_key(min_ext_key)
  , max_ext_key(max_ext_key)
  , key_range(map.rangeBetween(min_ext_key, max_ext_key) + glm::ivec3(1, 1, 1))
  , auto_add_neighbours(auto_add_neighbours)
{
  switch (up_axis)
  {
  case UpAxis::kX:
    /* fallthrough */
  case UpAxis::kNegX:
    axis_indices[0] = 1;
    axis_indices[1] = 2;
    axis_indices[2] = 0;
    break;
  case UpAxis::kY:
    /* fallthrough */
  case UpAxis::kNegY:
    axis_indices[0] = 0;
    axis_indices[1] = 2;
    axis_indices[2] = 1;
    break;
  case UpAxis::kZ:
    /* fallthrough */
  case UpAxis::kNegZ:
    axis_indices[0] = 0;
    axis_indices[1] = 1;
    axis_indices[2] = 2;
    break;
  }
}


bool PlaneFillWalker::begin(Key &key)
{
  while (!open_list.empty())
  {
    open_list.pop();
  }
  visit_list.clear();

  visit_list.resize(size_t(key_range[axis_indices[0]]) * size_t(key_range[axis_indices[1]]));

  if (visit_list.empty())
  {
    // Key out of range.
    return false;
  }

  for (auto &touched : visit_list)
  {
    touched = -1;
  }

  key.clampTo(min_ext_key, max_ext_key);
  touch(key);
  if (auto_add_neighbours)
  {
    addNeighbours(key);
  }

  return true;
}


bool PlaneFillWalker::walkNext(Key &key)
{
  // Pop the open list.
  while (!open_list.empty())
  {
    key = open_list.front();
    open_list.pop();

    // Validate the key is still the best available and hasn't been superceded by a Revisit rule.
    const unsigned touch_index = touchIndex(key);

    if (touch_index != ~0u)
    {
      // Check for multiple open pushes.
      const int n_visit_height = visitHeight(key);
      if (n_visit_height == visit_list[touch_index])
      {
        if (auto_add_neighbours)
        {
          addNeighbours(key);
        }

        return true;
      }
    }
  }

  return false;
}


void PlaneFillWalker::addNeighbours(const Key &key, Revisit revisit_behaviour)
{
  for (int row_delta = -1; row_delta <= 1; ++row_delta)
  {
    for (int col_delta = -1; col_delta <= 1; ++col_delta)
    {
      Key n_key = key;
      map.moveKeyAlongAxis(n_key, axis_indices[1], row_delta);
      map.moveKeyAlongAxis(n_key, axis_indices[0], col_delta);

      const auto idx = touchIndex(n_key);
      if (idx != ~0u)
      {
        bool add_to_open = false;
        const int n_visit_height = visitHeight(n_key);
        switch (revisit_behaviour)
        {
        case Revisit::kAll:
          add_to_open = true;
          break;

        case Revisit::kHigher:
          add_to_open = visit_list[idx] < 0 || n_visit_height > visit_list[idx];
          break;

        case Revisit::kLower:
          add_to_open = visit_list[idx] < 0 || n_visit_height < visit_list[idx];
          break;

        case Revisit::kNone:
        default:
          add_to_open = visit_list[idx] < 0;
          break;
        }

        if (add_to_open)
        {
          // Neighbour in range and not touched. Add to open list.
          open_list.push(n_key);
          visit_list[idx] = n_visit_height;
        }
      }
    }
  }
}


void PlaneFillWalker::touch(const Key &key)
{
  const auto idx = touchIndex(key);
  if (idx != ~0u)
  {
    visit_list[idx] = visitHeight(key);
  }
}


unsigned PlaneFillWalker::touchIndex(const Key &key)
{
  // Get the offset for the key.
  const auto offset_to_key = map.rangeBetween(min_ext_key, key);

  if (offset_to_key[axis_indices[0]] >= 0 && offset_to_key[axis_indices[1]] >= 0 &&
      offset_to_key[axis_indices[0]] < key_range[axis_indices[0]] &&
      offset_to_key[axis_indices[1]] < key_range[axis_indices[1]])
  {
    return unsigned(offset_to_key[axis_indices[0]]) +
           unsigned(offset_to_key[axis_indices[1]]) * unsigned(key_range[axis_indices[0]]);
  }

  // Key out of range.
  return ~0u;
}


int PlaneFillWalker::visitHeight(const Key &key) const
{
  return map.rangeBetween(min_ext_key, key)[axis_indices[2]];
}
}  // namespace ohm
