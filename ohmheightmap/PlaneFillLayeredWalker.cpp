// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PlaneFillLayeredWalker.h"

#include "HeightmapUtil.h"

#include <ohm/Key.h>
#include <ohm/OccupancyMap.h>

#include <algorithm>

namespace ohm
{
PlaneFillLayeredWalker::PlaneFillLayeredWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key,
                                               UpAxis up_axis)
  : map(map)
  , range(KeyRange(min_ext_key, max_ext_key, map))
  , key_range(map.rangeBetween(min_ext_key, max_ext_key) + glm::ivec3(1, 1, 1))
  , axis_indices(ohm::heightmap::heightmapAxisIndices(up_axis))
  , up_sign((int(up_axis) >= 0) ? 1 : -1)
{}


bool PlaneFillLayeredWalker::begin(Key &key)
{
  // Clear existing data.
  open_list_.clear();
  touched_grid_.clear();
  touched_list_.clear();

  if (range.isValid())
  {
    // Size the 2D grid (fixed size)
    touched_grid_.resize(size_t(key_range[axis_indices[0]]) * size_t(key_range[axis_indices[1]]));
    // Reserve the open_list_ with double the initial capacity of the grid.
    touched_list_.reserve(2 * touched_grid_.size());

    // Clear the grid.
    std::fill(touched_grid_.begin(), touched_grid_.end(), 0u);

    // Ensure the key is in range.
    key.clampTo(range.minKey(), range.maxKey());

    return true;
  }

  return false;
}


bool PlaneFillLayeredWalker::walkNext(Key &key)
{
  // Pop the open list.
  if (!open_list_.empty())
  {
    key = open_list_.front();
    open_list_.pop_front();
    return true;
  }

  return false;
}


size_t PlaneFillLayeredWalker::visit(const Key &key, PlaneWalkVisitMode mode, std::array<Key, 8> &added_neighbours)
{
  size_t added = 0;
  if (mode != PlaneWalkVisitMode::kIgnoreNeighbours)
  {
    Key start_key = key;
    for (int row_delta = -1; row_delta <= 1; ++row_delta)
    {
      for (int col_delta = -1; col_delta <= 1; ++col_delta)
      {
        Key n_key = start_key;
        map.moveKeyAlongAxis(n_key, axis_indices[1], row_delta);
        map.moveKeyAlongAxis(n_key, axis_indices[0], col_delta);

        const auto idx = gridIndexForKey(n_key);
        if (idx != ~0u && (row_delta != 0 || col_delta != 0))
        {
          const int n_visit_height = keyHeight(n_key);
          if (mode == PlaneWalkVisitMode::kAddUnvisitedNeighbours && !hasTouched(idx, n_visit_height) ||
              mode == PlaneWalkVisitMode::kAddUnvisitedColumnNeighbours && !hasTouched(idx))
          {
            // Neighbour in range and not touched. Add to touched and open lists.
            touch(idx, n_visit_height);
            open_list_.push_back(n_key);
            assert(added < added_neighbours.size());
            added_neighbours[added] = n_key;
            ++added;
          }
        }
      }
    }
  }

  return added;
}


unsigned PlaneFillLayeredWalker::gridIndexForKey(const Key &key)
{
  // Get the offset for the key.
  const auto offset_to_key = map.rangeBetween(range.minKey(), key);

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


int PlaneFillLayeredWalker::keyHeight(const Key &key) const
{
  return map.rangeBetween(range.minKey(), key)[axis_indices[2]];
}


void PlaneFillLayeredWalker::touch(int grid_index, int visit_height)
{
  touched_list_.emplace_back();
  Touched &new_touch = touched_list_.back();
  new_touch.height = visit_height;
  new_touch.next = touched_grid_[grid_index];
  // Note: it is correct to use the touched_list_.size() to find the index of the new item as we are using a 1-based
  // index.
  touched_grid_[grid_index] = unsigned(touched_list_.size());
}


bool PlaneFillLayeredWalker::hasTouched(int grid_index, int visit_height) const
{
  // Traverse the linked list of items for this grid index.
  // Get the item head.
  unsigned current = touched_grid_[grid_index];
  // current is a 1-based index in to touched_list_. Zero is a terminating value.
  while (current > 0)
  {
    const Touched &touched = touched_list_[current - 1];
    if (touched.height == visit_height)
    {
      return true;
    }
    current = touched.next;
  }

  return false;
}
}  // namespace ohm
