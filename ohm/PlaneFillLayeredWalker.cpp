// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PlaneFillLayeredWalker.h"

#include "HeightmapUtil.h"
#include "Key.h"
#include "OccupancyMap.h"

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
  opened_grid_.clear();
  visited_grid_.clear();
  opended_list_.clear();

  if (range.isValid())
  {
    // Size the 2D grid (fixed size)
    opened_grid_.resize(size_t(key_range[axis_indices[0]]) * size_t(key_range[axis_indices[1]]));
    visited_grid_.resize(size_t(key_range[axis_indices[0]]) * size_t(key_range[axis_indices[1]]));
    // Reserve the open_list_ with double the initial capacity of the grid.
    opended_list_.reserve(2 * opened_grid_.size());

    // Clear the grid.
    std::fill(opened_grid_.begin(), opened_grid_.end(), 0u);
    std::fill(visited_grid_.begin(), visited_grid_.end(), 0u);

    // Ensure the key is in range.
    key.clampTo(range.minKey(), range.maxKey());

    // Mark the column as visited.
    const unsigned grid_index = gridIndexForKey(key);
    if (grid_index < visited_grid_.size())
    {
      visited_grid_[grid_index] = 1u;
    }

    return true;
  }

  return false;
}


bool PlaneFillLayeredWalker::walkNext(Key &key, bool &is_first_column_visit)
{
  // Pop the open list.
  if (!open_list_.empty())
  {
    key = open_list_.front();
    open_list_.pop_front();

    const unsigned grid_index = gridIndexForKey(key);
    if (grid_index < visited_grid_.size())
    {
      // Check if this is the first visit to the column.
      is_first_column_visit = (visited_grid_[grid_index] != 0);
      // Mark the column as visited.
      visited_grid_[grid_index] = 1u;
    }
    else
    {
      // Technically an error.
      is_first_column_visit = false;
    }
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
          if (mode == PlaneWalkVisitMode::kAddUnvisitedNeighbours && !hasOpened(idx, n_visit_height) ||
              mode == PlaneWalkVisitMode::kAddUnvisitedColumnNeighbours && !hasOpened(idx))
          {
            // Neighbour in range and not touched. Add to open list.
            open(idx, n_visit_height);
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


void PlaneFillLayeredWalker::open(int grid_index, int visit_height)
{
  opended_list_.emplace_back();
  Opened &new_open = opended_list_.back();
  new_open.height = visit_height;
  new_open.next = opened_grid_[grid_index];
  // Note: it is correct to use the opended_list_.size() to find the index of the new item as we are using a 1-based
  // index.
  opened_grid_[grid_index] = unsigned(opended_list_.size());
}


bool PlaneFillLayeredWalker::hasOpened(int grid_index, int visit_height) const
{
  // Traverse the linked list of items for this grid index.
  // Get the item head.
  unsigned current = opened_grid_[grid_index];
  // current is a 1-based index in to opended_list_. Zero is a terminating value.
  while (current > 0)
  {
    const Opened &opened = opended_list_[current - 1];
    if (opened.height == visit_height)
    {
      return true;
    }
    current = opened.next;
  }

  return false;
}
}  // namespace ohm
