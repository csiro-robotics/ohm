// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "FloodFillLayeredWalker.h"

#include "ohm/Key.h"
#include "ohm/OccupancyMap.h"

#include <algorithm>

namespace ohm
{
FloodFillLayeredWalker::FloodFillLayeredWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key,
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


bool FloodFillLayeredWalker::begin(Key &key)
{
  while (!open_list.empty())
  {
    open_list.pop();
  }
  visit_grid.clear();
  visit_list.clear();

  visit_grid.resize(size_t(key_range[axis_indices[0]]) * size_t(key_range[axis_indices[1]]));
  visit_list.reserve(visit_grid.size());

  if (visit_grid.empty())
  {
    // Key out of range.
    return false;
  }

  std::fill(visit_grid.begin(), visit_grid.end(), 0u);

  key.clampTo(min_ext_key, max_ext_key);

  // Need to visit the key voxel.
  const auto idx = touchIndex(key);
  const int visit_height = visitHeight(key);
  visit(idx, visit_height);
  if (auto_add_neighbours)
  {
    addNeighbours(key);
  }

  return true;
}


bool FloodFillLayeredWalker::walkNext(Key &key)
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
      // const int n_visit_height = visitHeight(key);
      // if (visit_list[touch_index].isAt(n_visit_height))
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


size_t FloodFillLayeredWalker::addNeighbours(const Key &key, std::array<Key, 8> &added_neighbours,
                                             Revisit revisit_behaviour)
{
  size_t added = 0;
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
          add_to_open = revisitAll(idx, n_visit_height);
          break;

        case Revisit::kHigher:
          add_to_open = revisitHigher(idx, n_visit_height);
          break;

        case Revisit::kLower:
          add_to_open = revisitLower(idx, n_visit_height);
          break;

        case Revisit::kNone:
        default:
          add_to_open = revisitNone(idx);
          break;
        }

        if (add_to_open)
        {
          // Neighbour in range and not touched. Add to open list.
          open_list.push(n_key);
          visit(idx, n_visit_height);
          visit_list[idx].visit(n_visit_height);
          added_neighbours[added] = n_key;
          ++added;
        }
      }
    }
  }

  return added;
}


void FloodFillLayeredWalker::touch(const Key &key)
{
  const auto idx = touchIndex(key);
  if (idx != ~0u)
  {
    visit_list[idx].visit(visitHeight(key));
  }
}


unsigned FloodFillLayeredWalker::touchIndex(const Key &key)
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


int FloodFillLayeredWalker::visitHeight(const Key &key) const
{
  return map.rangeBetween(min_ext_key, key)[axis_indices[2]];
}


void FloodFillLayeredWalker::visit(int grid_index, int visit_height)
{
  visit_list.emplace_back();
  Visit &new_visit = visit_list.back();
  new_visit.height = visit_height;
  new_visit.next = visit_grid[grid_index];
  // Note: it is correct to use the visit_list.size() to find the index of the new item as we are using a 1-based index.
  visit_grid[grid_index] = unsigned(visit_list.size());
}


bool FloodFillLayeredWalker::revisitAll(int grid_index, int visit_height) const
{
  unsigned next_index = visit_grid[grid_index];
  while (next_index)
  {
    // Convert from 1-based index to zero-based index.
    --next_index;
    const Visit visit = visit_list[next_index];
    if (visit.height == visit_height)
    {
      // Already visited at this height.
      return false;
    }
    next_index = visit.next;
  }
  return true;
}

bool FloodFillLayeredWalker::revisitHigher(int grid_index, int visit_height) const
{
  unsigned next_index = visit_grid[grid_index];
  while (next_index)
  {
    // Convert from 1-based index to zero-based index.
    --next_index;
    const Visit visit = visit_list[next_index];
    if (visit.height >= visit_height)
    {
      // Already visited at this height or above.
      return false;
    }
    next_index = visit.next;
  }
  return true;
}

bool FloodFillLayeredWalker::revisitLower(int grid_index, int visit_height) const
{
  unsigned next_index = visit_grid[grid_index];
  while (next_index)
  {
    // Convert from 1-based index to zero-based index.
    --next_index;
    const Visit visit = visit_list[next_index];
    if (visit.height <= visit_height)
    {
      // Already visited at this height or below.
      return false;
    }
    next_index = visit.next;
  }
  return true;
}
}  // namespace ohm
