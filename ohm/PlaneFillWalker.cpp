// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PlaneFillWalker.h"

#include "HeightmapUtil.h"
#include "OccupancyMap.h"

#include <algorithm>

namespace ohm
{
PlaneFillWalker::PlaneFillWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key,
                                 UpAxis up_axis, Revisit revisit_behaviour)
  : map(map)
  , min_ext_key(min_ext_key)
  , max_ext_key(max_ext_key)
  , key_range(map.rangeBetween(min_ext_key, max_ext_key) + glm::ivec3(1, 1, 1))
  , axis_indices(ohm::heightmap::heightmapAxisIndices(up_axis))
  , revisit_behaviour(revisit_behaviour)
{}


bool PlaneFillWalker::begin(Key &key)
{
  open_list_.clear();
  visit_grid_.clear();
  visit_grid_.resize(size_t(key_range[axis_indices[0]]) * size_t(key_range[axis_indices[1]]));

  if (visit_grid_.empty())
  {
    // Key out of range.
    return false;
  }

  key.clampTo(min_ext_key, max_ext_key);
  return true;
}


bool PlaneFillWalker::walkNext(Key &key)
{
  // Pop the open list.
  while (!open_list_.empty())
  {
    key = open_list_.front();
    open_list_.pop_front();

    if (glm::all(glm::greaterThan(key_range, glm::ivec3(0))))
    {
      key.clampTo(min_ext_key, max_ext_key);
      const unsigned grid_index = gridIndex(key);
      visit_grid_[grid_index].visit(keyHeight(key));
      return true;
    }
  }

  return false;
}


size_t PlaneFillWalker::visit(const Key &key, PlaneWalkVisitMode mode, std::array<Key, 8> &added_neighbours)
{
  size_t added = 0;

  if (mode != PlaneWalkVisitMode::kIgnoreNeighbours)
  {
    unsigned grid_index = gridIndex(key);
    if (grid_index != ~0u)
    {
      // Note: we do not update the visit height for key. This can result in recurring loops. We only want to track the
      // best height at which a voxel was added to the open list.
      for (int row_delta = -1; row_delta <= 1; ++row_delta)
      {
        for (int col_delta = -1; col_delta <= 1; ++col_delta)
        {
          Key n_key = key;
          map.moveKeyAlongAxis(n_key, axis_indices[1], row_delta);
          map.moveKeyAlongAxis(n_key, axis_indices[0], col_delta);

          grid_index = gridIndex(n_key);
          if (grid_index != ~0u)
          {
            bool add_to_open = false;
            const int n_visit_height = keyHeight(n_key);
            switch (revisit_behaviour)
            {
            case Revisit::kHigher:
              add_to_open = visit_grid_[grid_index].revisitHigher(n_visit_height);
              break;

            case Revisit::kLower:
              add_to_open = visit_grid_[grid_index].revisitLower(n_visit_height);
              break;

            case Revisit::kNone:
            default:
              add_to_open = visit_grid_[grid_index].revisitNone(n_visit_height);
              break;
            }

            if (add_to_open)
            {
              // Neighbour in range and not touched. Add to open list.
              open_list_.push_back(n_key);
              visit_grid_[grid_index].visit(n_visit_height);
              added_neighbours[added] = n_key;
              ++added;
            }
          }
        }
      }
    }
  }

  return added;
}


void PlaneFillWalker::touch(const Key &key)
{
  const auto idx = gridIndex(key);
  if (idx != ~0u)
  {
    visit_grid_[idx].visit(keyHeight(key));
  }
}


unsigned PlaneFillWalker::gridIndex(const Key &key)
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


int PlaneFillWalker::keyHeight(const Key &key) const
{
  return map.rangeBetween(min_ext_key, key)[axis_indices[2]];
}
}  // namespace ohm
