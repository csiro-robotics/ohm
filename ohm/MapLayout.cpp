// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapLayout.h"

#include "DefaultLayer.h"
#include "MapChunk.h"

#include "private/MapLayoutDetail.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <list>
#include <string>

using namespace ohm;

namespace
{
  void filterLayers(MapLayoutDetail &imp, const std::vector<unsigned> &preserve_layers)
  {
    if (imp.layers.empty())
    {
      return;
    }

    unsigned effective_index = 0;
    for (unsigned i = 0; i < unsigned(imp.layers.size()); ++effective_index)
    {
      MapLayer *layer = imp.layers[i];
      bool preserve = false;
      for (unsigned preserve_index : preserve_layers)
      {
        if (preserve_index == effective_index)
        {
          preserve = true;
          break;
        }
      }

      if (!preserve)
      {
        delete layer;
        imp.layers.erase(imp.layers.begin() + i);
      }
      else
      {
        layer->setLayerIndex(i);
        ++i;
      }
    }
  }
}  // namespace


MapLayout::MapLayout()
  : imp_(new MapLayoutDetail)
{}


MapLayout::MapLayout(MapLayout &&other) noexcept
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


MapLayout::MapLayout(const MapLayout &other)
  : MapLayout()
{
  *this = other;
}


MapLayout::~MapLayout()
{
  clear();
  delete imp_;
}


void MapLayout::clear()
{
  if (imp_)
  {
    for (auto layer : imp_->layers)
    {
      delete layer;
    }
    imp_->layers.clear();
    imp_->using_sub_voxel_patterns = MapLayoutDetail::kSubUnknown;
  }
}


int MapLayout::occupancyLayer() const
{
  return imp_->occupancy_layer;
}


int MapLayout::clearanceLayer() const
{
  return imp_->clearance_layer;
}


bool MapLayout::hasSubVoxelPattern() const
{
  if (imp_->using_sub_voxel_patterns == MapLayoutDetail::kSubUnknown)
  {
    // Need to look up the layer.
    if (occupancyLayer() >= 0)
    {
      const MapLayer *layer = imp_->layers[occupancyLayer()];
      if (layer && layer->voxelByteSize() > 0)
      {
        imp_->using_sub_voxel_patterns =
          (layer->voxelByteSize() == sizeof(float)) ? MapLayoutDetail::kSubOff : MapLayoutDetail::kSubOn;
      }
    }
  }

  return imp_->using_sub_voxel_patterns == MapLayoutDetail::kSubOn;
}


void MapLayout::invalidateSubVoxelPatternState()
{
  imp_->using_sub_voxel_patterns = MapLayoutDetail::kSubUnknown;
}


void MapLayout::filterLayers(const std::initializer_list<const char *> &preserve_layers)
{
  if (imp_->layers.empty())
  {
    return;
  }

  // Remove layers.
  // Convert preserve list to std::string
  std::vector<unsigned> preserve_indices;
  for (const char *layer_name : preserve_layers)
  {
    if (const MapLayer *layer = this->layer(layer_name))
    {
      preserve_indices.push_back(layer->layerIndex());
    }
  }

  ::filterLayers(*imp_, preserve_indices);
}


void MapLayout::filterLayers(const std::initializer_list<unsigned> &preserve_layers)
{
  ::filterLayers(*imp_, preserve_layers);
}


MapLayer *MapLayout::addLayer(const char *name, unsigned short subsampling)
{
  MapLayer *new_layer = new MapLayer(name, static_cast<unsigned short>(imp_->layers.size()), subsampling);
  imp_->layers.push_back(new_layer);

  std::string name_str(name);
  if (imp_->occupancy_layer == -1 && name_str.compare(default_layer::occupancyLayerName()) == 0)
  {
    imp_->occupancy_layer = new_layer->layerIndex();
  }
  else if (imp_->clearance_layer == -1 && name_str.compare(default_layer::clearanceLayerName()) == 0)
  {
    imp_->clearance_layer = new_layer->layerIndex();
  }

  return new_layer;
}


const MapLayer *MapLayout::layer(const char *layer_name) const
{
  const std::string name(layer_name);
  for (const MapLayer *layer : imp_->layers)
  {
    if (layer && name.compare(layer->name()) == 0)
    {
      return layer;
    }
  }

  return nullptr;
}


const MapLayer &MapLayout::layer(size_t index) const
{
  return *imp_->layers[index];
}


const MapLayer *MapLayout::layerPtr(size_t index) const
{
  return (imp_ && index < imp_->layers.size()) ? imp_->layers[index] : nullptr;
}


MapLayer *MapLayout::layerPtr(size_t index)
{
  return (imp_ && index < imp_->layers.size()) ? imp_->layers[index] : nullptr;
}


size_t MapLayout::layerCount() const
{
  return imp_->layers.size();
}


MapLayout &MapLayout::operator=(MapLayout &&other) noexcept
{
  delete imp_;
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}


MapLayout &MapLayout::operator=(const MapLayout &other)
{
  if (other.imp_)
  {
    for (auto &&layer : other.imp_->layers)
    {
      MapLayer *new_layer = addLayer(layer->name(), layer->subsampling());
      new_layer->copyVoxelLayout(*layer);
    }
  }
  return *this;
}
