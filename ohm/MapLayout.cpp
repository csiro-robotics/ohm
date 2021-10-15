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

namespace ohm
{
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
    imp_->clear();
  }
}


int MapLayout::occupancyLayer() const
{
  return imp_->occupancy_layer;
}


int MapLayout::meanLayer() const
{
  return imp_->mean_layer;
}


int MapLayout::traversalLayer() const
{
  return imp_->traversal_layer;
}


int MapLayout::covarianceLayer() const
{
  return imp_->covariance_layer;
}


int MapLayout::clearanceLayer() const
{
  return imp_->clearance_layer;
}


int MapLayout::intensityLayer() const
{
  return imp_->intensity_layer;
}


int MapLayout::hitMissCountLayer() const
{
  return imp_->hit_miss_count_layer;
}


MapLayoutMatch MapLayout::checkEquivalent(const MapLayout &other) const
{
  if (this == &other)
  {
    return MapLayoutMatch::kExact;
  }

  // Check the obvious first: number of layers and layer sizes.
  if (layerCount() != other.layerCount())
  {
    return MapLayoutMatch::kDifferent;
  }

  MapLayoutMatch match = MapLayoutMatch::kExact;
  for (size_t i = 0; i < layerCount(); ++i)
  {
    const MapLayoutMatch layer_match = layer(i).checkEquivalent(other.layer(i));
    match = std::min(match, layer_match);
    if (match == MapLayoutMatch::kDifferent)
    {
      return MapLayoutMatch::kDifferent;
    }
  }

  return match;
}


size_t MapLayout::calculateOverlappingLayerSet(std::vector<std::pair<unsigned, unsigned>> &overlap,
                                               const MapLayout &other) const
{
  size_t matched = 0;
  // Special case self overlap.
  if (this == &other)
  {
    for (const MapLayer *layer : imp_->layers)
    {
      assert(layer);
      overlap.emplace_back(std::make_pair<unsigned, unsigned>(layer->layerIndex(), layer->layerIndex()));
      ++matched;
    }
    return matched;
  }

  for (const MapLayer *layer : imp_->layers)
  {
    assert(layer);
    const int other_layer_index = other.layerIndex(layer->name());
    if (other_layer_index >= 0)
    {
      // Matched by name. Check layout equivalence.
      const MapLayoutMatch layer_match = layer->checkEquivalent(other.layer(other_layer_index));
      if (layer_match == MapLayoutMatch::kExact)
      {
        overlap.emplace_back(std::make_pair<unsigned, unsigned>(layer->layerIndex(), unsigned(other_layer_index)));
        ++matched;
      }
    }
  }
  return matched;
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

  ohm::filterLayers(*imp_, preserve_indices);
  // Rebind the layer index caches.
  cacheLayerIndices();
}


void MapLayout::filterLayers(const std::initializer_list<unsigned> &preserve_layers)
{
  ohm::filterLayers(*imp_, preserve_layers);
  // Rebind the layer index caches.
  cacheLayerIndices();
}


MapLayer *MapLayout::addLayer(const char *name, uint16_t subsampling)
{
  auto *new_layer = new MapLayer(name, static_cast<uint16_t>(imp_->layers.size()), subsampling);
  imp_->layers.push_back(new_layer);

  cacheLayerIndex(new_layer);
  return new_layer;
}


void MapLayout::cacheLayerIndex(const MapLayer *layer)
{
  if (layer)
  {
    std::string name_str(layer->name());
    // This form of caching layer indices is not scalable. Do no more.
    if (imp_->occupancy_layer == -1 && name_str == default_layer::occupancyLayerName())
    {
      imp_->occupancy_layer = int(layer->layerIndex());
    }
    else if (imp_->mean_layer == -1 && name_str == default_layer::meanLayerName())
    {
      imp_->mean_layer = int(layer->layerIndex());
    }
    else if (imp_->traversal_layer == -1 && name_str == default_layer::traversalLayerName())
    {
      imp_->traversal_layer = int(layer->layerIndex());
    }
    else if (imp_->covariance_layer == -1 && name_str == default_layer::covarianceLayerName())
    {
      imp_->covariance_layer = int(layer->layerIndex());
    }
    else if (imp_->clearance_layer == -1 && name_str == default_layer::clearanceLayerName())
    {
      imp_->clearance_layer = int(layer->layerIndex());
    }
    else if (imp_->intensity_layer == -1 && name_str == default_layer::intensityLayerName())
    {
      imp_->intensity_layer = layer->layerIndex();
    }
    else if (imp_->hit_miss_count_layer == -1 && name_str == default_layer::hitMissCountLayerName())
    {
      imp_->hit_miss_count_layer = layer->layerIndex();
    }
  }
}


void MapLayout::cacheLayerIndices()
{
  imp_->occupancy_layer = -1;
  imp_->mean_layer = -1;
  imp_->traversal_layer = -1;
  imp_->covariance_layer = -1;
  imp_->clearance_layer = -1;
  imp_->intensity_layer = -1;
  imp_->hit_miss_count_layer = -1;
  for (MapLayer *layer : imp_->layers)
  {
    cacheLayerIndex(layer);
  }
}


const MapLayer *MapLayout::layer(const char *layer_name) const
{
  const std::string name(layer_name);
  for (const MapLayer *layer : imp_->layers)
  {
    if (layer && name == layer->name())
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


int MapLayout::layerIndex(const char *layer_name) const
{
  const std::string name(layer_name);
  for (const MapLayer *layer : imp_->layers)
  {
    if (layer && name == layer->name())
    {
      return int(layer->layerIndex());
    }
  }

  return -1;
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
  if (this != &other)
  {
    clear();
    if (other.imp_)
    {
      for (auto &&layer : other.imp_->layers)
      {
        MapLayer *new_layer = addLayer(layer->name(), layer->subsampling());
        new_layer->copyVoxelLayout(*layer);
      }
    }
  }
  return *this;
}
}  // namespace ohm
