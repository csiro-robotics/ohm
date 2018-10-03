// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapLayout.h"

#include "MapChunk.h"

#include "private/MapLayoutDetail.h"

#include <glm/glm.hpp>

#include <algorithm>

using namespace ohm;

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
  }
}


MapLayer *MapLayout::addLayer(const char *name, unsigned short subsampling)
{
  MapLayer *new_layer = new MapLayer(name, static_cast<unsigned short>(imp_->layers.size()), subsampling);
  imp_->layers.push_back(new_layer);
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
