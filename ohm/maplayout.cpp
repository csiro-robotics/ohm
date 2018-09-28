// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "maplayout.h"

#include "mapchunk.h"
#include "occupancymap.h"

#include "private/maplayoutdetail.h"

#include <glm/glm.hpp>

#include <algorithm>

using namespace ohm;

MapLayout::MapLayout()
  : _detail(new MapLayoutDetail)
{}


MapLayout::MapLayout(MapLayout &&other) noexcept
  : _detail(other._detail)
{
  other._detail = nullptr;
}


MapLayout::MapLayout(const MapLayout &other)
  : MapLayout()
{
  *this = other;
}


MapLayout::~MapLayout()
{
  clear();
  delete _detail;
}


void MapLayout::clear()
{
  if (_detail)
  {
    for (auto layer : _detail->layers)
    {
      delete layer;
    }
    _detail->layers.clear();
  }
}


MapLayer *MapLayout::addLayer(const char *name, unsigned short subsampling)
{
  MapLayer *newLayer = new MapLayer(name, (unsigned short)_detail->layers.size(), subsampling);
  _detail->layers.push_back(newLayer);
  return newLayer;
}


const MapLayer *MapLayout::layer(const char *layerName) const
{
  const std::string name(layerName);
  for (const MapLayer *layer : _detail->layers)
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
  return *_detail->layers[index];
}


const MapLayer *MapLayout::layerPtr(size_t index) const
{
  return (_detail && index < _detail->layers.size()) ? _detail->layers[index] : nullptr;
}


size_t MapLayout::layerCount() const
{
  return _detail->layers.size();
}


MapLayout &MapLayout::operator=(MapLayout &&other) noexcept
{
  delete _detail;
  _detail = other._detail;
  other._detail = nullptr;
  return *this;
}


MapLayout &MapLayout::operator=(const MapLayout &other)
{
  if (other._detail)
  {
    for (auto &&layer : other._detail->layers)
    {
      MapLayer *newLayer = addLayer(layer->name(), layer->subsampling());
      newLayer->copyVoxelLayout(*layer);
    }
  }
  return *this;
}
