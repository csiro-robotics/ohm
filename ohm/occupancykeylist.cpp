//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "occupancykeylist.h"

#include <cmath>
#include <cstring>

using namespace ohm;

inline size_t ceilPowerOf2(size_t v)
{
  size_t next;
  bool isPow2;
  isPow2 = v && !(v & (v - 1));
  next = size_t(1) << (size_t(1) + size_t(std::floor(std::log2(float(v)))));
  return isPow2 ? v : next;
}


OccupancyKeyList::OccupancyKeyList(size_t initialCount)
  : _keys(nullptr)
  , _capacity(0)
  , _count(0)
{
  if (initialCount)
  {
    resize(ceilPowerOf2(initialCount));
  }
  else
  {
    reserve(32);
  }
}


OccupancyKeyList::~OccupancyKeyList()
{
  delete[] _keys;
}


void OccupancyKeyList::reserve(size_t capacity)
{
  if (_capacity < capacity)
  {
    OccupancyKey *keys = new OccupancyKey[capacity];
    if (_keys && _count)
    {
      memcpy(keys, _keys, sizeof(*keys) * _count);
    }
    delete[] _keys;
    _keys = keys;
    _capacity = capacity;
  }
}


void OccupancyKeyList::resize(size_t count)
{
  if (_capacity < count)
  {
    reserve(count);
  }
  _count = count;
}


void OccupancyKeyList::push_back(const OccupancyKey &key)
{
  if (_count == _capacity)
  {
    reserve(ceilPowerOf2(_count + 1));
  }
  _keys[_count++] = key;
}


OccupancyKey &OccupancyKeyList::add()
{
  if (_count == _capacity)
  {
    reserve(ceilPowerOf2(_count + 1));
  }
  return _keys[_count++];
}
