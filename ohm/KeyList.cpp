//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "KeyList.h"

#include <cmath>
#include <cstring>

using namespace ohm;

inline size_t ceilPowerOf2(size_t v)
{
  const bool is_pow2 = v && !(v & (v - 1));
  const size_t next = size_t(1) << (size_t(1) + size_t(std::floor(std::log2(float(v)))));
  return is_pow2 ? v : next;
}


OccupancyKeyList::OccupancyKeyList(size_t initial_count)
  : keys_(nullptr)
  , capacity_(0)
  , count_(0)
{
  if (initial_count)
  {
    resize(ceilPowerOf2(initial_count));
  }
  else
  {
    reserve(32);
  }
}


OccupancyKeyList::~OccupancyKeyList()
{
  delete[] keys_;
}


void OccupancyKeyList::reserve(size_t capacity)
{
  if (capacity_ < capacity)
  {
    OccupancyKey *keys = new OccupancyKey[capacity];
    if (keys_ && count_)
    {
      memcpy(keys, keys_, sizeof(*keys) * count_);
    }
    delete[] keys_;
    keys_ = keys;
    capacity_ = capacity;
  }
}


void OccupancyKeyList::resize(size_t count)
{
  if (capacity_ < count)
  {
    reserve(count);
  }
  count_ = count;
}


void OccupancyKeyList::push_back(const OccupancyKey &key)
{
  if (count_ == capacity_)
  {
    reserve(ceilPowerOf2(count_ + 1));
  }
  keys_[count_++] = key;
}


OccupancyKey &OccupancyKeyList::add()
{
  if (count_ == capacity_)
  {
    reserve(ceilPowerOf2(count_ + 1));
  }
  return keys_[count_++];
}
