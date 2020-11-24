//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "KeyList.h"

#include <cmath>
#include <cstring>

namespace ohm
{
KeyList::KeyList(size_t initial_count)
{
  if (initial_count)
  {
    keys_.resize(initial_count);
  }
}


KeyList::~KeyList() = default;


void KeyList::reserve(size_t capacity)
{
  keys_.reserve(capacity);
}


void KeyList::resize(size_t count)
{
  keys_.resize(count);
}


void KeyList::emplace_back(const Key &key)  // NOLINT
{
  keys_.emplace_back(key);
}


Key &KeyList::add()
{
  keys_.emplace_back(Key::kNull);
  return keys_.back();
}
}  // namespace ohm
