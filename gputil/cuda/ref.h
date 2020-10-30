// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef REF_H
#define REF_H

#include "gpuConfig.h"

#include <functional>
#include <mutex>

namespace gputil
{
template <typename T>
class Ref
{
public:
  using ReleaseFunc = std::function<void(T &)>;

  Ref(T obj, unsigned initial_ref_count, const ReleaseFunc &release);
  Ref(Ref &&other) noexcept;
  Ref(const Ref &other) = delete;

protected:
  ~Ref();

public:
  unsigned reference();
  unsigned release();

  void set(T obj, unsigned ref_count);

  inline T obj() { return obj_; }
  inline const T obj() const { return obj_; }

  inline T operator()() { return obj_; }
  inline const T operator()() const { return obj_; }

  inline unsigned referenceCount() const { return reference_count_; }

  Ref &operator=(Ref &&other) noexcept;
  Ref &operator=(const Ref &other) = delete;

private:
  T obj_;
  unsigned reference_count_;
  ReleaseFunc release_func_;
  std::mutex lock_;
};

template <typename T>
inline Ref<T>::Ref(T obj, unsigned initial_ref_count, const ReleaseFunc &release)
  : obj_(obj)
  , reference_count_(initial_ref_count)
  , release_func_(release)
{}


template <typename T>
inline Ref<T>::Ref(Ref &&other) noexcept
  : obj_(other.obj_)
  , reference_count_(other.reference_count_)
  , release_func_(std::move(other.release_func_))
{
  other.reference_count_ = 0;
}


template <typename T>
inline Ref<T>::~Ref() = default;


template <typename T>
inline unsigned Ref<T>::reference()
{
  std::unique_lock<std::mutex> guard(lock_);
  if (reference_count_)
  {
    ++reference_count_;
  }
  return reference_count_;
}


template <typename T>
inline unsigned Ref<T>::release()
{
  std::unique_lock<std::mutex> guard(lock_);
  unsigned released_count = 0;
  if (reference_count_)
  {
    --reference_count_;
    released_count = reference_count_;
    if (reference_count_ == 0)
    {
      release_func_(obj_);
      guard.unlock();
      delete this;
    }
  }

  return released_count;
}


template <typename T>
inline void Ref<T>::set(T obj, unsigned ref_count)
{
  std::unique_lock<std::mutex> guard(lock_);
  if (reference_count_)
  {
    // Should be an error when reference_count_ > 1.
    release_func_(obj_);
  }

  obj_ = obj;
  reference_count_ = ref_count;
}


template <typename T>
inline Ref<T> &Ref<T>::operator=(Ref &&other) noexcept
{
  std::unique_lock<std::mutex> guard(lock_);
  if (reference_count_)
  {
    // Should be an error when reference_count_ > 1.
    release_func_(obj_);
  }
  reference_count_ = other.reference_count_;
  obj_ = other.obj_;
  release_func_ = other.release_func_;
  other.reference_count_ = 0;
  return *this;
}
}  // namespace gputil

#endif  // REF_H
