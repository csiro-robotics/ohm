// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef REF_H
#define REF_H

#include "gpuConfig.h"

#include <mutex>
#include <functional>

namespace gputil
{
  template <typename T>
  class Ref
  {
  public:
    typedef std::function<void (T&)> ReleaseFunc;

    Ref(T obj, unsigned initial_ref_count, const ReleaseFunc &release);
    Ref(Ref &&other);
    Ref(const Ref &other) = delete;

  protected:
    ~Ref();

  public:
    unsigned reference();
    unsigned release();

    void set(T obj, unsigned refCount);

    inline T obj() { return obj_; }
    inline const T obj() const { return obj_; }

    inline T operator()() { return obj_; }
    inline const T operator()() const { return obj_; }

    inline unsigned referenceCount() const { return reference_count_; }

    Ref &operator=(Ref &&other);
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
  {
  }


  template <typename T>
  inline Ref<T>::Ref(Ref &&other)
    : obj_(other.obj_)
    , reference_count_(other.reference_count_)
    , release_func_(other.release_func_)
  {
    other.reference_count_ = 0;
  }


  template <typename T>
  inline Ref<T>::~Ref()
  {
  }


  template <typename T>
  inline unsigned Ref<T>::reference()
  {
    std::unique_lock<std::mutex> guard(lock_);
    if (reference_count_)
    {
      ++reference_count_;
    }
  }


  template <typename T>
  inline unsigned Ref<T>::release()
  {
    lock_.lock();
    if (reference_count_)
    {
      --reference_count_;
      if (reference_count_ == 0)
      {
        release_func_(obj_);
        delete this;
      }
    }
    lock_.unlock();
  }


  template <typename T>
  inline void Ref<T>::set(T obj, unsigned refCount)
  {
    std::unique_lock<std::mutex> guard(lock_);
    if (reference_count_)
    {
      // Should be an error when reference_count_ > 1.
      release_func_(obj_);
    }

    obj_ = obj;
    reference_count_ = refCount;
  }


  template <typename T>
  inline Ref<T> &Ref<T>::operator=(Ref &&other)
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
}

#endif // REF_H
