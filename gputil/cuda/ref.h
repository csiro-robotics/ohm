// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef REF_H_
#define REF_H_

#include "gpuconfig.h"

#include <mutex>
#include <functional>

namespace gputil
{
  template <typename T>
  class Ref
  {
  public:
    typedef std::function<void (T&)> ReleaseFunc;

    Ref(T obj, unsigned initialRefCount, const ReleaseFunc &release);
    Ref(Ref &&other);
    Ref(const Ref &other) = delete;

  protected:
    ~Ref();

  public:
    unsigned reference();
    unsigned release();

    void set(T obj, unsigned refCount);

    inline T obj() { return _obj; }
    inline const T obj() const { return _obj; }

    inline T operator()() { return _obj; }
    inline const T operator()() const { return _obj; }

    inline unsigned referenceCount() const { return _referenceCount; }

    Ref &operator=(Ref &&other);
    Ref &operator=(const Ref &other) = delete;

  private:
    T _obj;
    unsigned _referenceCount;
    ReleaseFunc _releaseFunc;
    std::mutex _lock;
  };

  template <typename T>
  inline Ref<T>::Ref(T obj, unsigned initialRefCount, const ReleaseFunc &release)
    : _obj(obj)
    , _referenceCount(initialRefCount)
    , _releaseFunc(release)
  {
  }


  template <typename T>
  inline Ref<T>::Ref(Ref &&other)
    : _obj(other._obj)
    , _referenceCount(other._referenceCount)
    , _releaseFunc(other._releaseFunc)
  {
    other._referenceCount = 0;
  }


  template <typename T>
  inline Ref<T>::~Ref()
  {
  }


  template <typename T>
  inline unsigned Ref<T>::reference()
  {
    std::unique_lock<std::mutex> guard(_lock);
    if (_referenceCount)
    {
      ++_referenceCount;
    }
  }


  template <typename T>
  inline unsigned Ref<T>::release()
  {
    _lock.lock();
    if (_referenceCount)
    {
      --_referenceCount;
      if (_referenceCount == 0)
      {
        _releaseFunc(_obj);
        delete this;
      }
    }
    _lock.unlock();
  }


  template <typename T>
  inline void Ref<T>::set(T obj, unsigned refCount)
  {
    std::unique_lock<std::mutex> guard(_lock);
    if (_referenceCount)
    {
      // Should be an error when _referenceCount > 1.
      _releaseFunc(_obj);
    }

    _obj = obj;
    _referenceCount = refCount;
  }


  template <typename T>
  inline Ref<T> &Ref<T>::operator=(Ref &&other)
  {
    std::unique_lock<std::mutex> guard(_lock);
    if (_referenceCount)
    {
      // Should be an error when _referenceCount > 1.
      _releaseFunc(_obj);
    }
    _referenceCount = other._referenceCount;
    _obj = other._obj;
    _releaseFunc = other._releaseFunc;
    other._referenceCount = 0;
    return *this;
  }
}

#endif // REF_H_
