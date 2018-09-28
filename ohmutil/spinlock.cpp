//
// author: Kazys Stepanas
//
#include "spinlock.h"

#include <atomic>
#include <thread>

struct SpinLockImp
{
  std::atomic_bool lock;

  inline SpinLockImp()
    : lock(false)
  {
  }
};

SpinLock::SpinLock()
  : _imp(new SpinLockImp)
{
}


SpinLock::~SpinLock()
{
  delete _imp;
}


void SpinLock::lock()
{
  while (_imp->lock.exchange(true))
  {
    std::this_thread::yield();
  }
}


bool SpinLock::try_lock()
{
  return !_imp->lock.exchange(true);
}


void SpinLock::unlock()
{
  _imp->lock = false;
}
