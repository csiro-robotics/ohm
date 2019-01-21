//
// author: Kazys Stepanas
//
#include "SpinLock.h"

#include <atomic>
#include <thread>

struct SpinLockImp
{
  std::atomic_bool lock;

  inline SpinLockImp()
    : lock(false)
  {}
};

SpinLock::SpinLock()
  : imp_(new SpinLockImp)
{}


SpinLock::~SpinLock()
{
  delete imp_;
}


void SpinLock::lock()
{
  while (imp_->lock.exchange(true))
  {
    std::this_thread::yield();
  }
}


bool SpinLock::try_lock()  // NOLINT
{
  return !imp_->lock.exchange(true);
}


void SpinLock::unlock()
{
  imp_->lock = false;
}
