//
// author: Kazys Stepanas
//
#ifndef SPINLOCK_H_
#define SPINLOCK_H_

#include "ohmutilexport.h"

struct SpinLockImp;

/// A spin lock implementation. Prefered over std::mutex as that class
/// can be very slow (i.e, Clang OSX).
///
/// This is a naive implementation and does not support re-locking.
///
/// Best used with @c std::unique_lock as an exception and scope safe guard.
class ohmutil_API SpinLock
{
public:
  /// Construct a spin lock (unlocked).
  SpinLock();
  /// Destructor.
  ~SpinLock();

  /// Block until the spin lock can be attained.
  void lock();

  /// Try attain the lock without blocking.
  /// @return True if the lock is attained, false if it could not be attained.
  bool try_lock();

  /// Unlock the lock. Should only ever be called by the scope which called @c lock()
  /// or succeeded at @c try_lock.
  void unlock();

private:
  SpinLockImp *_imp;  ///< Implementation detail.
};

class ScopedSpinLock
{
public:
  inline ScopedSpinLock(SpinLock &lock) : _lock(lock), _haveLock(true) { _lock.lock(); }
  inline ~ScopedSpinLock() { unlock(); }

  inline void lock() { if (!_haveLock) { _lock.lock(); _haveLock = true; } }
  inline void unlock() { if (_haveLock) { _lock.unlock(); _haveLock = false; } }
  inline void try_lock() { if (!_haveLock) { _haveLock = _lock.try_lock(); } }

private:
  SpinLock _lock;
  bool _haveLock;
};


#endif // SPINLOCK_H_
