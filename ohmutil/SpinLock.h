//
// author: Kazys Stepanas
//
#ifndef OHMUTIL_SPINLOCK_H
#define OHMUTIL_SPINLOCK_H

#include "OhmUtilExport.h"

struct SpinLockImp;

/// A spin lock implementation. Preferred over std::mutex as that class
/// can be very slow (i.e, Clang OSX).
///
/// This is a naive implementation and does not support re-locking.
///
/// Best used with @c std::unique_lock as an exception and scope safe guard.
///
/// @note This class is deprecated. It is only about as fast as GCC's actual mutex. Use tbb::spin_mutex for a spin lock.
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
  bool try_lock();  // NOLINT

  /// Unlock the lock. Should only ever be called by the scope which called @c lock()
  /// or succeeded at @c try_lock.
  void unlock();

private:
  SpinLockImp *imp_;  ///< Implementation detail.
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/// Scope lock for @c SpinLock .
/// Deprecated.
class ScopedSpinLock
{
public:
  inline ScopedSpinLock(SpinLock &lock)
    : lock_(lock)
    , have_lock_(true)
  {
    lock_.lock();
  }
  inline ~ScopedSpinLock() { unlock(); }

  inline void lock()
  {
    if (!have_lock_)
    {
      lock_.lock();
      have_lock_ = true;
    }
  }
  inline void unlock()
  {
    if (have_lock_)
    {
      lock_.unlock();
      have_lock_ = false;
    }
  }

  // Match naming with std::mutex
  inline void try_lock()  // NOLINT
  {
    if (!have_lock_)
    {
      have_lock_ = lock_.try_lock();
    }
  }  // NOLINT

private:
  SpinLock lock_;
  bool have_lock_;
};

#endif  // DOXYGEN_SHOULD_SKIP_THIS

#endif  // OHMUTIL_SPINLOCK_H
