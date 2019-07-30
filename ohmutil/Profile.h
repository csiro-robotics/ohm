// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_PROFILE_H
#define OHMUTIL_PROFILE_H

#include "ProfileMarker.h"

#include <chrono>
#include <iosfwd>

namespace ohm
{
  using ProfileClock = std::chrono::high_resolution_clock;
  struct ProfileDetail;

  /// A generalised instrumented profiling API.
  ///
  /// This class provides the means for adding profiling markers to track how long code takes to execute via a
  /// singleton instance. Except for reporting, the class should not be used directly, instead using @c ProfileMarker,
  /// or the @c PROFILE(), @c PROFILE_END() and @c PROFILE_RESTART() macros (preferred).
  ///
  /// General usage is to add a @c PROFILE() marker at the start of the scope which is to be profiled;
  /// equivalent to declaring a local @c ProfileMarker variable. This pushes a named marker onto the profiling stack
  /// via @c push(). When the @c ProfileMarker goes out of scope, it calls @c pop(), which removes the marker
  /// and adds the time spend within the named scope to the internal database.
  ///
  /// Markers may be nested, where one @c ProfileMarker scope contained another with the outer scope including the
  /// time spend in the inner scope. The profiling system maintains a named scope stack per thread via @c push() and
  /// @c pop(). This is threadsafe with each call implicitly resolving the active @c std::thread. Scope naming is
  /// relative to the parent scope, so two scopes may have the same name with different parents (and/or within different
  /// threads) and will be individually tracked.
  ///
  /// On program exit, the profile report is automatically shown so long as @c Profile::instance() has been called at
  /// least once (generally from the @c ProfileMarker constructor and destructor). The report may be shown sooner, or
  /// logged to @c std::ostream by explicitly calling @c report(). The report logs the total and average time spend in
  /// each scope in a hierarchical fashion.
  ///
  /// The report is printed in the following format:
  /// @code{.unparsed}
  /// ----------------------------------------
  ///  Profile report
  /// ----------------------------------------
  /// thread <id>
  ///   <scope> avg: <average-time> total: <total-time> / <call-count> calls
  ///     <scope> avg: <average-time> total: <total-time> / <call-count> calls
  ///       <scope> avg: <average-time> total: <total-time> / <call-count> calls
  ///     <scope> avg: <average-time> total: <total-time> / <call-count> calls
  ///     <scope> avg: <average-time> total: <total-time> / <call-count> calls
  /// thread <id>
  ///   <scope> avg: <average-time> total: <total-time> / <call-count> calls
  /// ----------------------------------------
  /// ----------------------------------------
  /// @endcode
  class Profile
  {
  public:
    /// Profile system constructor.
    Profile();
    /// @c report() and cleanup the profile system.
    ~Profile();

    /// Access the singleton profile instance. This is what @c ProfileMarker uses by default.
    /// @return A singleton instance of the profile system.
    static Profile &instance();

    /// Push a named scope onto the profile stack. Timing for this name is tracked until @c pop() is called and
    /// this name is at the top of the stack.
    ///
    /// @par String Assumption
    /// The assumption is made that the memory for @p name remains valid throughout the lifetime of the program.
    /// This is most easily achieved by using string literals.
    ///
    /// @param name Name of the scope to profile.
    /// @return True on success. False indicates a stack error and the marker has not been pushed (e.g., self reference)
    bool push(const char *name);
    /// Pop the last scope given to @c push() and record the time spent.
    void pop();

    /// Print a report of the current profiling time spent.
    /// The profile is printed to @p optr if provided or @c std::cout otherwise.
    ///
    /// Prevent further reports unless new timing information is given (@c push() or @c pop() called).
    ///
    /// @param optr The output stream to print to.
    void report(std::ostream *optr = nullptr);
    /// @overload
    inline void report(std::ostream &optr) { report(&optr); }

    /// Suppresses @c report() printing.
    void suppressReport(bool suppress);

    /// Checks if reporting has been suppressed.
    /// @return True when reporting has been suppressed.
    bool reportSupressed() const;

  private:
    ProfileDetail *imp_;
    static Profile s_instance;
  };
}  // namespace ohm

/// @def PROFILE(name)
/// Pushes a @c ProfileMarker to the default @c Profile using the given @p name.
///
/// Ignored when @c PROFILING is not defined or zero.
/// @param name The profile marker name. Must contain only characters valid for an identifier.
/// @see Profile

/// @def PROFILE_END(name)
/// Pops the named @c ProfileMarker from the default Profile. The @p name must match that of a previous @c PROFILE()
/// statement within the current scope.
///
/// Ignored when @c PROFILING is not defined or zero.
/// @param name The profile marker name. Must contain only characters valid for an identifier.
/// @see Profile

/// @def PROFILE_RESTART(name)
/// Re-pushes the named @c ProfileMarker to the default Profile. The @p name must match that of a previous @c PROFILE()
/// statement within the current scope.
///
/// Ignored when @c PROFILING is not defined or zero.
/// @param name The profile marker name. Must contain only characters valid for an identifier.
/// @see Profile

#if PROFILING

#define PROFILE(name) ohm::ProfileMarker __profile##name(#name);
#define PROFILE_IF(name, cond) ohm::ProfileMarker __profile##name(#name, cond);
#define PROFILE_END(name) __profile##name.end();
#define PROFILE_RESTART(name) __profile##name.restart();
#define PROFILE_RESTART_IF(name, cond) __profile##name.restart(cond);

#define PROFILE2(name, prof) ohm::ProfileMarker __profile##name(#name, prof);
#define PROFILE_IF2(name, cond) ohm::ProfileMarker __profile##name(#name, prof, cond);

#else  // PROFILING

#define PROFILE(name)
#define PROFIILE_IF(name)
#define PROFILE_END(name)
#define PROFILE_RESTART(name)
#define PROFILE_RESTART_IF(name)

#endif  // PROFILING

#endif  // OHMUTIL_PROFILE_H
