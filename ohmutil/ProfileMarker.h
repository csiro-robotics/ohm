// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_PROFILEMARKER_H
#define OHMUTIL_PROFILEMARKER_H

namespace ohmutil
{
  class Profile;

  /// A utility class for profiling a scope.
  ///
  /// The marker pushes itself onto the current profiling stack via @c Profile::push() on creation and
  /// pops itself on destruction. Push and pop operations may be explicitly controlled by using @c end() - pop - and
  /// @c restart() - push. These are ignored if the marker is not in the appropriate state.
  ///
  /// @see Profile
  class ProfileMarker
  {
  public:
    /// Create a marker with the given name using the @c Profile::instance().
    ///
    /// @par String Assumption
    /// The assumption is made that the memory for @p name remains valid throughout the lifetime of the program.
    /// This is most easily achieved by using string literals.
    ///
    /// @param name Name of the scope to profile.
    /// @param activate Only start profiling if this is true. Useful for conditional profiling.
    ProfileMarker(const char *name, bool activate = true);
    /// Create a marker with the given name targetting the given @p profile.
    ///
    /// @par String Assumption
    /// The assumption is made that the memory for @p name remains valid throughout the lifetime of the program.
    /// This is most easily achieved by using string literals.
    ///
    /// @param name Name of the scope to profile.
    /// @param profile The profile system to add the marker to.
    /// @param activate Only start profiling if this is true. Useful for conditional profiling.
    ProfileMarker(const char *name, Profile *profile, bool activate = true);

    /// Pops the marker from the profile stack so long as @c end() has not been called.
    ~ProfileMarker();

    /// Explicitly pops the marker from the profile stack. The destructor will not pop it again unless @c restart()
    /// is called.
    void end();

    /// Pushes the marker onto the stack so long as @c end() has been called.
    /// @param activate Only restart if this is true. Useful for conditional profiling.
    void restart(bool activate = true);

  private:
    const char *name_;
    Profile *profile_;
    bool active_;
  };
}

#endif // OHMUTIL_PROFILEMARKER_H
