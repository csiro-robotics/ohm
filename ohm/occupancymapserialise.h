//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OCCUPANCYMAPSERIALISATION_H_
#define OCCUPANCYMAPSERIALISATION_H_

#include "ohmconfig.h"

#include <cinttypes>

namespace ohm
{
  class OccupancyMap;

    /// An enumeration of potential serialisation errors.
  enum SerialisationError
  {
    /// No error.
    SE_OK = 0,
    /// Failed to create a file.
    SE_FileCreateFailure,
    /// Failed to open a file for reading.
    SE_FileOpenFailure,

    /// Failed to write to a file.
    SE_FileWriteFailure,
    /// Failed to read a file.
    SE_FileReadFailure,

    SE_ValueOverflow,

    SE_MemberOffsetError,

    SE_UnsupportedVersion
  };

  /// Structure holding information about a loaded map version number.
  struct MapVersion
  {
    uint32_t major = 0;
    uint16_t minor = 0;
    uint16_t patch = 0;

    inline MapVersion() {};
    inline MapVersion(uint32_t major, uint16_t minor = 0, uint16_t patch = 0)
      : major(major), minor(minor), patch(patch)
    {
    }

    inline MapVersion &operator = (const MapVersion &other)
    {
      major = other.major;
      minor = other.minor;
      patch = other.patch;
      return *this;
    }

    inline bool operator == (const MapVersion &other) const
    {
      return major == other.major && minor == other.minor && patch == other.patch;
    }

    inline bool operator != (const MapVersion &other) const
    {
      return !operator==(other);
    }

    inline bool operator < (const MapVersion &other) const
    {
      return major < other.major && minor < other.minor && patch < other.patch;
    }

    inline bool operator <= (const MapVersion &other) const
    {
      return major <= other.major && minor <= other.minor && patch <= other.patch;
    }

    inline bool operator > (const MapVersion &other) const
    {
      return major > other.major && minor > other.minor && patch > other.patch;
    }

    inline bool operator >= (const MapVersion &other) const
    {
      return major >= other.major && minor >= other.minor && patch >= other.patch;
    }
  };

  /// Header marker bytes for a serialised occupancy map.
  extern const uint32_t MapHeaderMarker;
  /// Minimum version number which can be loaded by this library.
  extern const MapVersion SupportedVersionMin;
  /// Maximum version number which can be loaded by this library.
  extern const MapVersion SupportedVersionMax;
  /// Current MapVersion version.
  extern const MapVersion CurrentVersion;

  /// Progress observer interface for serialisation.
  ///
  /// This can be derived to track serialisation progress in @c save() and @c load().
  /// When given to one of those methods, the following methods will be called during serialisation:
  /// - @c setTargetProgess() to set the maximum progress value once known.
  /// - @c incrementProgress() periodically to increment progress up to the target.
  ///
  /// The @c quit() method may also be used to abort serialisation. It is called periodically to
  /// check if serialisation should abort.
  class ohm_API SerialiseProgress
  {
  public:
    /// Virtual destructor.
    virtual inline ~SerialiseProgress() {}

    /// Should serialisation quit early?
    /// @return True to quit/abort.
    virtual bool quit() const = 0;
    /// Sets the target progress value. Current progress should be reset when this is called.
    /// @param target The maximum progress value.
    virtual void setTargetProgress(unsigned target) = 0;
    /// Increment progress towards the target value.
    /// @param inc The increment. Generally 1.
    virtual void incrementProgress(unsigned inc = 1) = 0;
  };

  /// Save @p map to @p filename.
  ///
  /// This method saves an @c OccuancyMap to file. The progress may optionally be tracked by providing
  /// a @c SerialiseProgress object via @p progress. That object may also be used to abort serialisation
  /// should it's @c SerialiseProgress::quit() method report @c true.
  ///
  /// @param filename The name of the file to save to.
  /// @param map The map to save.
  /// @param progress Optional progress tracking object.
  /// @return @c SE_OK on success, or a non zero @c SerialisationError on failure.
  int ohm_API save(const char *filename, const OccupancyMap &map, SerialiseProgress *progress = nullptr);

  /// Load @p map from @p filename.
  ///
  /// This method loads an @c OccuancyMap from file. The progress may optionally be tracked by providing
  /// a @c SerialiseProgress object via @p progress. That object may also be used to abort serialisation
  /// should it's @c SerialiseProgress::quit() method report @c true.
  ///
  /// The current content of @p map is overwritten by the loaded data.
  ///
  /// @param filename The name of the file to load from.
  /// @param map The map object to load into.
  /// @param progress Optional progress tracking object.
  /// @param[out] versionOut When present, set to the version number of the loaded map format.
  /// @return @c SE_OK on success, or a non zero @c SerialisationError on failure.
  int ohm_API load(const char *filename, OccupancyMap &map, SerialiseProgress *progress = nullptr, MapVersion *versionOut = nullptr);

  /// Loads the header and layers of a map file without loading the chunks for voxel data.
  ///
  /// The resulting @p map contains no chunks or voxel data, but does contain valid @c MapLayout data.
  /// This can be used to inspect information about the map, but without the data content.
  ///
  /// The current content of @p map is emptied and overwritten by the loaded data.
  ///
  /// @param filename The name of the file to load from.
  /// @param map The map object to load into.
  /// @param[out] versionOut When present, set to the version number of the loaded map format.
  /// @return @c SE_OK on success, or a non zero @c SerialisationError on failure.
  int ohm_API loadHeader(const char *filename, OccupancyMap &map, MapVersion *versionOut = nullptr);
}

#endif // OCCUPANCYMAPSERIALISATION_H_
