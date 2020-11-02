//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OHM_MAPSERIALISATION_H
#define OHM_MAPSERIALISATION_H

#include "OhmConfig.h"

#include <glm/fwd.hpp>

#include <cinttypes>

#ifdef major
#undef major
#endif  // major
#ifdef minor
#undef minor
#endif  // minor
#ifdef patch
#undef patch
#endif  // patch

namespace ohm
{
class Heightmap;
class OccupancyMap;

/// An enumeration of potential serialisation errors.
enum SerialisationError
{
  /// No error.
  kSeOk = 0,
  /// Failed to create a file.
  kSeFileCreateFailure,
  /// Failed to open a file for reading.
  kSeFileOpenFailure,

  /// Failed to write to a file.
  kSeFileWriteFailure,
  /// Failed to read a file.
  kSeFileReadFailure,

  kSeValueOverflow,

  kSeMemberOffsetError,

  /// Error serialising @c MapInfo.
  kSeInfoError,

  /// @c MapInfo does not represent a heightmap.
  kSeHeightmapInfoMismatch,

  kSeDataItemTooLarge,
  kSeUnknownDataType,

  kSeUnsupportedVersion
};

/// Structure holding information about a loaded map version number specified as `<major>.<minor>.<patch>`.
struct MapVersion
{
  uint32_t major = 0;  ///< The major version number.
  uint16_t minor = 0;  ///< The minor version number.
  uint16_t patch = 0;  ///< The patch version number.

  /// Default constructor - generates a zero version number.
  inline MapVersion() = default;
  /// Construct with the given version number.
  /// @param major The major version number.
  /// @param minor The minor version number.
  /// @param patch The patch version number.
  inline MapVersion(uint32_t major, uint16_t minor = 0, uint16_t patch = 0)
    : major(major)
    , minor(minor)
    , patch(patch)
  {}

  /// Copy by value assignment.
  /// @param other Object to copy.
  inline MapVersion &operator=(const MapVersion &other) = default;

  /// Equalty operator comparing two version for exact match.
  /// @param other Version to compare against.
  /// @return True if the version numbers exactly match.
  inline bool operator==(const MapVersion &other) const
  {
    return major == other.major && minor == other.minor && patch == other.patch;
  }

  /// Negated equality operator comparing two version.
  /// @param other Version to compare against.
  /// @return True if the version numbers do not exactly match.
  inline bool operator!=(const MapVersion &other) const { return !operator==(other); }

  /// Inequality operator comparing two versions.
  /// @param other Version to compare against.
  /// @return True if this version number is less than @p other .
  inline bool operator<(const MapVersion &other) const
  {
    return major < other.major && minor < other.minor && patch < other.patch;
  }

  /// Inequality operator comparing two versions.
  /// @param other Version to compare against.
  /// @return True if this version number is less than or equal to @p other .
  inline bool operator<=(const MapVersion &other) const
  {
    return major <= other.major && minor <= other.minor && patch <= other.patch;
  }

  /// Inequality operator comparing two versions.
  /// @param other Version to compare against.
  /// @return True if this version number is greater than @p other .
  inline bool operator>(const MapVersion &other) const
  {
    return major > other.major && minor > other.minor && patch > other.patch;
  }

  /// Inequality operator comparing two versions.
  /// @param other Version to compare against.
  /// @return True if this version number is greater than or equal to @p other .
  inline bool operator>=(const MapVersion &other) const
  {
    return major >= other.major && minor >= other.minor && patch >= other.patch;
  }
};

/// Header marker bytes for a serialised occupancy map.
extern const uint32_t kMapHeaderMarker;
/// Minimum version number which can be loaded by this library.
extern const MapVersion kSupportedVersionMin;
/// Maximum version number which can be loaded by this library.
extern const MapVersion kSupportedVersionMax;
/// Current MapVersion version.
extern const MapVersion kCurrentVersion;

/// Progress observer interface for serialisation.
///
/// This can be derived to track serialisation progress in @c save() and @c load().
/// When given to one of those methods, the following methods will be called during serialisation:
/// - @c setTargetProgress() to set the maximum progress value once known.
/// - @c incrementProgress() periodically to increment progress up to the target.
///
/// The @c quit() method may also be used to abort serialisation. It is called periodically to
/// check if serialisation should abort.
class ohm_API SerialiseProgress
{
public:
  /// Virtual destructor.
  virtual inline ~SerialiseProgress() = default;

  /// Should serialisation quit early?
  /// @return True to quit/abort.
  virtual bool quit() const = 0;
  /// Sets the target progress value. Current progress should be reset when this is called.
  /// @param target The maximum progress value.
  virtual void setTargetProgress(unsigned target) = 0;
  /// Increment progress towards the target value.
  /// @param inc The increment. Generally 1.
  virtual void incrementProgress(unsigned inc) = 0;
  /// @overload
  inline void incrementProgress() { incrementProgress(1); }
};

/// Translate @c err to an English error code string.
/// @param err The error code. Out of range values are handled.
/// @return The English error code string or "<unknown>".
const char *errorCodeString(int err);

/// Save @p map to @p filename.
///
/// This method saves an @c OccupancyMap to file. The progress may optionally be tracked by providing
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
/// This method loads an @c OccupancyMap from file. The progress may optionally be tracked by providing
/// a @c SerialiseProgress object via @p progress. That object may also be used to abort serialisation
/// should it's @c SerialiseProgress::quit() method report @c true.
///
/// The current content of @p map is overwritten by the loaded data.
///
/// @param filename The name of the file to load from.
/// @param map The map object to load into.
/// @param progress Optional progress tracking object.
/// @param[out] version_out When present, set to the version number of the loaded map format.
/// @return @c SE_OK on success, or a non zero @c SerialisationError on failure.
int ohm_API load(const char *filename, OccupancyMap &map, SerialiseProgress *progress = nullptr,
                 MapVersion *version_out = nullptr);

int ohm_API load(const char *filename, Heightmap &heightmap, SerialiseProgress *progress = nullptr,
                 MapVersion *version_out = nullptr);

/// Loads the header and layers of a map file without loading the chunks for voxel data.
///
/// The resulting @p map contains no chunks or voxel data, but does contain valid @c MapLayout data.
/// This can be used to inspect information about the map, but without the data content.
///
/// The current content of @p map is emptied and overwritten by the loaded data.
///
/// @param filename The name of the file to load from.
/// @param map The map object to load into.
/// @param[out] version_out When present, set to the version number of the loaded map format.
/// @param[out] region_count When present, set to the number of regions in the map. Regions are not loaded.
/// @return @c SE_OK on success, or a non zero @c SerialisationError on failure.
int ohm_API loadHeader(const char *filename, OccupancyMap &map, MapVersion *version_out = nullptr,
                       size_t *region_count = nullptr);
}  // namespace ohm

#endif  // OHM_MAPSERIALISATION_H
