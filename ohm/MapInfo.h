//
// @author Kazys Stepanas
//
// Copyright (c) 2015 CSIRO
//
#ifndef MAPINFO_H
#define MAPINFO_H

#include "OhmConfig.h"

#include <cinttypes>
#include <memory>

namespace ohm
{
/// A data value for use with @c MapInfo. Acts as a key/value pair, supporting
/// a variety of data types.
///
/// @par Conversion Operators
/// @parblock
/// The class includes cast operators for converting to any of the supported types.
/// Conversion succeeds even if the contained type does not match the target type,
/// including converting from strings to numeric types. However, numeric truncation
/// may occur and failure to convert is not detectable.
///
/// Conversion from string is implied, parsing the string to the appropriate type.
/// String conversion from bool includes converting the following strings (case insensitive):
/// { true, false, yes, no, on, off }.
///
/// Conversion to string is not automatic. The cast operator to string only works if
/// the type is @c kString. String conversion is better supported by @c toStringValue(),
/// which creates a new @c MapValue() of type @c kString.
/// @endparblock
class ohm_API MapValue
{
public:
  /// Supported data types for @c MapValue.
  enum Type
  {
    kTypeNone,
    kInt8,
    kUInt8,
    kInt16,
    kUInt16,
    kInt32,
    kUInt32,
    kInt64,
    kUInt64,
    kFloat32,
    kFloat64,
    kBoolean,
    kString
  };

  /// Create an invalid @c MapValue().
  MapValue();
  /// Constructor making a (deep) copy of @c other.
  /// @param other The value to copy.
  MapValue(const MapValue &other);
  /// R-value reference constructor.
  /// @param other The reference to steal data from.
  MapValue(MapValue &&other) noexcept;
  /// Create a @c kInt8 value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, int8_t val);
  /// Create a @c kUInt8 value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, uint8_t val);
  /// Create a @c kInt16 value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, int16_t val);
  /// Create a @c kUInt16 value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, uint16_t val);
  /// Create a @c kInt32 value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, int32_t val);
  /// Create a @c kUInt32 value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, uint32_t val);
  /// Create a @c kInt64 value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, int64_t val);
  /// Create a @c kUInt64 value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, uint64_t val);
  /// Create a @c kFloat32 value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, float val);
  /// Create a @c kFloat64 value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, double val);
  /// Create a @c kBoolean value.
  /// @param name The value key.
  /// @param val The value to assign.
  MapValue(const char *name, bool val);
  /// Create a @c kString value.
  /// @param name The value key.
  /// @param string The value to assign.
  MapValue(const char *name, const char *string);

  /// Copy assignment.
  /// @param other Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(const MapValue &other);
  /// Move assignment.
  /// @param other Object to move.
  /// @return A reference to this object.
  MapValue &operator=(MapValue &&other) noexcept;
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(int8_t val);
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(uint8_t val);
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(int16_t val);
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(uint16_t val);
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(int32_t val);
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(uint32_t val);
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(int64_t val);
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(uint64_t val);
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(float val);
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(double val);
  /// Value assignment. This changes the @c Type as required.
  /// @param val Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(bool val);
  /// Value assignment. This changes the @c Type as required.
  /// @param string Value to assign.
  /// @return A reference to this object.
  MapValue &operator=(const char *string);

  /// Destructor.
  ~MapValue();

  /// Clear the value and release the key name, making @c isValid() false and @c name() empty.
  void clear();

  /// Clear just the value, leaving the name unchanged. @c isValid() becomes false.
  void clearValue();

  /// Is the @c MapValue valid. This checks only the type. The entry may have a (dangling) name.
  /// @return True if the value is of a valid type.
  inline bool isValid() const { return type_ != kTypeNone; }

  /// Retrieve the value name/key.
  /// @return The value name.
  const char *name() const;

  /// Set the value name/key, replacing the existing name. Does not change the results of @c isValid().
  /// @param name The name to set.
  void setName(const char *name);

  /// Provide access to the internal name pointer. For internal use.
  /// @return The name key pointer.
  inline const void *namePtr() const { return name_; }

  /// Get the @c MapValue type.
  /// @return The value @c Type.
  inline Type type() const { return type_; }

  /// Compare the name of @p other against this @c name().
  /// @param other The @c MapValue to compare the name of.
  /// @return True if the names match or are othe empty/null or any combination of empty/null.
  bool namesEqual(const MapValue &other) const;

  /// Cast conversion. See casting notes in class description.
  /// @return The value as a signed byte.
  explicit operator int8_t() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as an unsigned byte.
  explicit operator uint8_t() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as a signed 16-bit integer.
  explicit operator int16_t() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as an unsigned 16-bit integer.
  explicit operator uint16_t() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as a signed 32-bit integer.
  explicit operator int32_t() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as an unsigned 32-bit integer.
  explicit operator uint32_t() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as a signed 64-bit integer.
  explicit operator int64_t() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as an unsigned 64-bit integer.
  explicit operator uint64_t() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as a single precision float.
  explicit operator float() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as a double precision float.
  explicit operator double() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as a boolean value.
  explicit operator bool() const;
  /// Cast conversion. See casting notes in class description.
  /// @return The value as a string.
  explicit operator const char *() const;

  /// Return a @c MapValue with type @c kString containing a string representation of
  /// this item's value.
  /// @return A string conversion of this @c MapValue.
  MapValue toStringValue() const;

  /// Equality comparison. Equality requires:
  /// - Same @c Type (including @c kTypeNone).
  /// - Same or empty/null names.
  /// - Equal values.
  /// @param other Item to compare.
  /// @return True if this equals @p other as described above.
  bool operator==(const MapValue &other) const;

  /// Inequality. Negation of ==.
  /// @param other Item to compare.
  /// @return False if this equals @p other as described in the == operator.
  bool operator!=(const MapValue &other) const;

  /// Less than operator for sorting. The result is entirely based on the names
  /// and the result of a less than comparison on the names.
  bool operator<(const MapValue &other) const;

  /// Comparison function for sorting list of @c MapValue objects.
  /// @param a The first value.
  /// @param b The second value.
  /// @return True when @p a is less than @p b.
  static bool sortCompare(const MapValue &a, const MapValue &b);

private:
  /// Internal type cast.
  /// @tparam T Target type.
  /// @return The converted value.
  template <typename T>
  T value() const;

  void *name_ = nullptr;  ///< Name member. Internal type is hidden to prevent exposure in the header.
  union
  {
    int8_t i8;
    uint8_t u8;
    int16_t i16;
    uint16_t u16;
    int32_t i32;
    uint32_t u32;
    int64_t i64;
    uint64_t u64;
    float f32;
    double f64;
    bool b;
    char *str;
  } value_{};              ///< Value member.
  Type type_ = kTypeNone;  ///< Type member.
};


struct MapInfoDetail;

/// A data store used to store meta data about an @c Octree. This includes
/// information about the tree depth. The structure can contain any data type
/// supported by @c MapValue. Some keys are reserved for use by the @c Octree
/// implementation, while others may be added by wrapping data structures such
/// as @c Region and @c OccupancyMap.
class ohm_API MapInfo
{
public:
  /// Create an empty @c MapInfo data store.
  MapInfo();

  /// Create @c MapInfo as an exact copy of @p other.
  /// @param other Data to copy.
  MapInfo(const MapInfo &other);

  /// R-value reference constructor, taking ownership of data in @c other and invalidating @c other.
  /// @param other Data to take ownership of.
  MapInfo(MapInfo &&other) noexcept;

  MapInfo &operator=(const MapInfo &other);
  MapInfo &operator=(MapInfo &&other) noexcept;

  /// Destructor.
  ~MapInfo();

  /// Set a value in the data store. Replaces existing data with a matching key.
  /// @param value Data value to set.
  void set(const MapValue &value);

  /// Get a value from the data store.
  /// @param name The key of the data to retrieve.
  /// @param default_value The value to return if the key @p name is not found.
  /// @return The existing value or an empty/invalid @c MapValue when @p name is not matched.
  MapValue get(const char *name, MapValue default_value = MapValue()) const;

  /// Remove the data entry associated with @p name, removing it from the tree.
  /// Does nothing if there is no such key.
  /// @param name The key of the value to remove.
  /// @return True if there was an item matching @p name.
  bool remove(const char *name);

  /// Remove all data entries.
  void clear();

  /// Extracts values from the info structure, or fetches the value count.
  ///
  /// @param values Array to extract into or null to query number of values.
  /// @param max_count Element capacity of @p values, or zero to query number of values.
  /// @param offset Offset the extraction by this amount. That is, skip the first N entries
  ///     where N = @p offset.
  /// @return The number of items added to values. If @p values is null or @p max_count is
  ///   zero, returns the number of values available.
  unsigned extract(MapValue *values, unsigned max_count, unsigned offset = 0) const;

private:
  std::unique_ptr<MapInfoDetail> imp_;  ///< Data implementation.
};
}  // namespace ohm

#endif  // MAPINFO_H
