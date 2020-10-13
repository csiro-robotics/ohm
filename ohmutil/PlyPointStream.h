// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTILPLYPOINTSTREAM_H
#define OHMUTILPLYPOINTSTREAM_H

#include "OhmUtilExport.h"

#include <glm/fwd.hpp>

#include <cinttypes>
#include <ostream>
#include <string>
#include <vector>

namespace ohm
{
  /// A utility class for writing out a point cloud to PLY format. The cloud is written in a progressive, streaming
  /// fashion so has low memory overhead and can handle large clouds. This relies on using a @c std::ostream which
  /// supports seeking in order to patch the number of points in the correct location on completion.
  ///
  /// This only supports writing point clouds and does not support any polygonal data.
  ///
  /// Usage:
  /// - Open a file output stream.
  /// - Create a @c PlyPointStream with the desired properties and the file stream
  /// - For each point
  ///   - call @c setPointPosition() and additional @c setProperty() functions as required
  ///   - call @c writePoint()
  /// - call @c close()
  ///
  /// @code
  /// void writePly(const std::string &filename, const std::vector<glm::dvec3> &points)
  /// {
  ///   std::ofstream out(filename.c_str());
  ///   using Property = ohm::PlyPointStream::Property;
  ///   using Type = ohm::PlyPointStream::Type;
  ///   ohm::PlyPointStream ply({ Property{ "x", Type::kFloat64 }, Property{ "y", Type::kFloat64 },
  ///                             Property{ "z", Type::kFloat64 },
  ///                           out);
  ///   for (const auto &point : points)
  ///   {
  ///     ply.setPointPosition(point);
  ///     ply.writePoint();
  ///   }
  ///   ply.close();
  /// }
  /// @endcode
  class ohmutil_API PlyPointStream
  {
  public:
    /// Defines the available ply property types.
    enum class ohmutil_API Type : unsigned
    {
      kNull,
      kInt8,
      kUInt8,
      kInt16,
      kUInt16,
      kInt32,
      kUInt32,
      kFloat32,
      kFloat64
    };

    /// Represents a point property in the ply file. At the very least, "x", "y", "z" properties of type @c kFloat64
    /// are expected.
    struct ohmutil_API Property
    {
      std::string name;
      Type type;
    };

    /// Create a stream with the given properties. @c open() to be called later.
    /// @param properties The point properties to write. Cannot be changed after construction.
    PlyPointStream(const std::vector<Property> &properties);
    /// Create a stream with the given properties writing to the given stream. The header is written immediately.
    /// @param properties The point properties to write. Cannot be changed after construction.
    /// @param out The output stream to write to. Must be seekable. Must outlive this object.
    PlyPointStream(const std::vector<Property> &properties, std::ostream &out);
    /// Destructor. Ensures the point count is finalised.
    ~PlyPointStream();

    /// Open ply writing with the given stream. Ensures any current stream is first closed.
    /// @param out The output stream to write to. Must be seekable. Must outlive this object.
    void open(std::ostream &out);
    /// Close the current stream (if open)
    /// @return True if open and the point count is successfully finalised (using seek).
    bool close();

    /// Is the object currently open with an output stream?
    bool isOpen() const;

    /// Query the number of points written.
    /// @return The nubmer of points written.
    inline uint64_t pointCount() const { return point_count_; }

    /// Set the position property values "x", "y", "z" for the current point.
    /// @param pos The position values to write.
    /// @return True if the properties exist and are of the correct type.
    bool setPointPosition(const glm::dvec3 &pos);
    /// Set the "time" value for the current point.
    /// @param time The time value to write.
    /// @return True if the property exists and is of the correct type.
    bool setPointTimestamp(double time);
    /// Set the position property values "nx", "ny", "nz" for the current point.
    /// @param normal The normal values to write.
    /// @return True if the properties exist and are of the correct type.
    bool setPointNormal(const glm::dvec3 &normal);

    /// Set the value for the property @p name for the current point.
    /// @param name The property name.
    /// @param value Value to write - must match the expected type.
    /// @return True if the property exists and is of the correct type.
    bool setProperty(const std::string &name, int8_t value);
    /// @overload
    bool setProperty(const std::string &name, uint8_t value);
    /// @overload
    bool setProperty(const std::string &name, int16_t value);
    /// @overload
    bool setProperty(const std::string &name, uint16_t value);
    /// @overload
    bool setProperty(const std::string &name, int32_t value);
    /// @overload
    bool setProperty(const std::string &name, uint32_t value);
    /// @overload
    bool setProperty(const std::string &name, float value);
    /// @overload
    bool setProperty(const std::string &name, double value);

    /// Write the current collected point data. Values which have not been set will retain their previous value.
    void writePoint();

    /// Query the ply string name for @p type . This is written to the ply file as the property type.
    /// @param type The type to query.
    /// @return The ply type name for @p type
    static std::string typeName(Type type);

  private:
    union Value
    {
      int8_t i8;
      uint8_t u8;
      int16_t i16;
      uint16_t u16;
      int32_t i32;
      uint32_t u32;
      float f32;
      double f64 = 0;
    };

    /// Internal helper for setting the a property value. Checks type equality.
    /// @param name The property name.
    /// @param value Value to write - must match the expected type.
    /// @return True if the property exists and is of the correct type.
    template <typename T>
    bool setPropertyT(const std::string &name, T value);

    /// Internal helper for setting the a property value after validation.
    /// @param dst The value to write to.
    /// @param value Value to write.
    void setValue(Value *dst, int8_t value);
    /// @overload
    void setValue(Value *dst, uint8_t value);
    /// @overload
    void setValue(Value *dst, int16_t value);
    /// @overload
    void setValue(Value *dst, uint16_t value);
    /// @overload
    void setValue(Value *dst, int32_t value);
    /// @overload
    void setValue(Value *dst, uint32_t value);
    /// @overload
    void setValue(Value *dst, float value);
    /// @overload
    void setValue(Value *dst, double value);

    /// Write the ply header with a placeholder point count of 0.
    void writeHeader();

    /// Rewind the stream to the point count and write the correct value. Additional padding will be consumed with a
    /// ply comment.
    /// @return True on success, false on failure to rewind, padding size overflow or when not open.
    bool finalisePointCount();

    std::ostream *out_{ nullptr };
    std::vector<Property> properties_;
    std::vector<Value> values_;
    uint64_t point_count_ = 0;
    std::ostream::pos_type point_count_pos_ = -1;
  };
}  // namespace ohm

#endif  // OHMUTILPLYPOINTSTREAM_H
