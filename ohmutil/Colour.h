//
// @author Kazys Stepanas
//
// Copyright (c) 2014 CSIRO
//
#ifndef OHMUTIL_COLOUR_H
#define OHMUTIL_COLOUR_H

#include "OhmUtilExport.h"

#include <array>
#include <cinttypes>
#include <limits>

namespace ohm
{
/// Colour conversion utility functions.
namespace colour
{
/// Converts from HSV to RGB colour.
///
/// All floating points values are expected to be in the range [0, 1], except for
/// @p h (see below), or the results are undefined.
///
/// @param[out] r The calculated red colour channel.
/// @param[out] g The calculated green colour channel.
/// @param[out] b The calculated blue colour channel.
/// @param h The colour hue channel [0, 360].
/// @param s The colour saturation channel.
/// @param v The colour value channel.
void ohmutil_API hsvToRgb(float &r, float &g, float &b, float h, float s, float v);

/// Converts from HSV to RGB colour.
///
/// All floating points values are expected to be in the range [0, 1], except for
/// @p h (see below), or the results are undefined.
///
/// @param[out] r The calculated red colour channel.
/// @param[out] g The calculated green colour channel.
/// @param[out] b The calculated blue colour channel.
/// @param h The colour hue channel [0, 360].
/// @param s The colour saturation channel.
/// @param v The colour value channel.
void ohmutil_API hsvToRgb(uint8_t &r, uint8_t &g, uint8_t &b, float h, float s, float v);

/// Converts from RGB to HSV colour.
///
/// All floating points values are expected to be in the range [0, 1], or
/// the results are undefined.
///
/// @param[out] h The calculated hue channel.
/// @param[out] s The calculated saturation channel.
/// @param[out] v The calculated value channel.
/// @param r The red colour channel.
/// @param g The green colour channel.
/// @param b The blue colour channel.
void rgbToHsv(float &h, float &s, float &v, float r, float g, float b);

/// Converts from RGB to HSV colour.
///
/// @param[out] h The calculated hue channel.
/// @param[out] s The calculated saturation channel.
/// @param[out] v The calculated value channel.
/// @param r The red colour channel.
/// @param g The green colour channel.
/// @param b The blue colour channel.
void rgbToHsv(float &h, float &s, float &v, uint8_t r, uint8_t g, uint8_t b);
}  // namespace colour

/// Defines an RGB or RGBA colour value. Each colour channel is represented by an unsigned byte.
struct ohmutil_API Colour
{
  /// The colour channels as indexed in @c rgba.
  enum
  {
    kR = 0,
    kG = 1,
    kB = 2,
    kA = 3
  };
  /// Colour shift values used in conversion to/from 32-bit integer.
  enum : unsigned
  {
    kAShift = 24,
    kRShift = 16,
    kGShift = 8,
    kBShift = 0
  };

  /// Maximum value of a single unsigned byte expressed as a floating point number.
  static constexpr float kMaxByteF = 255.0f;

  /// The rgba colour definition.
  std::array<uint8_t, 4> rgba;

  /// Empty constructor: the colour channels are uninitialised.
  inline Colour() = default;

  /// Initialise a colour with the given channel values.
  /// @param r The red colour channel.
  /// @param g The green colour channel.
  /// @param b The blue colour channel.
  /// @param a The alpha colour channel. Defaults to opaque.
  inline Colour(
    uint8_t r, uint8_t g, uint8_t b,
    uint8_t a = std::numeric_limits<uint8_t>::max()) noexcept  // NOLINT(cppcoreguidelines-pro-type-member-init)
  {
    rgba[kR] = r;
    rgba[kG] = g;
    rgba[kB] = b;
    rgba[kA] = a;
  }

  /// Copy constructor.
  /// @param c The colour to copy.
  inline Colour(const Colour &c) { *this = c; }  // NOLINT(cppcoreguidelines-pro-type-member-init)

  /// Initialise from a 32-bit integer.
  ///
  /// Colour channels are extracted according to the values of
  /// @c AShift, @c RShift, @c GShift and @c BShift.
  /// @param c The integer colour value.
  inline Colour(uint32_t c) noexcept  // NOLINT
  {
    const unsigned byte_mask = 0xffu;
    rgba[kA] = uint8_t((c >> kAShift) & byte_mask);
    rgba[kR] = uint8_t((c >> kRShift) & byte_mask);
    rgba[kG] = uint8_t((c >> kGShift) & byte_mask);
    rgba[kB] = uint8_t((c >> kBShift) & byte_mask);
  }

  /// Initialise a colour with the given floating point channel values.
  /// @param r The red colour channel.
  /// @param g The green colour channel.
  /// @param b The blue colour channel.
  /// @param a The alpha colour channel. Defaults to opaque.
  inline static Colour fromRgbf(float r, float g, float b, float a = 1.0f)
  {
    return Colour(uint8_t(r * kMaxByteF), uint8_t(g * kMaxByteF), uint8_t(b * kMaxByteF), uint8_t(a * kMaxByteF));
  }

  /// Convert to a 32-bit integer colour value.
  ///
  /// Colour channels are extracted according to the values of
  /// @c AShift, @c RShift, @c GShift and @c BShift.
  /// @return The integer colour representation.
  inline explicit operator uint32_t() const noexcept
  {
    uint32_t c = 0;
    c |= unsigned(rgba[kA]) << kAShift;
    c |= unsigned(rgba[kR]) << kRShift;
    c |= unsigned(rgba[kG]) << kGShift;
    c |= unsigned(rgba[kB]) << kBShift;
    return c;
  }

  /// Assignment operator.
  /// @param c The colour to copy.
  /// @return This.
  inline Colour &operator=(const Colour &c) noexcept
  {
    rgba[kR] = c.rgba[kR];
    rgba[kG] = c.rgba[kG];
    rgba[kB] = c.rgba[kB];
    rgba[kA] = c.rgba[kA];
    return *this;
  }

  /// Equality check operator.
  /// @param other The object to compare against.
  /// @return True if the colours are equal.
  inline bool operator==(const Colour &other) const
  {
    return rgba[kR] == other.rgba[kR] && rgba[kG] == other.rgba[kG] && rgba[kB] == other.rgba[kB] &&
           rgba[kA] == other.rgba[kA];
  }

  /// Inequality check operator.
  /// @param other The object to compare against.
  /// @return True if the colours are equal.
  inline bool operator!=(const Colour &other) const { return !operator==(other); }

  /// Access the alpha channel value.
  /// @return The alpha channel.
  inline uint8_t &a() { return rgba[kA]; }

  /// Access the red channel value.
  /// @return The red channel.
  inline uint8_t &r() { return rgba[kR]; }

  /// Access the green channel value.
  /// @return The green channel.
  inline uint8_t &g() { return rgba[kG]; }

  /// Access the 3 channel value.
  /// @return The blue channel.
  inline uint8_t &b() { return rgba[kB]; }

  /// Access the alpha channel value.
  /// @return The alpha channel.
  inline const uint8_t &a() const { return rgba[kA]; }

  /// Access the red channel value.
  /// @return The red channel.
  inline const uint8_t &r() const { return rgba[kR]; }

  /// Access the green channel value.
  /// @return The green channel.
  inline const uint8_t &g() const { return rgba[kG]; }

  /// Access the blue channel value.
  /// @return The blue channel.
  inline const uint8_t &b() const { return rgba[kB]; }

  /// Access red channel as a float.
  /// @return Red channel as a float.
  inline float rf() const { return float(r()) / kMaxByteF; }

  /// Access green channel as a float.
  /// @return Green channel as a float.
  inline float gf() const { return float(g()) / kMaxByteF; }

  /// Access blue channel as a float.
  /// @return Blue channel as a float.
  inline float bf() const { return float(b()) / kMaxByteF; }

  /// Access alpha channel as a float.
  /// @return Alpha channel as a float.
  inline float af() const { return float(a()) / kMaxByteF; }

  /// Set red channel from a float.
  /// @param channel Channel value.
  inline void setRf(float channel) { rgba[kR] = uint8_t(channel * kMaxByteF); }

  /// Set green channel from a float.
  /// @param channel Channel value.
  inline void setGf(float channel) { rgba[kG] = uint8_t(channel * kMaxByteF); }

  /// Set blue channel from a float.
  /// @param channel Channel value.
  inline void setBf(float channel) { rgba[kB] = uint8_t(channel * kMaxByteF); }

  /// Set alpha channel from a float.
  /// @param channel Channel value.
  inline void setAf(float channel) { rgba[kA] = uint8_t(channel * kMaxByteF); }

  /// Lighten or darken a colour by @p factor.
  /// Works in HSV space, multiplying the V value by @p factor and clamping the result [0, 1].
  /// @return The adjusted colour.
  Colour adjust(float factor) const;

  /// Lighten the colour by 1.5
  /// @return A lighter colour.
  inline Colour lighten() const { return adjust(1.5f); }

  /// Darken the colour by 0.5
  /// @return A darker colour.
  inline Colour darken() const { return adjust(0.5f); }

  /// Interpolate from one colour to another.
  /// @param from Colour to start at.
  /// @param to Colour to target at.
  /// @param factor Interpolation factor: [0, 1] or behaviour is undefined.
  /// @return A colour between the given colours.
  static Colour lerp(const Colour &from, const Colour &to, float factor);

  /// An enumeration of predefined colours (web colours).
  enum Names
  {
    // Blacks.
    kGainsboro,
    kLightGrey,
    kSilver,
    kDarkGray,
    kGray,
    kDimGray,
    kLightSlateGray,
    kSlateGray,
    kDarkSlateGray,
    kBlack,

    // Whites
    kWhite,
    kSnow,
    kHoneydew,
    kMintCream,
    kAzure,
    kAliceBlue,
    kGhostWhite,
    kWhiteSmoke,
    kSeashell,
    kBeige,
    kOldLace,
    kFloralWhite,
    kIvory,
    kAntiqueWhite,
    kLinen,
    kLavenderBlush,
    kMistyRose,

    // Pinks
    kPink,
    kLightPink,
    kHotPink,
    kDeepPink,
    kPaleVioletRed,
    kMediumVioletRed,

    // Reds
    kLightSalmon,
    kSalmon,
    kDarkSalmon,
    kLightCoral,
    kIndianRed,
    kCrimson,
    kFireBrick,
    kDarkRed,
    kRed,

    // Oranges
    kOrangeRed,
    kTomato,
    kCoral,
    kDarkOrange,
    kOrange,

    // Yellows
    kYellow,
    kLightYellow,
    kLemonChiffon,
    kLightGoldenrodYellow,
    kPapayaWhip,
    kMoccasin,
    kPeachPuff,
    kPaleGoldenrod,
    kKhaki,
    kDarkKhaki,
    kGold,

    // Browns
    kCornsilk,
    kBlanchedAlmond,
    kBisque,
    kNavajoWhite,
    kWheat,
    kBurlyWood,
    kTan,
    kRosyBrown,
    kSandyBrown,
    kGoldenrod,
    kDarkGoldenrod,
    kPeru,
    kChocolate,
    kSaddleBrown,
    kSienna,
    kBrown,
    kMaroon,

    // Greens
    kDarkOliveGreen,
    kOlive,
    kOliveDrab,
    kYellowGreen,
    kLimeGreen,
    kLime,
    kLawnGreen,
    kChartreuse,
    kGreenYellow,
    kSpringGreen,
    kMediumSpringGreen,
    kLightGreen,
    kPaleGreen,
    kDarkSeaGreen,
    kMediumSeaGreen,
    kSeaGreen,
    kForestGreen,
    kGreen,
    kDarkGreen,

    // Cyans
    kMediumAquamarine,
    kAqua,
    kCyan,
    kLightCyan,
    kPaleTurquoise,
    kAquamarine,
    kTurquoise,
    kMediumTurquoise,
    kDarkTurquoise,
    kLightSeaGreen,
    kCadetBlue,
    kDarkCyan,
    kTeal,

    // Blues
    kLightSteelBlue,
    kPowderBlue,
    kLightBlue,
    kSkyBlue,
    kLightSkyBlue,
    kDeepSkyBlue,
    kDodgerBlue,
    kCornflowerBlue,
    kSteelBlue,
    kRoyalBlue,
    kBlue,
    kMediumBlue,
    kDarkBlue,
    kNavy,
    kMidnightBlue,

    // Purples
    kLavender,
    kThistle,
    kPlum,
    kViolet,
    kOrchid,
    kFuchsia,
    kMagenta,
    kMediumOrchid,
    kMediumPurple,
    kBlueViolet,
    kDarkViolet,
    kDarkOrchid,
    kDarkMagenta,
    kPurple,
    kIndigo,
    kDarkSlateBlue,
    kSlateBlue,
    kMediumSlateBlue,

    kPredefinedCount
  };

  /// Predefined colours (web colours).
  static const std::array<Colour, kPredefinedCount> kColours;
};  // namespace ohm
}  // namespace ohm

#endif  // OHMUTIL_COLOUR_H
