//
// @author Kazys Stepanas
//
// Copyright (c) 2014 CSIRO
//
#ifndef OHMUTIL_COLOUR_H
#define OHMUTIL_COLOUR_H

#include "OhmUtilExport.h"

#include <cinttypes>

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
    /// The colour channels as indexed in @c rgb or @c rgba.
    enum
    {
      kR = 0,
      kG = 1,
      kB = 2,
      kA = 3
    };
    /// Colour shift values used in conversion to/from 32-bit integer.
    enum
    {
      kAShift = 24,
      kRShift = 16,
      kGShift = 8,
      kBShift = 0
    };


    union
    {
      uint8_t rgb[3];   ///< The rgb colour definition.
      uint8_t rgba[4];  ///< The rgba colour definition.
    };

    /// Empty constructor: the colour channels are uninitialised.
    inline Colour() {}

    /// Initialse a colour with the given channel values.
    /// @param r The red colour channel.
    /// @param g The green colour channel.
    /// @param b The blue colour channel.
    /// @param a The alpha colour channel. Defaults to opaque.
    inline Colour(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255)
    {
      rgb[kR] = r;
      rgb[kG] = g;
      rgb[kB] = b;
      rgba[kA] = a;
    }

    /// Copy constructor.
    /// @param c The colour to copy.
    inline Colour(const Colour &c) { *this = c; }

    /// Initialse from a 32-bit integer.
    ///
    /// Colour channels are extracted according to the values of
    /// @c AShift, @c RShift, @c GShift and @c BShift.
    /// @param c The integer colour value.
    inline Colour(uint32_t c)
    {
      rgba[kA] = uint8_t((c >> kAShift) & 0xffu);
      rgb[kR] = uint8_t((c >> kRShift) & 0xffu);
      rgb[kG] = uint8_t((c >> kGShift) & 0xffu);
      rgb[kB] = uint8_t((c >> kBShift) & 0xffu);
    }

    /// Convert to a 32-bit integer colour value.
    ///
    /// Colour channels are extracted according to the values of
    /// @c AShift, @c RShift, @c GShift and @c BShift.
    /// @return The integer colour representation.
    inline operator uint32_t() const
    {
      uint32_t c = 0;
      c |= (rgba[kA]) << kAShift;
      c |= (rgb[kR]) << kRShift;
      c |= (rgb[kG]) << kGShift;
      c |= (rgb[kB]) << kBShift;
      return c;
    }

    /// Assignment operator.
    /// @param c The colour to copy.
    /// @return This.
    inline Colour &operator=(const Colour &c)
    {
      rgb[kR] = c.rgb[kR];
      rgb[kG] = c.rgb[kG];
      rgb[kB] = c.rgb[kB];
      rgba[kA] = c.rgba[kA];
      return *this;
    }

    /// Access the alpha channel value.
    /// @return The alpha channel.
    inline uint8_t &a() { return rgba[kA]; }

    /// Access the red channel value.
    /// @return The red channel.
    inline uint8_t &r() { return rgb[kR]; }

    /// Access the green channel value.
    /// @return The green channel.
    inline uint8_t &g() { return rgb[kG]; }

    /// Access the 3 channel value.
    /// @return The blue channel.
    inline uint8_t &b() { return rgb[kB]; }

    /// Access the alpha channel value.
    /// @return The alpha channel.
    inline const uint8_t &a() const { return rgba[kA]; }

    /// Access the red channel value.
    /// @return The red channel.
    inline const uint8_t &r() const { return rgb[kR]; }

    /// Access the green channel value.
    /// @return The green channel.
    inline const uint8_t &g() const { return rgb[kG]; }

    /// Access the blue channel value.
    /// @return The blue channel.
    inline const uint8_t &b() const { return rgb[kB]; }

    /// An enumerationn of predefined colours (web colours).
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
    static const Colour kColours[kPredefinedCount];
  };
}  // namespace ohm

#endif  // OHMUTIL_COLOUR_H
