//
// @author Kazys Stepanas
//
// Copyright (c) 2014 CSIRO
//
#ifndef COLOUR_H
#define COLOUR_H

#include "ohmutilexport.h"

#include <cinttypes>

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
  void ohmutil_API hsvToRgb(float &r, float &g, float &b,
                            const float h, const float s, const float v);

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
  void ohmutil_API hsvToRgb(uint8_t &r, uint8_t &g, uint8_t &b,
                            const float h, const float s, const float v);

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
  void rgbToHsv(float &h, float &s, float &v,
                const float r, const float g, const float b);

  /// Converts from RGB to HSV colour.
  ///
  /// @param[out] h The calculated hue channel.
  /// @param[out] s The calculated saturation channel.
  /// @param[out] v The calculated value channel.
  /// @param r The red colour channel.
  /// @param g The green colour channel.
  /// @param b The blue colour channel.
  void rgbToHsv(float &h, float &s, float &v,
                uint8_t r, uint8_t g, uint8_t b);
}

/// Defines an RGB or RGBA colour value. Each colour channel is represented by an unsigned byte.
struct ohmutil_API Colour
{
  /// The colour channels as indexed in @c rgb or @c rgba.
  enum { R = 0, G = 1, B = 2, A = 3 };
  /// Colour shift values used in conversion to/from 32-bit integer.
  enum { ASHIFT = 24, RSHIFT = 16, GSHIFT = 8, BSHIFT = 0 };


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
    rgb[R] = r;
    rgb[G] = g;
    rgb[B] = b;
    rgba[A] = a;
  }

  /// Copy constructor.
  /// @param c The colour to copy.
  inline Colour(const Colour &c) { *this = c; }

  /// Initialse from a 32-bit integer.
  ///
  /// Colour channels are extracted according to the values of
  /// @c ASHIFT, @c RSHIFT, @c GSHIFT and @c BSHIFT.
  /// @param c The integer colour value.
  inline Colour(uint32_t c)
  {
    rgba[A] = uint8_t((c >> ASHIFT) & 0xffu);
    rgb[R] = uint8_t((c >> RSHIFT) & 0xffu);
    rgb[G] = uint8_t((c >> GSHIFT) & 0xffu);
    rgb[B] = uint8_t((c >> BSHIFT) & 0xffu);
  }

  /// Convert to a 32-bit integer colour value.
  ///
  /// Colour channels are extracted according to the values of
  /// @c ASHIFT, @c RSHIFT, @c GSHIFT and @c BSHIFT.
  /// @return The integer colour representation.
  inline operator uint32_t () const
  {
    uint32_t c = 0;
    c |= (rgba[A]) << ASHIFT;
    c |= (rgb[R]) << RSHIFT;
    c |= (rgb[G]) << GSHIFT;
    c |= (rgb[B]) << BSHIFT;
    return c;
  }

  /// Assignment operator.
  /// @param c The colour to copy.
  /// @return This.
  inline Colour &operator = (const Colour &c) { rgb[R] = c.rgb[R]; rgb[G] = c.rgb[G]; rgb[B] = c.rgb[B]; rgba[A] = c.rgba[A]; return *this; }

  /// Access the alpha channel value.
  /// @return The alpha channel.
  inline uint8_t &a() { return rgba[A]; }

  /// Access the red channel value.
  /// @return The red channel.
  inline uint8_t &r() { return rgb[R]; }

  /// Access the green channel value.
  /// @return The green channel.
  inline uint8_t &g() { return rgb[G]; }

  /// Access the 3 channel value.
  /// @return The blue channel.
  inline uint8_t &b() { return rgb[B]; }

  /// Access the alpha channel value.
  /// @return The alpha channel.
  inline const uint8_t &a() const { return rgba[A]; }

  /// Access the red channel value.
  /// @return The red channel.
  inline const uint8_t &r() const { return rgb[R]; }

  /// Access the green channel value.
  /// @return The green channel.
  inline const uint8_t &g() const { return rgb[G]; }

  /// Access the blue channel value.
  /// @return The blue channel.
  inline const uint8_t &b() const { return rgb[B]; }

  /// An enumerationn of predefined colours (web colours).
  enum Names
  {
    // Blacks.
    Gainsboro,
    LightGrey,
    Silver,
    DarkGray,
    Gray,
    DimGray,
    LightSlateGray,
    SlateGray,
    DarkSlateGray,
    Black,

    // Whites
    White,
    Snow,
    Honeydew,
    MintCream,
    Azure,
    AliceBlue,
    GhostWhite,
    WhiteSmoke,
    Seashell,
    Beige,
    OldLace,
    FloralWhite,
    Ivory,
    AntiqueWhite,
    Linen,
    LavenderBlush,
    MistyRose,

    // Pinks
    Pink,
    LightPink,
    HotPink,
    DeepPink,
    PaleVioletRed,
    MediumVioletRed,

    // Reds
    LightSalmon,
    Salmon,
    DarkSalmon,
    LightCoral,
    IndianRed,
    Crimson,
    FireBrick,
    DarkRed,
    Red,

    // Oranges
    OrangeRed,
    Tomato,
    Coral,
    DarkOrange,
    Orange,

    // Yellows
    Yellow,
    LightYellow,
    LemonChiffon,
    LightGoldenrodYellow,
    PapayaWhip,
    Moccasin,
    PeachPuff,
    PaleGoldenrod,
    Khaki,
    DarkKhaki,
    Gold,

    // Browns
    Cornsilk,
    BlanchedAlmond,
    Bisque,
    NavajoWhite,
    Wheat,
    BurlyWood,
    Tan,
    RosyBrown,
    SandyBrown,
    Goldenrod,
    DarkGoldenrod,
    Peru,
    Chocolate,
    SaddleBrown,
    Sienna,
    Brown,
    Maroon,

    // Greens
    DarkOliveGreen,
    Olive,
    OliveDrab,
    YellowGreen,
    LimeGreen,
    Lime,
    LawnGreen,
    Chartreuse,
    GreenYellow,
    SpringGreen,
    MediumSpringGreen,
    LightGreen,
    PaleGreen,
    DarkSeaGreen,
    MediumSeaGreen,
    SeaGreen,
    ForestGreen,
    Green,
    DarkGreen,

    // Cyans
    MediumAquamarine,
    Aqua,
    Cyan,
    LightCyan,
    PaleTurquoise,
    Aquamarine,
    Turquoise,
    MediumTurquoise,
    DarkTurquoise,
    LightSeaGreen,
    CadetBlue,
    DarkCyan,
    Teal,

    // Blues
    LightSteelBlue,
    PowderBlue,
    LightBlue,
    SkyBlue,
    LightSkyBlue,
    DeepSkyBlue,
    DodgerBlue,
    CornflowerBlue,
    SteelBlue,
    RoyalBlue,
    Blue,
    MediumBlue,
    DarkBlue,
    Navy,
    MidnightBlue,

    // Purples
    Lavender,
    Thistle,
    Plum,
    Violet,
    Orchid,
    Fuchsia,
    Magenta,
    MediumOrchid,
    MediumPurple,
    BlueViolet,
    DarkViolet,
    DarkOrchid,
    DarkMagenta,
    Purple,
    Indigo,
    DarkSlateBlue,
    SlateBlue,
    MediumSlateBlue,

    PredefinedCount
  };

  /// Predefined colours (web colours).
  static Colour Colours[PredefinedCount];
};

#endif // COLOUR_H
