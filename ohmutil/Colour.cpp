//
// @author Kazys Stepanas
//
// Copyright (c) 2014 CSIRO
//
#include "Colour.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
const float kSectorSizeDegrees = 60.0f;
const float kCircleDegrees = 360.0f;
}  // namespace

namespace ohm
{
void colour::hsvToRgb(float &r, float &g, float &b, float h, float s, float v)
{
  // if (s == 0)
  //{
  //  // achromatic (grey)
  //  r = g = b = v;
  //  return;
  //}
  const float h_sector = h / kSectorSizeDegrees;  // sector 0 to 5
  const int sector_index = int(std::min<float>(std::max<float>(0.0f, std::floor(h_sector)), 5.0f));
  const float f = h_sector - float(sector_index);
  const float p = v * (1 - s);
  const float q = v * (1 - s * f);
  const float t = v * (1 - s * (1 - f));

  static const std::array<int, 6> v_index = { 0, 1, 1, 2, 2, 0 };
  static const std::array<int, 6> p_index = { 2, 2, 0, 0, 1, 1 };
  static const std::array<int, 6> q_index = { 3, 0, 3, 1, 3, 2 };
  static const std::array<int, 6> t_index = { 1, 3, 2, 3, 0, 3 };

  std::array<float, 4> rgb = { 0 };
  rgb[v_index[sector_index]] = v;
  rgb[p_index[sector_index]] = p;
  rgb[q_index[sector_index]] = q;
  rgb[t_index[sector_index]] = t;

  // Handle achromatic here by testing s inline.
  r = (s != 0) ? rgb[0] : v;
  g = (s != 0) ? rgb[1] : v;
  b = (s != 0) ? rgb[2] : v;
}


void colour::hsvToRgb(uint8_t &r, uint8_t &g, uint8_t &b, float h, float s, float v)
{
  float rf;
  float gf;
  float bf;
  hsvToRgb(rf, gf, bf, h, s, v);
  r = uint8_t(rf * Colour::kMaxByteF);
  g = uint8_t(gf * Colour::kMaxByteF);
  b = uint8_t(bf * Colour::kMaxByteF);
}


void colour::rgbToHsv(float &h, float &s, float &v, float r, float g, float b)
{
  const float c_min = std::min<float>(r, std::min<float>(g, b));
  const float c_max = std::max<float>(r, std::max<float>(g, b));

  const float delta = c_max - c_min;


  if (!c_max)
  {
    // Black. s = 0, v is undefined.
    s = v = h = 0;
    return;
  }

  v = c_max;
  s = delta / c_max;

  if (r == c_max)
  {
    // Yellow to magenta.
    h = (g - b) / delta;
  }
  else if (g == c_max)
  {
    // Cyan to yellow
    h = 2 + (b - r) / delta;
  }
  else
  {
    // Magenta to cyan
    h = 4 + (r - g) / delta;
  }

  h *= kSectorSizeDegrees;

  h = (h < 0) ? h + kCircleDegrees : h;
  h = (h > kCircleDegrees) ? h - kCircleDegrees : h;
}


void colour::rgbToHsv(float &h, float &s, float &v, uint8_t r, uint8_t g, uint8_t b)
{
  rgbToHsv(h, s, v, float(r) / 255.0f, float(g) / 255.0f, float(b) / 255.0f);
}


Colour Colour::adjust(float factor) const
{
  float h, s, v;
  Colour c;
  ohm::colour::rgbToHsv(h, s, v, rf(), gf(), bf());
  v = std::max(0.0f, std::min(v * factor, 1.0f));
  c.rgba[kA] = this->rgba[kA];
  ohm::colour::hsvToRgb(c.rgba[kR], c.rgba[kG], c.rgba[kB], h, s, v);
  return c;
}

Colour Colour::lerp(const Colour &from, const Colour &to, float factor)
{
  // Convert to hsv space for the lerp.
  float from_h;
  float from_s;
  float from_v;
  colour::rgbToHsv(from_h, from_s, from_v, from.r(), from.g(), from.b());

  float to_h;
  float to_s;
  float to_v;
  colour::rgbToHsv(to_h, to_s, to_v, to.r(), to.g(), to.b());

  const float max_h = kCircleDegrees;
  if (std::abs(to_h - from_h) > 0.5f * max_h)
  {
    if (to_h < from_h)
    {
      to_h += max_h;
    }
    else
    {
      from_h += max_h;
    }
  }

  const float h = std::max(0.0f, std::min(from_h + (to_h - from_h) * factor, max_h));

  // Lerp s and v the easy way.
  const float s = std::max(0.0f, std::min(from_s + (to_s - from_s) * factor, 1.0f));
  const float v = std::max(0.0f, std::min(from_v + (to_v - from_v) * factor, 1.0f));

  Colour c{};
  colour::hsvToRgb(c.rgba[0], c.rgba[1], c.rgba[2], h, s, v);

  // Finally lerp alpha
  const float from_a = from.af();
  const float to_a = to.af();

  const float alpha = std::max(0.0f, std::min(from_a + (to_a - from_a) * factor, 1.0f));

  c.rgba[3] = static_cast<uint8_t>(alpha * std::numeric_limits<uint8_t>::max());

  return c;
}


// Lint(KS): small static allocation should be fine.
// NOLINTNEXTLINE(cert-err58-cpp)
const std::array<Colour, Colour::kPredefinedCount> Colour::kColours =  //
  {
    Colour(220, 220, 220),  //
    Colour(211, 211, 211),  //
    Colour(192, 192, 192),  //
    Colour(169, 169, 169),  //
    Colour(128, 128, 128),  //
    Colour(105, 105, 105),  //
    Colour(119, 136, 153),  //
    Colour(112, 128, 144),  //
    Colour(47, 79, 79),     //
    Colour(0, 0, 0),        //
    Colour(255, 255, 255),  //
    Colour(255, 250, 250),  //
    Colour(240, 255, 240),  //
    Colour(245, 255, 250),  //
    Colour(240, 255, 255),  //
    Colour(240, 248, 255),  //
    Colour(248, 248, 255),  //
    Colour(245, 245, 245),  //
    Colour(255, 245, 238),  //
    Colour(245, 245, 220),  //
    Colour(253, 245, 230),  //
    Colour(255, 250, 240),  //
    Colour(255, 255, 240),  //
    Colour(250, 235, 215),  //
    Colour(250, 240, 230),  //
    Colour(255, 240, 245),  //
    Colour(255, 228, 225),  //
    Colour(255, 192, 203),  //
    Colour(255, 182, 193),  //
    Colour(255, 105, 180),  //
    Colour(255, 20, 147),   //
    Colour(219, 112, 147),  //
    Colour(199, 21, 133),   //
    Colour(255, 160, 122),  //
    Colour(250, 128, 114),  //
    Colour(233, 150, 122),  //
    Colour(240, 128, 128),  //
    Colour(205, 92, 92),    //
    Colour(220, 20, 60),    //
    Colour(178, 34, 34),    //
    Colour(139, 0, 0),      //
    Colour(255, 0, 0),      //
    Colour(255, 69, 0),     //
    Colour(255, 99, 71),    //
    Colour(255, 127, 80),   //
    Colour(255, 140, 0),    //
    Colour(255, 165, 0),    //
    Colour(255, 255, 0),    //
    Colour(255, 255, 224),  //
    Colour(255, 250, 205),  //
    Colour(250, 250, 210),  //
    Colour(255, 239, 213),  //
    Colour(255, 228, 181),  //
    Colour(255, 218, 185),  //
    Colour(238, 232, 170),  //
    Colour(240, 230, 140),  //
    Colour(189, 183, 107),  //
    Colour(255, 215, 0),    //
    Colour(255, 248, 220),  //
    Colour(255, 235, 205),  //
    Colour(255, 228, 196),  //
    Colour(255, 222, 173),  //
    Colour(245, 222, 179),  //
    Colour(222, 184, 135),  //
    Colour(210, 180, 140),  //
    Colour(188, 143, 143),  //
    Colour(244, 164, 96),   //
    Colour(218, 165, 32),   //
    Colour(184, 134, 11),   //
    Colour(205, 133, 63),   //
    Colour(210, 105, 30),   //
    Colour(139, 69, 19),    //
    Colour(160, 82, 45),    //
    Colour(165, 42, 42),    //
    Colour(128, 0, 0),      //
    Colour(85, 107, 47),    //
    Colour(128, 128, 0),    //
    Colour(107, 142, 35),   //
    Colour(154, 205, 50),   //
    Colour(50, 205, 50),    //
    Colour(0, 255, 0),      //
    Colour(124, 252, 0),    //
    Colour(127, 255, 0),    //
    Colour(173, 255, 47),   //
    Colour(0, 255, 127),    //
    Colour(0, 250, 154),    //
    Colour(144, 238, 144),  //
    Colour(152, 251, 152),  //
    Colour(143, 188, 143),  //
    Colour(60, 179, 113),   //
    Colour(46, 139, 87),    //
    Colour(34, 139, 34),    //
    Colour(0, 128, 0),      //
    Colour(0, 100, 0),      //
    Colour(102, 205, 170),  //
    Colour(0, 255, 255),    //
    Colour(0, 255, 255),    //
    Colour(224, 255, 255),  //
    Colour(175, 238, 238),  //
    Colour(127, 255, 212),  //
    Colour(64, 224, 208),   //
    Colour(72, 209, 204),   //
    Colour(0, 206, 209),    //
    Colour(32, 178, 170),   //
    Colour(95, 158, 160),   //
    Colour(0, 139, 139),    //
    Colour(0, 128, 128),    //
    Colour(176, 196, 222),  //
    Colour(176, 224, 230),  //
    Colour(173, 216, 230),  //
    Colour(135, 206, 235),  //
    Colour(135, 206, 250),  //
    Colour(0, 191, 255),    //
    Colour(30, 144, 255),   //
    Colour(100, 149, 237),  //
    Colour(70, 130, 180),   //
    Colour(65, 105, 225),   //
    Colour(0, 0, 255),      //
    Colour(0, 0, 205),      //
    Colour(0, 0, 139),      //
    Colour(0, 0, 128),      //
    Colour(25, 25, 112),    //
    Colour(230, 230, 250),  //
    Colour(216, 191, 216),  //
    Colour(221, 160, 221),  //
    Colour(238, 130, 238),  //
    Colour(218, 112, 214),  //
    Colour(255, 0, 255),    //
    Colour(255, 0, 255),    //
    Colour(186, 85, 211),   //
    Colour(147, 112, 219),  //
    Colour(138, 43, 226),   //
    Colour(148, 0, 211),    //
    Colour(153, 50, 204),   //
    Colour(139, 0, 139),    //
    Colour(128, 0, 128),    //
    Colour(75, 0, 130),     //
    Colour(72, 61, 139),    //
    Colour(106, 90, 205),   //
    Colour(123, 104, 238),  //
  };
}  // namespace ohm
