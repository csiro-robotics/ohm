// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include <gtest/gtest.h>

#include <ohm/Aabb.h>

#include <ohmutil/GlmStream.h>

#include <sstream>

using namespace ohm;

inline std::ostream &operator<<(std::ostream &out, const ohm::Aabb &aabb)
{
  out << "[ (" << aabb.minExtents().x << ',' << aabb.minExtents().y << ',' << aabb.minExtents().z << ") ("
      << aabb.maxExtents().x << ',' << aabb.maxExtents().y << ',' << aabb.maxExtents().z << ") ]";
  return out;
}


namespace
{
  void testOverlap(const glm::ivec3 &axis_range, const Aabb &reference_box, const Aabb &initial_box, bool overlap_test)
  {
    char sign_char[3];
    int axis[3];
    Aabb test_box;

    const double move_scale = (overlap_test) ? 1.0 : 2.0;
    glm::dvec3 move;

    for (int k = 0; k < axis_range[2]; ++k)
    {
      move = glm::dvec3(0);

      sign_char[2] = ' ';
      axis[2] = -1;
      if (axis_range[2] > 1)
      {
        axis[2] = k % 3;
        if (k < 3)
        {
          move[axis[2]] = move_scale * reference_box.minExtents()[axis[2]];
          sign_char[2] = '-';
        }
        else
        {
          move[axis[2]] = move_scale * reference_box.maxExtents()[axis[2]];
          sign_char[2] = '+';
        }
      }

      for (int j = 0; j < axis_range[1]; ++j)
      {
        sign_char[1] = ' ';
        axis[1] = -1;
        if (axis_range[1] > 1)
        {
          if (j == k)
          {
            // Do not do duplicate parings.
            continue;
          }

          axis[1] = j % 3;
          if (j < 3)
          {
            move[axis[1]] = move_scale * reference_box.minExtents()[axis[1]];
            sign_char[1] = '-';
          }
          else
          {
            move[axis[1]] = move_scale * reference_box.maxExtents()[axis[1]];
            sign_char[1] = '+';
          }
        }

        for (int i = 0; i < axis_range[0]; ++i)
        {
          if (axis_range[2] && i == k || axis_range[1] && i == j)
          {
            // Don't pair the same axes.
            continue;
          }

          axis[0] = i % 3;
          if (i < 3)
          {
            move[axis[0]] = move_scale * reference_box.minExtents()[axis[0]];
            sign_char[0] = '-';
          }
          else
          {
            move[axis[0]] = move_scale * reference_box.maxExtents()[axis[0]];
            sign_char[0] = '+';
          }

          test_box = initial_box + move;

          std::ostringstream msg;
          if (overlap_test)
          {
            msg << "overlap ";
          }
          else
          {
            msg << "exclude ";
          }

          msg << sign_char[0] << '[' << axis[0] << ']';
          if (axis[1] >= 0)
          {
            msg << sign_char[1] << '[' << axis[1] << ']';
          }
          if (axis[2] >= 0)
          {
            msg << sign_char[2] << '[' << axis[2] << ']';
          }


          if (overlap_test)
          {
            EXPECT_TRUE(reference_box.overlaps(test_box)) << msg.str();
            EXPECT_TRUE(test_box.overlaps(reference_box)) << msg.str();
          }
          else
          {
            // Expect exclusion.
            EXPECT_FALSE(reference_box.overlaps(test_box)) << msg.str();
            EXPECT_FALSE(test_box.overlaps(reference_box)) << msg.str();
          }
        }
      }
    }
  }
}  // namespace


TEST(Maths, AabbOverlap)
{
  const Aabb reference_box(glm::dvec3(-1.1, -2.2, -3.3), glm::dvec3(4.4, 5.5, 6.6));
  const Aabb initial_box(glm::dvec3(-1.0), glm::dvec3(1.0));
  Aabb test_box;

  test_box = initial_box;
  EXPECT_TRUE(reference_box.overlaps(test_box)) << "inside";
  EXPECT_TRUE(test_box.overlaps(reference_box)) << "inside";

  // Move the test box so it overlaps each axis
  // Single axis overlap.
  testOverlap(glm::ivec3(6, 0, 0), reference_box, initial_box, true);
  // Two axis overlap
  testOverlap(glm::ivec3(6, 6, 0), reference_box, initial_box, true);
  // Three axis overlap
  testOverlap(glm::ivec3(6, 6, 6), reference_box, initial_box, true);

  // Move so we don't have an overlap.
  // Single axis overlap.
  testOverlap(glm::ivec3(6, 0, 0), reference_box, initial_box, false);
  // Two axis overlap
  testOverlap(glm::ivec3(6, 6, 0), reference_box, initial_box, false);
  // Three axis overlap
  testOverlap(glm::ivec3(6, 6, 6), reference_box, initial_box, false);
}


TEST(Maths, AabbClip)
{
  const Aabb reference_box(glm::dvec3(-1.1, -2.2, -3.3), glm::dvec3(4.4, 5.5, 6.6));

  std::vector<glm::dvec3> lines;

  // Setup some lines entirely within the box .
  lines.push_back(reference_box.minExtents() + glm::dvec3(1, 1, 1));
  lines.push_back(reference_box.maxExtents() - glm::dvec3(1, 1, 1));

  lines.push_back(reference_box.maxExtents() - glm::dvec3(1, 1, 1));
  lines.push_back(reference_box.minExtents() + glm::dvec3(1, 1, 1));

  unsigned clip_flags = 0;
  bool line_intersect = false;
  glm::dvec3 start, end, dir1, dir2;
  for (size_t i = 0; i < lines.size(); i += 2)
  {
    start = lines[i];
    end = lines[i + 1];
    // Cache initial direction.
    dir1 = glm::normalize(end - start);

    EXPECT_TRUE(reference_box.contains(start));
    EXPECT_TRUE(reference_box.contains(end));

    line_intersect = reference_box.clipLine(start, end, &clip_flags);
    EXPECT_EQ(clip_flags, 0);
    EXPECT_FALSE(line_intersect);

    // Clipped direction.
    dir2 = glm::normalize(end - start);

    EXPECT_EQ(start, lines[i]);
    EXPECT_EQ(end, lines[i + 1]);

    EXPECT_TRUE(reference_box.contains(start)) << "no-clip " << i;
    EXPECT_TRUE(reference_box.contains(end)) << "no-clip " << i;

    // Validate direction.
    EXPECT_FALSE(glm::any(glm::greaterThan(glm::abs(dir1 - dir2), glm::dvec3(1e-9))));
  }

  lines.clear();
  // Add lines with end points outside the box.
  for (int i = 0; i < 8; ++i)
  {
    lines.push_back(glm::dvec3(0));
    lines.push_back(reference_box.corner(i) * 2.5);
  }

  for (size_t i = 0; i < lines.size(); i += 2)
  {
    start = lines[i];
    end = lines[i + 1];
    // Cache initial direction.
    dir1 = glm::normalize(end - start);

    EXPECT_TRUE(reference_box.contains(start));
    EXPECT_FALSE(reference_box.contains(end));

    line_intersect = reference_box.clipLine(start, end, &clip_flags);
    EXPECT_EQ(clip_flags, Aabb::kClippedEnd);
    EXPECT_TRUE(line_intersect);

    // Clipped direction.
    dir2 = glm::normalize(end - start);

    EXPECT_EQ(start, lines[i]);
    EXPECT_NE(end, lines[i + 1]);

    EXPECT_TRUE(reference_box.contains(start)) << "clip-end " << i;
    EXPECT_TRUE(reference_box.contains(end)) << "clip-end " << i;

    // Validate direction.
    EXPECT_FALSE(glm::any(glm::greaterThan(glm::abs(dir1 - dir2), glm::dvec3(1e-9))));
  }

  lines.clear();
  // Add lines with start points outside the box.
  for (int i = 0; i < 8; ++i)
  {
    lines.push_back(reference_box.corner(i) * 2.5);
    lines.push_back(glm::dvec3(0));
  }

  for (size_t i = 0; i < lines.size(); i += 2)
  {
    start = lines[i];
    end = lines[i + 1];
    // Cache initial direction.
    dir1 = glm::normalize(end - start);

    EXPECT_FALSE(reference_box.contains(start));
    EXPECT_TRUE(reference_box.contains(end));

    line_intersect = reference_box.clipLine(start, end, &clip_flags);
    EXPECT_EQ(clip_flags, Aabb::kClippedStart);
    EXPECT_TRUE(line_intersect);

    // Clipped direction.
    dir2 = glm::normalize(end - start);

    EXPECT_NE(start, lines[i]);
    EXPECT_EQ(end, lines[i + 1]);

    EXPECT_TRUE(reference_box.contains(start)) << "clip-start " << i;
    EXPECT_TRUE(reference_box.contains(end)) << "clip-start " << i;

    // Validate direction.
    EXPECT_FALSE(glm::any(glm::greaterThan(glm::abs(dir1 - dir2), glm::dvec3(1e-9))));
  }

  lines.clear();
  // Add lines with both points outside the box but intersecting the box.
  lines.push_back(reference_box.corner(Aabb::kCornerLll) * 2.5);
  lines.push_back(reference_box.corner(Aabb::kCornerUuu) * 2.5);

  lines.push_back(reference_box.corner(Aabb::kCornerUll) * 2.5);
  lines.push_back(reference_box.corner(Aabb::kCornerLuu) * 2.5);

  lines.push_back(reference_box.corner(Aabb::kCornerLul) * 2.5);
  lines.push_back(reference_box.corner(Aabb::kCornerUlu) * 2.5);

  lines.push_back(reference_box.corner(Aabb::kCornerUul) * 2.5);
  lines.push_back(reference_box.corner(Aabb::kCornerLlu) * 2.5);

  for (size_t i = 0; i < lines.size(); i += 2)
  {
    start = lines[i];
    end = lines[i + 1];
    // Cache initial direction.
    dir1 = glm::normalize(end - start);

    EXPECT_FALSE(reference_box.contains(start));
    EXPECT_FALSE(reference_box.contains(end));

    line_intersect = reference_box.clipLine(start, end, &clip_flags);
    EXPECT_EQ(clip_flags, Aabb::kClippedStart | Aabb::kClippedEnd);
    EXPECT_TRUE(line_intersect);

    // Clipped direction.
    dir2 = glm::normalize(end - start);

    EXPECT_NE(start, lines[i]);
    EXPECT_NE(end, lines[i + 1]);

    EXPECT_TRUE(reference_box.contains(start)) << "clip-both " << i;
    EXPECT_TRUE(reference_box.contains(end)) << "clip-both " << i;

    // Validate direction.
    EXPECT_FALSE(glm::any(glm::greaterThan(glm::abs(dir1 - dir2), glm::dvec3(1e-9))));
  }

  lines.clear();
  // Add lines which do not intersect the box.
  lines.push_back(reference_box.corner(Aabb::kCornerLll) * 2.5);
  lines.push_back(reference_box.corner(Aabb::kCornerLlu) * 2.5);

  for (size_t i = 0; i < lines.size(); i += 2)
  {
    start = lines[i];
    end = lines[i + 1];

    EXPECT_FALSE(reference_box.contains(start));
    EXPECT_FALSE(reference_box.contains(end));

    line_intersect = reference_box.clipLine(start, end, &clip_flags);
    EXPECT_EQ(clip_flags, 0);
    EXPECT_FALSE(line_intersect);

    EXPECT_EQ(start, lines[i]);
    EXPECT_EQ(end, lines[i + 1]);

    EXPECT_FALSE(reference_box.contains(start));
    EXPECT_FALSE(reference_box.contains(end));
  }
}
