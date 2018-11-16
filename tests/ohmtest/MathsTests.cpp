// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <ohm/Aabb.h>

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


TEST(Maths, Aabb)
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
