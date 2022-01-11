// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include "slamio/Points.h"
#include "slamio/PointCloudReaderPly.h"

#include <ohmutil/PlyPointStream.h>

#include <glm/glm.hpp>

#include <fstream>
#include <limits>
#include <random>
#include <vector>

namespace slamio
{
TEST(Loader, Ply)
{
  // Create some points and save them to a ply.
  const std::string test_ply_name = "test_loader.ply";
  const size_t point_count = 100;
  std::vector<slamio::CloudPoint> points(point_count);

  ohm::PlyPointStream ply_out;

  ply_out.setProperties({
    ohm::PlyPointStream::Property{ "timestamp", ohm::PlyPointStream::Type::kFloat64 },
    ohm::PlyPointStream::Property{ "x", ohm::PlyPointStream::Type::kFloat64 },
    ohm::PlyPointStream::Property{ "y", ohm::PlyPointStream::Type::kFloat64 },
    ohm::PlyPointStream::Property{ "z", ohm::PlyPointStream::Type::kFloat64 },
    ohm::PlyPointStream::Property{ "nx", ohm::PlyPointStream::Type::kFloat64 },
    ohm::PlyPointStream::Property{ "ny", ohm::PlyPointStream::Type::kFloat64 },
    ohm::PlyPointStream::Property{ "nz", ohm::PlyPointStream::Type::kFloat64 },
    ohm::PlyPointStream::Property{ "red", ohm::PlyPointStream::Type::kUInt8 },
    ohm::PlyPointStream::Property{ "green", ohm::PlyPointStream::Type::kUInt8 },
    ohm::PlyPointStream::Property{ "blue", ohm::PlyPointStream::Type::kUInt8 },
    ohm::PlyPointStream::Property{ "alpha", ohm::PlyPointStream::Type::kUInt8 },
    ohm::PlyPointStream::Property{ "intensity", ohm::PlyPointStream::Type::kUInt16 },
  });

  std::ofstream ply_out_stream;

  ply_out_stream.open(test_ply_name);
  EXPECT_TRUE(ply_out_stream.is_open());

  ply_out.open(ply_out_stream);

  for (size_t i = 0; i < points.size(); ++i)
  {
    slamio::CloudPoint &pt = points[i];
    pt.timestamp = i * 1.0;
    pt.position.x = i * 1.0;
    pt.position.y = i * -1.0;
    pt.position.z = i * 1.0;
    pt.normal = pt.position;
    pt.colour.r = 1.0f;
    pt.colour.g = 1.0f;
    pt.colour.b = 1.0f;
    pt.colour.a = 1.0f;
    pt.intensity = 1.0f;

    ply_out.setProperty("timestamp", pt.timestamp);
    ply_out.setPointPosition(pt.position);
    ply_out.setPointNormal(pt.normal);
    ply_out.setProperty("red", uint8_t(pt.colour.r * 255.0f));
    ply_out.setProperty("green", uint8_t(pt.colour.g * 255.0f));
    ply_out.setProperty("blue", uint8_t(pt.colour.b * 255.0f));
    ply_out.setProperty("alpha", uint8_t(pt.colour.a * 255.0f));
    ply_out.setProperty("intensity", uint16_t(pt.intensity * float(std::numeric_limits<uint16_t>::max())));
    ply_out.writePoint();
  }

  ply_out.close();

  // Now read the PLY and compare.
  slamio::PointCloudReaderPly reader;
  reader.setDesiredChannels(DataChannel::Time | DataChannel::Position | DataChannel::Normal | DataChannel::Colour |
                            DataChannel::Intensity);

  ASSERT_TRUE(reader.open(test_ply_name.c_str()));
  ASSERT_EQ(reader.pointCount(), point_count);

  // Start reading points
  size_t read_count = 0;
  slamio::CloudPoint pt;
  while (reader.readNext(pt))
  {
    const float ef = 1e-4f;
    const auto &ref_pt = points[read_count];
    EXPECT_EQ(pt.timestamp, ref_pt.timestamp) << read_count;
    EXPECT_EQ(pt.position.x, ref_pt.position.x) << read_count;
    EXPECT_EQ(pt.position.y, ref_pt.position.y) << read_count;
    EXPECT_EQ(pt.position.z, ref_pt.position.z) << read_count;
    EXPECT_EQ(pt.normal.x, ref_pt.normal.x) << read_count;
    EXPECT_EQ(pt.normal.y, ref_pt.normal.y) << read_count;
    EXPECT_EQ(pt.normal.z, ref_pt.normal.z) << read_count;
    EXPECT_NEAR(pt.colour.r, ref_pt.colour.r, ef) << read_count;
    EXPECT_NEAR(pt.colour.g, ref_pt.colour.g, ef) << read_count;
    EXPECT_NEAR(pt.colour.b, ref_pt.colour.b, ef) << read_count;
    EXPECT_NEAR(pt.colour.a, ref_pt.colour.a, ef) << read_count;
    EXPECT_NEAR(pt.intensity, ref_pt.intensity, ef) << read_count;
    ++read_count;
  }

  EXPECT_EQ(read_count, point_count);
}
}  // namespace slamio
