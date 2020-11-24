// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <gputil/gpuDevice.h>
#include <gputil/gpuQueue.h>

#include <chrono>

extern gputil::Device g_gpu;

namespace gpudevicetest
{
TEST(GpuDevice, Enumerate)
{
  std::vector<gputil::DeviceInfo> device_infos;
  gputil::Device::enumerateDevices(device_infos);
  ASSERT_GT(device_infos.size(), 0u);

  // Select and create the first device.
  const gputil::DeviceInfo &device_info = device_infos[0];
  gputil::Device device(device_infos[0]);
  ASSERT_TRUE(device.isValid());

  EXPECT_EQ(device.info(), device_infos[0]);

  // Select the first device by simulated command line arguments.
  std::vector<std::string> args;
  std::string arg;
  switch (device_info.type)
  {
  case gputil::kDeviceCpu:
    arg = "cpu";
    break;
  case gputil::kDeviceGpu:
    arg = "gpu";
    break;
  default:
    arg = "any";
    break;
  }
  args.push_back("--accel=" + arg);

  std::ostringstream arg_stream;
  arg_stream << "--clver=" << device_info.version.major << '.' << device_info.version.minor;
  args.push_back(arg_stream.str());
  args.push_back("--device=\"" + device_info.name + "\"");
  args.push_back("--platform=\"" + device_info.platform + "\"");

  int argc = int(args.size());
  std::vector<const char *> argv;
  for (std::string &a : args)
  {
    argv.push_back(a.c_str());
  }
  device = gputil::Device();
  EXPECT_FALSE(device.isValid());
  device.select(argc, argv.data());
  ASSERT_TRUE(device.isValid());
  const gputil::DeviceInfo &info = device.info();
  EXPECT_EQ(info, device_info);
}
}  // namespace gpudevicetest
