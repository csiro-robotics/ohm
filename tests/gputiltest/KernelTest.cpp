// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <gputil/gpuApiException.h>
#include <gputil/gpuBuffer.h>
#include <gputil/gpuDevice.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuProgram.h>
#include <gputil/gpuQueue.h>

#include <chrono>

#if GPUTIL_TYPE == GPUTIL_OPENCL
#include "matrixResource.h"
#endif  // GPUTIL_TYPE == GPUTIL_OPENCL

extern gputil::Device g_gpu;

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(matrixMultiply);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

namespace gpukerneltest
{
const unsigned kN = 32;
extern const float kMatrixA[kN * kN];
extern const float kMatrixB[kN * kN];

/// Log a @c std::chrono::clock::duration to an output stream.
///
/// The resulting string displays in the smallest possible unit to show three three
/// decimal places with display units ranging from seconds to nanoseconds. The table below
/// shows some example times.
///
/// Time(s)     | Display
/// ----------- | --------
/// 0.000000018 | 18ns
/// 0.000029318 | 29.318us
/// 0.0295939   | 29.593ms
/// 0.93        | 930ms
/// 15.023      | 15.023s
/// 15.000025   | 15.000s
///
/// Note that times are truncated, not rounded.
///
/// @tparam D The duration type of the form @c std::chrono::clock::duration.
/// @param out The output stream to log to.
/// @param duration The duration to convert to string.
template <typename D>
inline void logDuration(std::ostream &out, const D &duration)
{
  const bool negative = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() < 0;
  const char *sign = (!negative) ? "" : "-";
  D abs_duration = (!negative) ? duration : duration * -1;
  auto s = std::chrono::duration_cast<std::chrono::seconds>(abs_duration).count();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(abs_duration).count();
  ms = ms % 1000;

  if (s)
  {
    out << sign << s << "." << std::setw(3) << std::setfill('0') << ms << "s";
  }
  else
  {
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(abs_duration).count();
    us = us % 1000;

    if (ms)
    {
      out << sign << ms << "." << std::setw(3) << std::setfill('0') << us << "ms";
    }
    else
    {
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(abs_duration).count();
      ns = ns % 1000;

      if (us)
      {
        out << sign << us << "." << std::setw(3) << std::setfill('0') << ns << "us";
      }
      else
      {
        out << sign << ns << "ns";
      }
    }
  }
}

template <typename T, typename R>
inline std::ostream &operator<<(std::ostream &out, const std::chrono::duration<T, R> &d)
{
  logDuration(out, d);
  return out;
}
using TimingClock = std::chrono::high_resolution_clock;

TEST(GpuKernel, Simple)
{
  int err = 0;
  gputil::Program program(g_gpu, "test-program");

  err = GPUTIL_BUILD_FROM_SOURCE(program, matrixCode, matrixCode_length, gputil::BuildArgs{});

  ASSERT_EQ(err, 0) << gputil::ApiException::errorCodeString(err);

  gputil::Kernel kernel = GPUTIL_MAKE_KERNEL(program, matrixMultiply);
  ASSERT_TRUE(kernel.isValid());

  kernel.addLocal([](size_t group_size) { return group_size * sizeof(float); });

  kernel.calculateOptimalWorkGroupSize();

  gputil::Queue queue = g_gpu.defaultQueue();

  gputil::Buffer matrix_a(g_gpu, sizeof(kMatrixA), gputil::kBfRead);
  gputil::Buffer matrix_b(g_gpu, sizeof(kMatrixB), gputil::kBfRead);
  gputil::Buffer matrix_out(g_gpu, sizeof(float) * kN, gputil::kBfWrite);

  const auto gpu_start = TimingClock::now();
  size_t copy_bytes = 0;
  copy_bytes = matrix_a.write(kMatrixA, sizeof(kMatrixA));
  ASSERT_EQ(copy_bytes, sizeof(kMatrixA));
  copy_bytes = matrix_b.write(kMatrixB, sizeof(kMatrixB));
  ASSERT_EQ(copy_bytes, sizeof(kMatrixB));
  err = kernel(gputil::Dim3(kN), gputil::Dim3(kN), &queue, gputil::BufferArg<float>(matrix_out),
               gputil::BufferArg<float>(matrix_a), gputil::BufferArg<float>(matrix_b), kN);
  ASSERT_EQ(err, 0) << gputil::ApiException::errorCodeString(err);
  queue.finish();

  // Copy results from GPU.
  std::vector<float> gpu_results(kN);
  copy_bytes = matrix_out.read(gpu_results.data(), sizeof(*gpu_results.data()) * gpu_results.size());
  ASSERT_EQ(copy_bytes, sizeof(*gpu_results.data()) * gpu_results.size());
  const auto gpu_end = TimingClock::now();
  std::cout << "GPU time: " << (gpu_end - gpu_start) << std::endl;

  // Calculate in CPU.
  std::vector<float> cpu_results(kN);

  const auto cpu_start = TimingClock::now();
  for (unsigned i = 0; i < kN; ++i)
  {
    float &out = cpu_results[i];
    out = 0;
    for (unsigned j = 0; j < kN; ++j)
    {
      out += kMatrixA[i * kN + j] * kMatrixB[j * kN + i];
    }
  }
  const auto cpu_end = TimingClock::now();
  std::cout << "CPU time: " << (cpu_end - cpu_start) << std::endl;

  // Validate results.
  for (size_t i = 0; i < gpu_results.size(); ++i)
  {
    EXPECT_NEAR(gpu_results[i], cpu_results[i], 1e-3f);
  }
}

TEST(GpuKernel, NullArg)
{
  int err = 0;
  gputil::Program program(g_gpu, "test-program");

  err = GPUTIL_BUILD_FROM_SOURCE(program, matrixCode, matrixCode_length, gputil::BuildArgs{});

  ASSERT_EQ(err, 0) << gputil::ApiException::errorCodeString(err);

  gputil::Kernel kernel = GPUTIL_MAKE_KERNEL(program, matrixMultiply);
  ASSERT_TRUE(kernel.isValid());

  kernel.addLocal([](size_t group_size) { return group_size * sizeof(float); });

  kernel.calculateOptimalWorkGroupSize();

  gputil::Queue queue = g_gpu.defaultQueue();

  err = kernel(gputil::Dim3(kN), gputil::Dim3(kN), &queue, gputil::BufferArg<float>(nullptr),
               gputil::BufferArg<float>(nullptr), gputil::BufferArg<float>(nullptr), kN);
  ASSERT_EQ(err, 0) << gputil::ApiException::errorCodeString(err);
}

const float kMatrixA[kN * kN] =  //
  { -5.36024f, 3.03301f,  7.89674f,  -6.46361f, 2.37263f,  6.38046f,  -4.85497f, 4.01736f,  -5.83961f, -0.30897f,
    2.34736f,  9.19266f,  -4.81157f, 5.12527f,  -6.22901f, 4.46971f,  3.42575f,  8.55879f,  5.77094f,  4.38269f,
    -1.98192f, -3.92955f, -7.52568f, 5.00008f,  -2.03231f, -0.67967f, -5.99790f, -9.10126f, 6.17714f,  -1.57793f,
    -9.70259f, -1.88551f, -4.55362f, 6.07408f,  -1.24905f, -2.56581f, -2.32887f, 0.18148f,  2.41612f,  -7.29838f,
    -1.10756f, 9.76593f,  -3.10208f, 5.38991f,  -3.62616f, -3.80617f, 8.14205f,  5.79305f,  1.71217f,  4.25061f,
    -6.16396f, -0.55964f, 7.84027f,  6.69293f,  -1.74443f, 0.32048f,  2.04413f,  2.42060f,  -2.69034f, -1.81077f,
    -7.80228f, -3.67605f, -7.90027f, -1.35433f, -5.90841f, 6.49489f,  -5.28090f, 4.95714f,  -9.37658f, -4.61623f,
    6.33320f,  9.05788f,  7.74236f,  -0.84028f, -6.64415f, 1.74848f,  5.77861f,  -3.17352f, 1.21083f,  5.08367f,
    -7.72516f, 6.42745f,  -9.83351f, -9.70465f, -3.54772f, -7.37416f, -2.08868f, 3.56927f,  -3.43333f, -3.02251f,
    7.18175f,  -8.15131f, -6.03459f, -1.73443f, -0.91152f, -3.75740f, 6.19633f,  4.14575f,  -5.37607f, -7.36705f,
    8.89244f,  1.39289f,  1.53383f,  1.12498f,  -5.87719f, -4.37070f, -6.50984f, 3.57958f,  5.40322f,  -4.65746f,
    -5.70183f, -7.44602f, 7.56560f,  -6.92505f, 6.06597f,  -8.85882f, 8.20909f,  -8.84451f, -7.55648f, -4.73150f,
    -0.96385f, 0.38258f,  -3.10631f, 9.93032f,  -7.06206f, 4.61396f,  -6.19065f, 8.91965f,  -3.56298f, 0.67778f,
    7.93350f,  9.53591f,  5.65004f,  2.51638f,  -9.49472f, -8.75178f, 4.07173f,  -4.84450f, -1.54252f, -0.56567f,
    7.19198f,  -2.53427f, -4.16850f, 1.47983f,  -5.05385f, 4.89241f,  -6.33212f, -0.49553f, 8.43587f,  -4.96575f,
    -6.58216f, -9.68587f, -1.80548f, 8.79417f,  6.98266f,  4.28916f,  0.70989f,  1.61469f,  -9.07296f, -1.58392f,
    -7.42654f, -2.39975f, -2.34463f, -6.53747f, -1.00427f, -8.92843f, 2.95637f,  3.31480f,  -9.64551f, -8.64690f,
    -3.58853f, 4.24319f,  7.50260f,  -3.61401f, -9.91683f, -4.56730f, -6.58313f, -8.46006f, -6.66975f, 0.09210f,
    -8.16130f, 2.19071f,  -7.38338f, -7.32992f, 1.43260f,  4.27257f,  -6.88868f, -4.32963f, -7.59036f, 1.21713f,
    6.43214f,  -8.95334f, -1.72864f, -1.03092f, 5.44935f,  0.31199f,  -8.90184f, 4.49388f,  7.20876f,  7.47423f,
    -2.83166f, 5.43556f,  9.89562f,  -1.17701f, 2.41719f,  0.93132f,  -2.64796f, 6.97072f,  6.98565f,  -1.43908f,
    9.49609f,  -8.53626f, 5.72548f,  0.14453f,  3.07307f,  1.92560f,  2.96141f,  7.39365f,  5.07517f,  -3.35717f,
    -9.99346f, -8.71735f, 4.27034f,  4.41887f,  -5.94145f, 6.74185f,  4.45710f,  0.85286f,  6.23896f,  -1.69507f,
    -0.28377f, 2.36344f,  7.08174f,  3.88279f,  -0.17452f, -3.92190f, 4.72935f,  -6.47502f, -7.46027f, -7.74994f,
    2.08494f,  -7.89875f, 9.70752f,  -0.47615f, -2.94333f, -8.64008f, -8.43222f, 1.79943f,  -0.72707f, -5.05841f,
    8.19380f,  -9.77925f, -7.57259f, -9.98601f, 5.58037f,  9.71955f,  -9.63579f, -6.12072f, 7.87742f,  -3.50368f,
    2.67012f,  -9.73830f, -3.51469f, 2.88600f,  1.19969f,  5.83671f,  8.51835f,  3.37408f,  5.75521f,  1.05292f,
    8.05218f,  -6.35638f, -9.35331f, -6.83586f, -9.18902f, 7.46615f,  4.29568f,  -5.33234f, 7.68480f,  1.73829f,
    8.37063f,  0.37742f,  -7.88483f, 6.52919f,  2.11953f,  -6.10367f, 4.85482f,  -3.29317f, -8.08474f, -8.98830f,
    4.65221f,  -1.64774f, -6.80234f, 3.86627f,  9.08059f,  3.37334f,  1.56074f,  -7.22893f, 0.11437f,  -6.27332f,
    8.09680f,  -2.57089f, 1.01584f,  -6.95102f, -1.32714f, -2.38946f, -1.19276f, -7.92486f, -4.55988f, 9.77793f,
    -4.71558f, 2.69454f,  -3.19649f, 8.08590f,  2.44223f,  7.55874f,  -2.88437f, 6.44491f,  -8.84439f, 9.65934f,
    -2.88316f, 6.63873f,  6.22111f,  -0.94517f, 1.12322f,  1.39204f,  -1.52287f, 5.85185f,  0.40659f,  -2.79680f,
    -8.36758f, -8.92702f, -4.47743f, -1.12899f, 2.93885f,  4.55356f,  7.32685f,  3.61080f,  -3.09299f, 3.34278f,
    7.43658f,  -8.00631f, 3.95550f,  3.51196f,  7.91253f,  5.53583f,  7.71750f,  6.74937f,  6.30989f,  -9.09123f,
    -0.55358f, -1.26978f, 6.43639f,  -0.07554f, -1.55975f, 7.11895f,  -6.25526f, -5.50645f, 6.79627f,  -3.74662f,
    6.33883f,  9.51613f,  8.90242f,  4.62723f,  -5.18018f, 8.09495f,  -5.46958f, -5.39303f, -7.75632f, -6.12000f,
    3.55112f,  7.44182f,  -9.13721f, 2.42484f,  7.64426f,  0.46464f,  -2.24053f, 4.31547f,  -8.39008f, 4.22868f,
    -8.98346f, -0.08195f, -2.41010f, 3.90651f,  -8.87544f, 1.98014f,  5.76648f,  7.06711f,  7.60926f,  0.86063f,
    1.39293f,  -3.60422f, 0.49759f,  8.38953f,  -6.78263f, -7.37406f, 3.63870f,  7.58595f,  -0.20697f, -8.83972f,
    0.26591f,  -5.29550f, -6.75722f, 4.18623f,  -6.89695f, -0.03029f, -1.04303f, 3.63271f,  -6.79129f, -7.75546f,
    6.29580f,  -9.10068f, 8.01421f,  -7.27066f, -6.83203f, 9.06481f,  -3.64834f, -0.86170f, -8.12355f, -3.06568f,
    1.20411f,  3.69757f,  -6.66817f, -2.82572f, 8.17502f,  3.21200f,  -3.59027f, -4.94095f, -1.46972f, -1.07293f,
    -6.52541f, 0.08928f,  1.53614f,  7.96836f,  -3.74326f, -3.75225f, -3.38809f, 2.67324f,  -5.92884f, 1.06186f,
    -2.23427f, 2.54998f,  4.38539f,  -4.85867f, 9.55938f,  -3.87439f, -1.59161f, 6.58846f,  -4.76559f, -3.40654f,
    -0.57622f, -5.13035f, -2.22436f, -1.33697f, -3.96378f, 0.05732f,  3.21759f,  -5.40274f, 1.15436f,  3.44465f,
    -0.73830f, 7.67949f,  9.65987f,  0.31785f,  8.12037f,  -8.01185f, 7.60512f,  -5.86755f, 8.77293f,  -6.75717f,
    -2.89329f, -5.79838f, -7.94096f, -1.10030f, -8.30164f, -4.31436f, 4.81738f,  -4.44737f, 0.23332f,  7.57553f,
    3.65214f,  1.03828f,  -4.95143f, -5.18061f, 6.88024f,  -1.84921f, -0.49890f, 9.95765f,  6.68037f,  2.39398f,
    -1.63835f, 0.77123f,  -1.98306f, 4.92871f,  -3.54779f, 8.01561f,  5.38072f,  -1.35251f, -5.72469f, 4.63131f,
    6.28110f,  -2.05917f, 5.05418f,  -1.96820f, 0.26304f,  -3.35312f, -8.72488f, -4.31363f, 2.09205f,  1.34882f,
    -8.02546f, -0.89581f, -6.26459f, -3.93678f, 1.68485f,  -9.94318f, 9.65893f,  -2.57243f, 0.31969f,  -8.32212f,
    3.05471f,  -0.73482f, -2.51721f, -3.15348f, -8.68139f, 0.04443f,  -4.44787f, 4.30533f,  1.07191f,  -9.43039f,
    -7.51728f, -3.40529f, -8.44354f, -6.64094f, -6.72808f, 6.99461f,  3.27563f,  7.80686f,  8.33628f,  -1.53385f,
    5.04029f,  -8.32624f, 4.43213f,  1.22868f,  -2.93367f, -1.88713f, -5.11814f, -2.45039f, 0.51051f,  -6.97197f,
    -1.15625f, -7.82870f, -2.22295f, 8.50125f,  -1.41646f, 7.43619f,  4.90238f,  6.69840f,  -6.78077f, -2.19466f,
    -2.07207f, 1.12748f,  -8.75182f, -8.09828f, 8.30520f,  -8.90648f, -7.00372f, 1.30008f,  -7.46223f, -5.07510f,
    4.41644f,  -3.87758f, -5.16742f, -5.74961f, -5.13970f, -4.48847f, 2.57534f,  -7.58444f, -6.22609f, -7.25655f,
    5.92211f,  -5.16910f, -2.96338f, 0.02568f,  -0.17946f, 3.99880f,  -5.41442f, 7.44778f,  -7.06188f, 2.98760f,
    3.37074f,  1.51384f,  5.91171f,  8.52166f,  -1.82390f, 4.18569f,  -1.64458f, -6.09212f, 4.97994f,  3.00369f,
    -2.73404f, 4.76250f,  2.59263f,  -5.31223f, 2.18863f,  2.61276f,  8.79504f,  2.70871f,  9.23223f,  -4.30108f,
    1.52192f,  -7.93707f, -7.49678f, 2.14144f,  6.98578f,  -9.50332f, 5.69072f,  4.46349f,  3.46314f,  -5.13240f,
    -9.64027f, 5.65795f,  8.53658f,  1.14730f,  6.73145f,  -6.27883f, -5.73621f, 9.34599f,  0.48297f,  -3.98020f,
    5.93749f,  9.03804f,  -7.68380f, 2.96965f,  3.48118f,  -5.21574f, -2.72929f, 6.84835f,  5.74761f,  -3.31882f,
    1.19517f,  9.46701f,  6.26552f,  -4.95271f, 9.61201f,  9.67824f,  6.49148f,  4.01508f,  0.74510f,  -9.28003f,
    4.88930f,  6.57170f,  8.22741f,  6.64313f,  7.29528f,  2.41739f,  -1.84803f, -8.01405f, -3.60924f, 2.71333f,
    -9.01155f, -4.14871f, -6.24235f, 5.38007f,  -2.16414f, 5.85224f,  -0.57668f, 7.28138f,  6.31025f,  2.66814f,
    6.15197f,  -0.70156f, -2.19628f, 4.90916f,  1.35013f,  2.50890f,  -4.20175f, -3.60368f, -8.59165f, -1.10329f,
    -7.19165f, -4.87665f, 6.73913f,  1.10513f,  5.73071f,  8.86371f,  4.79894f,  2.93544f,  2.91481f,  7.39946f,
    5.68901f,  -2.34821f, -5.85872f, -7.25887f, 6.46002f,  6.08832f,  5.65362f,  9.87776f,  5.09161f,  -0.01409f,
    -4.96815f, -5.66504f, 7.71045f,  -3.27393f, -4.29291f, -1.78235f, -7.61528f, 2.95097f,  -5.73902f, -8.70765f,
    2.39502f,  7.04027f,  -4.73373f, 1.09950f,  9.62861f,  9.98262f,  4.93704f,  -6.33815f, -5.94339f, -9.57139f,
    -6.03906f, -5.27811f, -5.01243f, 3.01877f,  8.96819f,  7.47303f,  -2.31526f, -5.66858f, -2.71667f, 1.72024f,
    0.11318f,  -2.70216f, 6.07467f,  9.78337f,  -4.72329f, -4.02421f, -0.18395f, 5.27181f,  3.96694f,  -7.48053f,
    -6.92074f, -3.42988f, -2.40490f, 9.77333f,  -0.30730f, 4.37769f,  2.63065f,  0.53899f,  -3.11768f, 5.71806f,
    8.91440f,  8.94023f,  8.02451f,  5.28794f,  0.70042f,  -4.69685f, -8.20483f, -2.71358f, 8.24728f,  -9.17618f,
    0.20763f,  -6.70992f, -4.39054f, 0.62377f,  -6.60567f, 1.77831f,  -7.72461f, -7.43482f, -8.09086f, -4.98254f,
    7.44223f,  3.77624f,  9.52371f,  9.37664f,  -9.34868f, 3.35080f,  -6.27862f, -8.37104f, -8.24965f, -1.84965f,
    -8.86093f, 0.18754f,  1.22750f,  4.23103f,  -1.45596f, -3.03658f, -7.51905f, -1.39812f, 2.69410f,  -3.25497f,
    1.77429f,  -7.11159f, -4.00556f, 4.15675f,  -5.66598f, -0.44158f, -9.76796f, 9.54147f,  -9.86836f, -4.84623f,
    -1.09882f, -9.89516f, 5.35354f,  9.58199f,  -4.00881f, 9.91393f,  -5.05084f, 4.24625f,  -7.21973f, -1.75442f,
    4.37824f,  -8.95964f, -6.88128f, -5.19692f, 3.27883f,  -0.78038f, 8.13298f,  6.63811f,  -0.86566f, 6.81223f,
    -9.96513f, -9.50483f, -1.65841f, -3.17166f, 4.88875f,  -8.84550f, 4.35808f,  -5.93102f, 7.72902f,  -4.47840f,
    0.94915f,  8.90513f,  -2.34326f, -5.83943f, 0.80032f,  9.48064f,  -8.88043f, -8.53178f, 8.49664f,  -2.25810f,
    4.40557f,  -5.06630f, -1.88566f, 3.45469f,  2.00550f,  -8.32170f, 6.64334f,  9.36825f,  -6.39497f, -9.24162f,
    5.66811f,  8.15070f,  -9.85093f, 7.74863f,  -3.68800f, 7.97893f,  -4.50936f, -3.12566f, -6.79998f, 1.17077f,
    -1.42418f, 6.95268f,  -8.15174f, 6.24649f,  -7.90176f, -7.18254f, -0.01591f, 1.21140f,  4.71176f,  -7.12009f,
    -1.44458f, -7.25360f, 9.59549f,  3.24089f,  1.61594f,  4.44725f,  -7.77722f, -9.66682f, -9.12136f, 5.34722f,
    -7.76316f, -2.12404f, 6.59313f,  -8.92211f, 1.71711f,  -3.43437f, -9.18456f, 7.97844f,  8.35414f,  -3.04156f,
    5.70274f,  3.46606f,  0.59527f,  5.55289f,  6.19152f,  -1.46750f, -5.02463f, 9.02596f,  -4.21127f, 3.32164f,
    -8.33186f, 4.49628f,  0.28004f,  2.55836f,  6.85301f,  -4.94816f, -9.18357f, 1.71467f,  -4.22061f, 5.04427f,
    2.10975f,  2.49635f,  6.53246f,  2.67699f,  5.66162f,  1.25500f,  -0.36122f, 9.59258f,  5.01117f,  -9.81135f,
    7.09733f,  1.14298f,  -7.04834f, 0.16270f,  -3.23290f, -8.00067f, -1.88708f, -3.64277f, -5.08015f, -6.67229f,
    -7.38615f, -6.44324f, -1.64319f, -2.12077f, -9.58402f, -4.62701f, 6.63675f,  3.77680f,  -2.78879f, -3.81928f,
    2.92034f,  -5.45090f, -7.53028f, 7.72819f,  -8.86762f, 6.86855f,  7.02809f,  -4.87721f, 1.48462f,  0.20060f,
    8.67957f,  -2.56093f, -6.42348f, 0.37000f,  9.35188f,  9.18333f,  -9.44512f, -0.96261f, -3.07814f, 7.24771f,
    -9.80699f, -5.89575f, 5.21080f,  0.42694f,  -8.56844f, 4.09713f,  8.89651f,  -5.50764f, 0.76284f,  7.66084f,
    -6.09291f, -6.84949f, 8.99053f,  -2.49485f, 8.34431f,  3.49296f,  9.39794f,  8.03067f,  -0.34257f, 4.59870f,
    -8.20489f, -2.41027f, -5.17759f, 4.23562f,  5.27438f,  5.16272f,  -1.55350f, -1.10750f, -0.26884f, -7.34586f,
    -3.73046f, 4.65437f,  -9.48321f, -4.41696f, 2.21830f,  2.05298f,  -9.80857f, 1.29349f,  -3.08281f, 5.69946f,
    -0.37416f, -3.32929f, -1.89301f, 5.33941f,  8.28091f,  1.60708f,  -4.06870f, 5.07272f,  7.90258f,  -4.51025f,
    1.08962f,  -6.81522f, -2.66204f, 3.71443f,  0.81955f,  4.86542f,  -6.39781f, 1.47674f,  0.07887f,  -4.24451f,
    2.11837f,  1.66433f,  5.47027f,  8.70229f };

const float kMatrixB[kN * kN] =  //
  { 9.72021f,  6.49670f,  -7.54882f, -8.00339f, 9.95481f,  4.63599f,  4.20078f,  -4.27181f, 0.30477f,  7.84052f,
    2.90157f,  -6.98213f, 7.47744f,  -7.89820f, -7.65084f, 2.48255f,  1.32322f,  -6.52176f, 4.50639f,  -3.53208f,
    -5.74609f, 3.93423f,  2.71765f,  -8.25343f, -0.35546f, 0.94940f,  -9.00612f, 0.58083f,  5.97516f,  6.82584f,
    1.93767f,  5.77743f,  -6.12759f, -9.13520f, 4.31707f,  7.22972f,  -7.86060f, 3.07278f,  -1.19097f, -0.84330f,
    -3.17867f, 6.15698f,  -6.53906f, 4.97187f,  -8.34039f, 5.87001f,  -1.54037f, -7.58075f, -9.56430f, 6.75385f,
    5.76880f,  2.87913f,  -4.01859f, -1.37107f, -9.25824f, 7.36066f,  -8.28706f, -4.43314f, 2.24932f,  2.66087f,
    -3.18037f, -8.11990f, 2.24388f,  3.49329f,  7.75417f,  -9.57024f, 1.95602f,  -1.52300f, -4.80017f, 9.40973f,
    -9.52204f, -2.72032f, 3.66919f,  1.47221f,  -8.97749f, 7.80755f,  -3.92024f, 3.61067f,  9.26715f,  -6.90612f,
    -0.61920f, -7.27325f, 1.82546f,  3.50142f,  5.60867f,  -4.10997f, -3.31390f, -7.62985f, 0.44593f,  -2.09340f,
    5.86013f,  -2.96442f, -8.30453f, -3.08324f, 1.55652f,  3.80774f,  5.96238f,  9.92005f,  -3.10870f, 1.66077f,
    2.70532f,  6.21491f,  3.05901f,  -7.99205f, -9.86389f, -0.51518f, -5.36827f, -2.75489f, 3.29030f,  -4.18169f,
    7.43399f,  -6.65926f, -6.55035f, -5.33037f, -6.13122f, 5.14048f,  -5.20897f, -4.77415f, 0.73274f,  6.44297f,
    3.99113f,  6.58172f,  -0.87100f, 2.19202f,  4.37717f,  4.37294f,  -3.53482f, 5.26908f,  -7.14042f, -0.19742f,
    5.79998f,  -7.74966f, 9.06204f,  0.59119f,  5.91872f,  -1.26244f, -9.30614f, 1.60701f,  -9.77323f, 7.84483f,
    -0.49254f, 1.09524f,  -2.61935f, -1.54054f, 3.64331f,  9.23285f,  4.14123f,  6.48069f,  -2.40550f, -7.77255f,
    5.96298f,  -8.09423f, -5.23870f, -3.21136f, -8.94306f, -3.34556f, 8.70678f,  -3.15603f, 2.92980f,  1.83122f,
    5.69786f,  9.91403f,  -3.72939f, -5.29828f, 7.88044f,  5.02729f,  8.64174f,  -4.67439f, -3.60892f, -0.31501f,
    5.45154f,  4.44833f,  -7.38359f, -2.57008f, 5.05439f,  6.14216f,  8.33218f,  -6.91002f, -5.85057f, 9.38616f,
    8.51808f,  -1.40023f, 5.30463f,  6.06921f,  9.31414f,  -5.25626f, 0.50421f,  9.37392f,  6.20538f,  -5.25048f,
    9.28179f,  7.46624f,  -4.29274f, -3.99396f, -1.70260f, -1.41805f, -2.99618f, -4.14984f, -4.43649f, 0.22653f,
    -3.85346f, -6.16905f, -1.03679f, 9.48265f,  6.99228f,  4.37460f,  0.88836f,  -8.59612f, -0.10017f, 8.61519f,
    1.44506f,  -1.95708f, 0.61289f,  -2.03448f, -4.29022f, 3.46706f,  3.58971f,  -2.08406f, -0.12826f, -6.55930f,
    -5.53889f, 5.33895f,  4.95526f,  -6.79917f, 4.64243f,  -0.96457f, -8.80923f, 7.97795f,  2.52293f,  5.33506f,
    0.93494f,  -4.64497f, -9.66885f, 6.96163f,  -5.15156f, -9.57337f, 2.04115f,  4.53541f,  -7.96206f, 4.67903f,
    -0.95967f, -3.80926f, 9.72102f,  6.95524f,  8.96244f,  -1.85067f, 8.34267f,  9.20739f,  -2.97400f, 0.50522f,
    4.45701f,  -2.45030f, -9.37817f, 0.51074f,  -0.64578f, -1.08584f, 0.93652f,  9.04023f,  -0.38636f, -0.92356f,
    5.74699f,  1.76563f,  8.96288f,  5.37908f,  1.03215f,  -3.62605f, -1.49537f, -9.85393f, -1.15119f, 3.10760f,
    -1.09071f, -0.44696f, 4.50947f,  1.11015f,  -6.82136f, -2.81877f, -8.25014f, -1.69993f, 3.39525f,  6.94262f,
    9.30892f,  -0.91876f, -7.59360f, -3.07290f, -6.58888f, -8.53159f, -5.13604f, 4.48454f,  4.35136f,  9.60140f,
    -4.84089f, 4.81833f,  3.76677f,  5.70460f,  -9.98024f, 4.96289f,  0.57483f,  -6.88775f, 9.53015f,  -8.31271f,
    -3.50710f, -8.36483f, -7.66609f, -9.78223f, -7.69138f, -6.74942f, 9.82744f,  -4.79658f, -5.08425f, 0.58182f,
    -9.45953f, 2.42277f,  -8.02090f, -2.17367f, -2.36670f, -5.69205f, 6.37509f,  9.93978f,  7.07821f,  0.04397f,
    4.21192f,  2.26467f,  1.84011f,  8.99113f,  4.24749f,  0.21837f,  -1.81613f, -9.64470f, 2.29366f,  -0.08927f,
    -2.86600f, -3.90374f, 2.29998f,  -7.22178f, 2.46880f,  -2.22964f, 8.15552f,  7.34957f,  8.34927f,  9.99332f,
    0.90497f,  -8.41086f, -3.36952f, 0.03720f,  -4.21072f, 4.74920f,  1.94162f,  3.85573f,  -3.48802f, 4.34596f,
    -1.90485f, -0.13649f, 9.01643f,  -6.86455f, -5.64406f, 0.56684f,  -7.40443f, 3.03110f,  3.16201f,  -0.50511f,
    -1.95300f, 1.86506f,  -2.57450f, 8.32579f,  -8.20120f, -1.86530f, 9.84893f,  8.08739f,  7.01831f,  2.31716f,
    3.77998f,  3.89498f,  6.77104f,  -9.78017f, -9.93904f, 8.96840f,  -4.54495f, -4.99620f, 2.91460f,  -7.72258f,
    3.79553f,  -1.70769f, 3.52195f,  -7.85207f, 4.08745f,  4.18976f,  -2.19477f, 4.42266f,  6.44037f,  0.03779f,
    -6.08180f, 6.59765f,  2.98794f,  8.87748f,  2.37885f,  -9.15098f, -3.68875f, -6.28752f, -3.12921f, 2.56893f,
    0.04527f,  7.34534f,  6.84835f,  4.55525f,  2.56147f,  0.32295f,  4.09264f,  -9.77629f, -2.20164f, 8.98122f,
    -2.37686f, 3.67821f,  -8.71226f, -3.63423f, -3.95822f, -5.07941f, -6.28217f, 5.45521f,  8.29136f,  -3.95979f,
    5.34883f,  8.49710f,  -9.45647f, 3.52080f,  -6.85006f, -2.51907f, -2.27403f, -0.96760f, 1.38505f,  7.14016f,
    -5.29428f, 2.03995f,  8.33777f,  -1.08866f, 9.93598f,  -2.34727f, 4.51508f,  9.88285f,  -6.72229f, -5.94667f,
    8.92935f,  7.51640f,  6.30818f,  -3.17546f, 4.99993f,  -2.77358f, 5.52634f,  7.15288f,  -1.37215f, 4.91670f,
    -8.88527f, 1.98117f,  -6.50936f, -3.61517f, -7.12799f, 1.12608f,  9.85108f,  -7.29680f, 9.47304f,  -0.33523f,
    -9.91071f, 7.42134f,  -8.52789f, -7.01830f, -1.47236f, 2.82212f,  7.18804f,  5.02125f,  5.50058f,  -6.17321f,
    6.48908f,  -9.92950f, -6.71491f, 4.40735f,  7.97303f,  -4.97025f, -5.60849f, 1.59477f,  -1.81922f, 3.71182f,
    -1.09127f, -7.62416f, 3.53435f,  -9.64159f, 1.32484f,  7.28214f,  0.27252f,  8.80178f,  3.66838f,  3.98759f,
    -2.02252f, 4.07606f,  8.82064f,  8.81843f,  3.26974f,  2.14770f,  -4.84002f, 6.72732f,  9.31378f,  -8.35054f,
    -6.75543f, 0.04947f,  0.32170f,  -7.36081f, 8.97718f,  -3.82033f, 7.50091f,  9.80347f,  -9.85488f, -3.27167f,
    0.20347f,  8.70520f,  -7.09139f, -8.08796f, -4.49265f, -6.69558f, -9.82386f, -9.14083f, -4.98351f, -8.62950f,
    5.41930f,  -0.34157f, -6.57185f, 2.84796f,  3.53601f,  -8.92027f, -0.40482f, 4.10119f,  9.76288f,  5.11375f,
    3.81444f,  -8.32347f, 6.88764f,  6.94036f,  -6.72126f, -6.71859f, 6.90051f,  -0.63200f, 2.23868f,  1.76648f,
    2.33194f,  5.19969f,  -2.25654f, -2.86422f, 4.06623f,  -2.14925f, -4.11815f, 0.00345f,  7.41533f,  -6.53569f,
    0.81220f,  9.68861f,  -0.28606f, 9.41337f,  -1.86321f, 2.28227f,  -1.06735f, -4.68472f, -1.37232f, -3.14666f,
    -8.96117f, -3.28558f, -9.98905f, 4.53115f,  -6.46004f, 8.31572f,  -1.53614f, 7.73957f,  6.71808f,  -2.61464f,
    -9.50774f, -0.10605f, 2.83820f,  -6.07639f, -8.05567f, 7.19532f,  6.72458f,  -8.00312f, 7.50167f,  9.14375f,
    0.77346f,  3.22835f,  -6.08578f, 0.31340f,  -7.80426f, -1.62902f, 8.78675f,  5.50377f,  -6.59283f, -3.05217f,
    -6.94200f, 8.81406f,  -0.25877f, -9.58286f, -7.76065f, -7.84144f, 4.98209f,  3.28529f,  6.34059f,  8.59927f,
    3.24917f,  -7.25361f, 2.25505f,  2.97473f,  0.47787f,  -2.50142f, 7.04716f,  8.43826f,  1.77323f,  -6.62130f,
    6.73207f,  3.35351f,  -2.59137f, 5.57984f,  3.54933f,  -0.94410f, -4.35811f, 2.79520f,  -3.10291f, -2.70173f,
    7.30030f,  4.61770f,  -5.70417f, -6.84566f, -0.25059f, 0.10636f,  9.64547f,  -6.82760f, 2.37075f,  -7.82050f,
    -2.94639f, 5.89824f,  4.98369f,  0.12572f,  -2.39117f, 3.23328f,  -8.50044f, -7.46540f, 1.74079f,  -4.88408f,
    8.08733f,  -5.25252f, -6.14164f, -1.86469f, 1.62247f,  -4.80371f, 1.11036f,  4.95736f,  -2.01481f, -4.29467f,
    -6.41822f, 0.85665f,  9.62229f,  0.08958f,  9.63046f,  -7.88349f, 7.33600f,  -4.94165f, 4.75199f,  0.38527f,
    6.51710f,  -1.57478f, -7.06609f, -8.11342f, 7.98754f,  2.23761f,  -9.37049f, -3.11926f, 7.84700f,  0.26796f,
    -0.86136f, -6.92978f, -4.75270f, -1.07718f, 0.19839f,  -7.83948f, 5.21860f,  -5.59296f, 8.28023f,  7.97795f,
    -8.78858f, 2.62840f,  0.75065f,  4.95771f,  3.76687f,  -0.05831f, 6.48141f,  -0.92250f, -9.68367f, -9.94385f,
    -2.10315f, -6.87092f, 2.18411f,  1.68643f,  -1.18880f, 3.82465f,  -6.41588f, -4.70454f, 6.11489f,  -1.35429f,
    8.46660f,  -0.48650f, -3.45996f, 4.07882f,  6.81541f,  9.17308f,  -1.97050f, -5.76110f, 1.74471f,  1.27162f,
    0.37992f,  -3.93265f, 3.43787f,  5.80283f,  0.95588f,  4.29761f,  -6.69165f, 2.87665f,  -5.62054f, -6.31030f,
    -0.72317f, 2.72721f,  -6.34081f, -8.53630f, 7.06262f,  0.87856f,  7.72985f,  -3.86377f, -6.96312f, -1.58210f,
    -8.50367f, 8.34352f,  6.58261f,  -0.78997f, -4.05883f, -6.57532f, -1.53368f, -5.81707f, 8.21309f,  -2.28107f,
    -1.07931f, 6.87076f,  -5.64831f, 9.03000f,  5.74198f,  3.93601f,  5.72864f,  -4.49742f, 1.34126f,  -4.51461f,
    0.69773f,  4.08540f,  -0.47407f, 9.33771f,  -9.12955f, -0.41582f, 1.76603f,  -9.95335f, -2.19463f, -8.03589f,
    -5.28637f, 5.54113f,  0.66964f,  8.06367f,  3.42697f,  2.77308f,  -8.86805f, 6.72570f,  -2.39519f, 3.34673f,
    -2.54326f, -1.54055f, 5.62776f,  4.26711f,  -9.73248f, 5.78107f,  6.63174f,  -7.02314f, 2.96437f,  8.08580f,
    7.75526f,  -7.81702f, 9.53008f,  4.75763f,  2.66545f,  9.14802f,  -8.81257f, -5.60602f, -9.04797f, 5.89291f,
    -5.27688f, -9.09167f, -2.31738f, -3.26110f, -5.47175f, 4.52934f,  -7.04145f, -1.39708f, 0.76648f,  -8.86058f,
    6.67461f,  -6.77147f, 3.83488f,  7.93292f,  -8.44415f, -5.42389f, -1.18755f, 7.32417f,  9.59265f,  -7.95509f,
    5.62093f,  7.12622f,  5.64040f,  9.70391f,  7.89931f,  3.16333f,  0.09640f,  -6.19396f, -6.02146f, 5.45094f,
    -9.28257f, 1.30429f,  7.99676f,  1.18445f,  -4.00784f, -1.67790f, -3.01441f, 8.26108f,  0.79506f,  -1.41668f,
    -3.27432f, -6.23652f, 6.84399f,  4.54946f,  -8.84018f, -4.91627f, -6.75428f, 5.16635f,  2.13818f,  -2.12039f,
    8.50615f,  -0.48173f, -0.82237f, -3.47784f, -3.76297f, 6.28055f,  -2.94092f, -4.43520f, -6.33660f, -4.74115f,
    -4.94868f, 1.76024f,  -4.03274f, 7.67758f,  1.45710f,  8.71992f,  -1.17401f, 0.94716f,  -0.40519f, 2.31730f,
    7.70648f,  -9.08301f, -2.60225f, 3.06807f,  8.55542f,  8.51581f,  -6.03861f, 5.94079f,  -4.35557f, -0.50496f,
    -7.69884f, 1.28922f,  6.31896f,  -9.48662f, -7.08348f, -3.46687f, -7.50693f, -0.58527f, -2.44543f, 4.11910f,
    -7.57746f, 7.25402f,  -9.21561f, 9.73029f,  2.24737f,  -2.02344f, 6.47679f,  -6.02309f, -9.56763f, 6.85342f,
    2.48227f,  7.14803f,  9.90175f,  -3.56324f, -3.12123f, -7.39850f, -0.33606f, 7.16785f,  3.89968f,  -6.50857f,
    -3.56363f, -5.90069f, 6.72724f,  -1.14711f, -4.38121f, -3.80071f, -5.63624f, -3.85042f, 5.21815f,  -2.09133f,
    -3.02341f, 9.35707f,  -3.41354f, -5.53424f, 6.98790f,  8.44789f,  0.25365f,  7.98675f,  -9.72488f, 1.77071f,
    3.02028f,  2.25621f,  -9.62384f, -5.26989f, 9.18194f,  -9.40513f, -5.42840f, -5.80036f, 3.42547f,  6.38808f,
    -7.05080f, 1.16107f,  3.64027f,  -9.01784f, -4.97042f, -2.22902f, -0.96993f, 4.11411f,  -1.50448f, 9.37264f,
    -7.45777f, -2.76789f, -0.51130f, -0.64516f, -6.09574f, 7.19147f,  6.34692f,  -1.67457f, -4.69518f, -8.90410f,
    -5.24984f, 2.68163f,  1.49346f,  6.63707f,  4.64022f,  3.38275f,  9.58110f,  6.09008f,  0.75057f,  -0.43655f,
    -9.55913f, 6.07248f,  5.68667f,  -1.39139f, -8.26138f, -0.59398f, -5.68159f, -6.57423f, -6.83017f, 8.95342f,
    -3.55264f, -5.18688f, 4.03319f,  9.11997f,  3.20559f,  8.87284f,  0.23848f,  -9.19120f, 1.44016f,  -0.54968f,
    -8.31378f, 0.80618f,  3.63805f,  -2.99200f, 4.42686f,  5.58783f,  1.59461f,  -3.27937f, -4.83772f, -4.13061f,
    -8.75030f, -8.99610f, 9.12702f,  -3.83355f, -0.04116f, 5.80286f,  9.80364f,  -5.51030f, -5.28546f, 8.44179f,
    -5.61439f, -8.22297f, 5.64440f,  -7.04520f, 0.10237f,  -0.60731f, -8.78457f, 2.47468f,  8.79759f,  9.10771f,
    -2.59082f, -2.69320f, 7.91567f,  7.88814f,  1.50752f,  5.77965f,  1.63419f,  -9.32001f, -2.35408f, 2.25639f,
    -9.24492f, 2.41877f,  6.47009f,  0.73462f };
}  // namespace gpukerneltest
