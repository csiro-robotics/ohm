//
// author Kazys Stepanas
//
#ifndef DEBUG_IDS_H_
#define DEBUG_IDS_H_

namespace ohm
{
enum class Category
{
  kDefault,
  kMap,
  // Parent category for below.
  kPopulate,
  kRays,
  kFreeCells,
  kOccupiedCells,
  kInfo
};

enum Resource
{
  kMap = 1,
  kMapMesh = 1,  // OK to be the same as kMap. One is object, the other is resource.
};
}  // namespace ohm

#endif  // DEBUG_IDS_H_