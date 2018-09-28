// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef LINEWALK_H_
#define LINEWALK_H_

#include <glm/glm.hpp>

#include <cassert>

namespace ohmutil
{
  /// A templatised, voxel based line walking algorithm. Voxels are accurately traversed from @p startPoint to
  /// @p endPoint, invoking @p walkFunc for each traversed voxel.
  ///
  /// The @p walkFunc is simply a callable object which accepts a @p KEY argument. Keys are provided in order of
  /// traversal.
  ///
  /// The templatisation requires @p funcs to provide a set of key manipulation utily functions. Specifically,
  /// the @p KEYFUNCS type must have the following signature:
  /// @code
  /// struct KeyFuncs
  /// {
  ///   // Query the voxel resolution along a particular axis. Axis may be { 0, 1, 2 } corresponding to XYZ.
  ///   double voxelResolutin(int axis) const;
  ///   // Convert from pt to it's voxel key. The result may be null/invalid
  ///   KEY voxelKey(const glm::dvec3 &pt) const;
  ///   // Check if key is a null or invalid key.
  ///   bool isNull(const KEY &key) const;
  ///   // Convert from key to the centre of the corresponding voxel.
  ///   glm::dvec3 voxelCentre(const KEY &key) const;
  ///   // Move the key by one voxel. The axis may be {0, 1, 2} correlating the XYZ axes respectively.
  ///   // The step will be 1 or -1, indicating the direction of the step.
  ///   void stepKey(KEY &key, int axis, int step) const;
  /// };
  /// @endcode
  ///
  /// @param walkFunc The callable object to invoke for each traversed voxel key.
  /// @param startPoint The start of the line in 3D space.
  /// @param endPoint The end of the line in 3D space.
  /// @param includeEndPoint Should be @c true if @p walkFunc should be called for the voxel containing
  ///     @c endPoint, when it does not lie in the same voxel as @p startPoint.
  /// @param funcs Key helper functions object.
  /// @return The number of voxels traversed. This includes @p endPoint when @p includeEndPoint is true.
  template <typename KEY, typename KEYFUNCS, typename WALKFUNC>
  size_t walkSegmentKeys(const WALKFUNC &walkFunc,
                         const glm::dvec3 &startPoint, const glm::dvec3 &endPoint,
                         bool includeEndPoint, const KEYFUNCS &funcs)
  {
    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    KEY startPointKey = funcs.voxelKey(startPoint);
    KEY endPointKey = funcs.voxelKey(endPoint);

    glm::dvec3 direction = glm::dvec3(endPoint - startPoint);
    double length = glm::dot(direction, direction);
    length = (length >= 1e-6) ? std::sqrt(length) : 0;
    direction *= 1.0 / length;

    if (funcs.isNull(startPointKey) || funcs.isNull(endPointKey))
    {
      return 0;
    }

    if (startPointKey == endPointKey)
    {
      if (includeEndPoint)
      {
        walkFunc(startPointKey);
      }
      return 1;
    }

    int step[3] = { 0 };
    glm::dvec3 voxel;
    double timeMax[3];
    double timeDelta[3];
    double timeLimit[3];
    double nextVoxelBorder;
    double directionAxisInv;
    size_t added = 0;
    KEY currentKey = startPointKey;

  //  debugOut = true;
    voxel = funcs.voxelCentre(currentKey);
  //  debugOut = false;

    // printf("Start point : %f %f %f\n", startPoint.x, startPoint.y, startPoint.z);
    // printf("End point : %f %f %f\n", endPoint.x, endPoint.y, endPoint.z);
    // printf("V: %f %f %f\n", voxel.x, voxel.y, voxel.z);
    // std::cout << currentKey << std::endl;

    // Compute step direction, increments and maximums along each axis.
    for (unsigned i = 0; i < 3; ++i)
    {
      if (direction[i] != 0)
      {
        directionAxisInv = 1.0 / direction[i];
        step[i] = (direction[i] > 0) ? 1 : -1;
        // Time delta is the ray time between voxel boundaries calculated for each axis.
        timeDelta[i] = funcs.voxelResolution(i) * std::abs(directionAxisInv);
        // Calculate the distance from the origin to the nearest voxel edge for this axis.
        nextVoxelBorder = voxel[i] + step[i] * 0.5 * funcs.voxelResolution(i);
        timeMax[i] = (nextVoxelBorder - startPoint[i]) * directionAxisInv;
        timeLimit[i] = std::abs((endPoint[i] - startPoint[i]) * directionAxisInv);// +0.5f * funcs.voxelResolution(i);
      }
      else
      {
        timeMax[i] = timeDelta[i] = std::numeric_limits<double>::max();
        timeLimit[i] = 0;
      }
    }

    int axis = 0;
    bool limitReached = false;
    while (!limitReached && currentKey != endPointKey)
    {
      walkFunc(currentKey);
      ++added;
      axis = (timeMax[0] < timeMax[2]) ? ((timeMax[0] < timeMax[1]) ? 0 : 1) : ((timeMax[1] < timeMax[2]) ? 1 : 2);
      limitReached = std::abs(timeMax[axis]) > timeLimit[axis];
      funcs.stepKey(currentKey, axis, step[axis]);
      timeMax[axis] += timeDelta[axis];
    }

    if (includeEndPoint)
    {
      walkFunc(endPointKey);
      ++added;
    }

    assert(added);
    return added;
  }


  /// A @c walkSegmentKeys() overload which passes @p includeEndPoint as @c true.
  /// @param walkFunc The callable object to invoke for each traversed voxel key.
  /// @param startPoint The start of the line in 3D space.
  /// @param endPoint The end of the line in 3D space.
  /// @param funcs Key helper functions object.
  /// @return The number of voxels traversed. This includes @p endPoint.
  template <typename KEY, typename KEYFUNCS, typename WALKFUNC>
  size_t walkSegmentKeys(const WALKFUNC &walkFunc,
                         const glm::dvec3 &startPoint, const glm::dvec3 &endPoint,
                         const KEYFUNCS &funcs)
  {
    walkSegmentKeys(walkFunc, startPoint, endPoint, true, funcs);
  }

  /// A @c walkSegmentKeys() overload which uses a default passes constructed @p KEYFUNCS object and sets
  /// @p includeEndPoint @c true.
  /// @param walkFunc The callable object to invoke for each traversed voxel key.
  /// @param startPoint The start of the line in 3D space.
  /// @param endPoint The end of the line in 3D space.
  /// @return The number of voxels traversed. This includes @p endPoint.
  template <typename KEY, typename KEYFUNCS, typename WALKFUNC>
  size_t walkSegmentKeys(const WALKFUNC &walkFunc,
                         const glm::dvec3 &startPoint, const glm::dvec3 &endPoint)
  {
    return walkSegmentKeys(walkFunc, startPoint, endPoint, true, KEYFUNCS());
  }
}

#endif // LINEWALK_H_
