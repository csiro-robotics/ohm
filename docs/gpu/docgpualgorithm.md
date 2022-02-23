<!--
Copyright (c) 2022
Commonwealth Scientific and Industrial Research Organisation (CSIRO)
ABN 41 687 119 230

Author: Kazys Stepanas

-->

<!-- Use details section to partly hide doxygen specific details of the page. -->
<details><summary></summary>
@page docgpualgorithm GPU algorithm overview
</details>

# GPU algorithm overview

There are two parts of interest to highlight in the high level GPU occupancy algorithm;

- GPU thread algorithm
- CPU cache management

# GPU thread algorithm

OHM GPU threads are used such that there is one thread per input ray. Each GPU thread traces its voxels using a 3D
adaptation of <em>"A Fast Voxel Traversal Algorithm for Ray Tracing", John Amanatides and Andrew Woo</em>. This
algorithm steps a single voxel on each iteration selecting the step direction based on the largest remaining distance
to cover on each axis. Using this algorithm on GPU introduces contention when updating voxels, which is resolved using
atomic Compare and Swap (CAS) semantics.

Below is a high level pseudo code listing of the OHM GPU ray algorithm.

```cpp
struct float3
{
  float x, y, z;
};

struct GpuKey
{
  short region_key[3];
  unsigned char voxel_key[4]; // Index 3 is used as a marker bit (not show below).
};

// voxel_occupancies : Voxels for the regions intersecting the input rays
// rays : Start/end point pairs for the sample rays to process
// occupied_occupancy_adjustment : occupancy adjustment to make for the end voxel.
// free_occupancy_adjustment : occupancy adjustment to make for voxels other than the end voxel.
void traceRay(float *voxel_occupancies, float3 *rays,
              float occupied_occupancy_adjustment, float free_occupancy_adjustment)
{
  // Initialise the ray
  float3 ray_start = rays[thread_index * 2];
  float3 ray_end = rays[thread_index * 2 + 1];
  GpuKey end_voxel = getVoxel(ray_end);
  GpuKey start_voxel = getVoxel(ray_start);
  GpuKey current_voxel = start_voxel;

  // Walk the ray
  bool done = false;
  while (!done)
  {
    // Calculate the occupancy adjustment.
    float occupancy_adjustment = 0;
    if (current_voxel == end_voxel)
    {
      occupancy_adjustment = occupied_occupancy_adjustment;
    }
    else
    {
      occupancy_adjustment = free_occupancy_adjustment;
    }

    // Update the value using CAS, looping to avoid contention.
    // Note: the implementation imposes a hard limit on the maximum number of attempts to be made.
    bool updated = false;
    while (!updated)
    {
      // Note: the implementation of voxelIndex() is quite intricate (see below)
      int voxel_index = voxelIndex(current_voxel, voxel_occupancies)
      float expected_value = voxel_occupancies[voxel_index];
      float new_value = expected_value + occupancy_adjustment;
      updated = atomicCAS(&voxel_occupancies[voxel_index], expected_value, new_value);
    }

    // Walk the next voxel.
    done = current_voxel == end_voxel;
    current_voxel = walkNextVoxel(current_voxel, start_voxel, end_voxel);
}
```

The actual implementation differs significantly in its details - for example OHM supports reverse ray tracing while
the listing shows forward ray tracing and more variables are required to track the ray tracing. Another hidden detail is
that `voxelIndex()` appears to be a trivial operation in the listing above, but its actual implementation is fairly
complex. The CPU uploads voxel regions required for the update into `voxel_occupancies`. This is a single, flat array of
voxels, but contains data for multiple regions, though each region's voxels are in a contiguous block. To resolve
`current_voxel` key to an index, we must first calculate the offset of `current_voxel->region_key` into
`voxel_occupancies` using a lookup table, which is traversed using a linear search, then the `current_voxel->voxel_key`
can be simply converted from a 3D index into a 1D index. In practice, we cache the last accessed `region_key` and
corresponding offset to reduce the number of linear searches performed as rays will generally access multiple voxels
from the same region in sequence.

NDT uses a more involved calculation for the `free_occupancy_adjustment` and defers the `occupied_occupancy_adjustment`
to a second phase. The second, occupancy phase is required to avoid contention when updating a voxel's covariance
matrix. While the occupancy value is independent and can be updated using CAS, the covariance matrix consist of
multiple, codependent values which must be updated together.

The NDT occupancy phase launches one thread per voxel ray as well, however, rays are pre-sorted (on CPU) such that all
rays ending at the same sample voxel are collated. Thus multiple NDT occupancy threads may be launched for a single end
voxel (one per sample), however, only the first thread is allowed to update the voxel occupancy, covariance matrix, mean
or other values. The remaining threads targetting the same voxel are essentially wasted.

While this is inefficient, this has proven effective in updating NDT occupancy in GPU without needing to involve the
CPU for this phase.

Note that the earliest implementation of OHM occupancy launched one thread per voxel in a region with each voxel
checking which rays intersect it. This proved very slow as most GPU threads performed no work. While the ray solution
introduces contention and prevents the use of local memory caching, it is significantly faster than the per voxel thread
algorithm and significantly faster than the single threaded CPU algorithm.

# CPU cache management

Before updating voxels in GPU, the CPU must ensure that the required voxel regions are uploaded to the GPU. The regions
to be uploaded are identified by performing the same line walking algorithm for each ray in CPU, however, we do so at
the much coarser region resolution. Most rays only touch a handful of regions. The intersected regions are added to the
appropriate `GpuLayerCache`.
