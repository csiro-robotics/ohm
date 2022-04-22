<!--
Copyright (c) 2020
Commonwealth Scientific and Industrial Research Organisation (CSIRO)
ABN 41 687 119 230

Author: Kazys Stepanas
-->

<!-- Use details section to partly hide doxygen specific details of the page. -->
<details><summary></summary>
@page docvoxellayers Built in voxel layers
</details>

# Built in voxel layers

There are a number of "built in" or "default" supported OHM voxels layers. These layers are known within the OHM library and in some cases are in algorithms unrelated to the voxel calculations which generated them. For example, occupancy and TSDF layers may both be used to identify the `MapChunk::first_valid_index`, which marks the first observed voxel in a `MapChunk`. The default OHM layers are listed below.

| Layer               | Internal structure      | Byte size | Description                                                                       |
| ------------------- | ----------------------- | --------- | --------------------------------------------------------------------------------- |
| `occupancy`         | `float`                 | 4         | Log odds, probabilistic occupancy value.                                          |
| `mean`              | `VoxelMean`             | 8         | Sub voxel mean positioning.                                                       |
| `traversal`         | `float`                 | 4         | Voxel distance traversal used to support the density model.                       |
| `covariance`        | `CovarianceVoxel`       | 24        | Triangular square root matrix, voxel covariance storage.                          |
| `intensity`         | `IntensityMeanCov`      | 8         | Intensity mean and covariance for traversability mode (NDT-TM)                    |
| `hit_miss_count`    | `HitMissCount`          | 8         | Number of hits and misses on a voxel for NDT-TM                                   |
| `touch_time`        | `uint32_t`              | 4         | Approximate, quantised last update time (ms) of a voxel (not supported for TSDF). |
| `incident_normal`   | `uint32_t`              | 4         | Quantised, packed approximate normal of the average ray incident for a voxel.     |
| `tsdf`              | `VoxelTsdf`             | 8         | Truncated signed distance fields voxel (experimental).                            |
| `clearance`         | `float`                 | 4         | Distance to nearest occupied voxel (experimental).                                |
| `secondary_samples` | `VoxelSecondarySamples` | 8         | Stats on secondary samples (lidar dual returns) falling in a voxel.               |

## Example memory requirements

Some algorithms use layers independently - e.g., pure occupancy - while others algorithms introduce layer codependencies - e.g., NDT-OM requires occupancy, mean and covariance. This can impose significant memory burdens in large maps. For offline processing, the `VoxelBlockCompressionQueue` can be enabled to compress least recently used voxel blocks, based on memory watermark levels.

The expectation for realtime processing is that only a local map is maintained, and may need to be sized such that all layers can fit in the `GpuCache`. Below are some examples of approximate memory requirements based on the local region of interest, voxel resolution and voxel algorithm. Values are given in mibibytes (MiB) or gibibytes (GiB).

| Dimensions (m) | Resolution |    OCC |    NDT |   TSDF |
| -------------- | ---------: | -----: | -----: | -----: |
| 20x20x10       |       0.2m | 1.9MiB |  17MiB | 3.8MiB |
| 20x20x20       |       0.2m | 3.8MiB |  34MiB | 7.6MiB |
| 20x20x10       |       0.1m |  15MiB | 137MiB |  31MiB |
| 20x20x20       |       0.1m |  31MiB | 275MiB |  61MiB |
| 40x40x20       |       0.1m | 122MiB | 1.1GiB | 244MiB |
| 40x40x40       |       0.1m | 244MiB | 2.1GiB | 488MiB |


# Layer population

This section describes how to populate each of the default layers. Using a layer first requires the layer is present in a map.

## Creating layers

Layers can be created:

1. using `MapFlags` on construction,
2. using `OccupancyMap` functions which add known layers, or
3. updating the layout manually.

`MapFlags` contains a value for all of the default layers which can be composed as a constructor argument:

```c++
// Construct a map with occupancy, mean, traversal and TSDF layers.
ohm::OccupancyMap map(0.1, ohm::MapFlag::kVoxelMean | ohm::MapFlag::kTraversal | ohm::MapFlag::Tsdf);
```

Note the occupancy map layer is implied, except with TSDF is the *only* layer requested. That is, the occupancy layer is created when;

- No `MapFlag` layer values are specified.
- An `MapFlag` layer other than `MapFlag::Tsdf` is specified regardless of whether `MapFlag::Tsdf` is specified.

However, the occupancy layer is not created when only `MapFlag::Tsdf` is specified.

The following functions of `OccupancyMap` may be used to add layers after a map has been created:

- `OccupancyMap::addVoxelMeanLayer()`
- `OccupancyMap::addTraversalLayer()`
- `OccupancyMap::addTouchTimeLayer()`
- `OccupancyMap::addIncidentNormalLayer()`
- `OccupancyMap::addLayer()` - generic helper

These functions perform similar code which is equivalent to updating the layout manually as shown below.

```c++
ohm::OccupancyMap map(0.1);

// Add tsdf layer.
if (map.layout().layerIndex(ohm::default_layer::tsdfName()) == -1)
{
  // Layer not already present.
  // Copy the layout.
  ohm::MapLayout layout = map.layout();
  // Add the Tsdf layer.
  ohm::addTsdf(layout);
  // Apply the updated layout back to the map.
  map.updateLayout(layout);
}
```

## Populating layers

Most layers are populated using the either `RayMapperOccupancy` and `RayMapperNdt`, or the equivalent GPU implementations - `GpuMap` and `GpuNdtMap` respectively. The table below shows which layers are populated by which `RayMapper` implementations. Some layers are optional and are populate if present, other layers are required. Using a `RayMapper` without the required layers present leads to undefined behaviour. Optional layers are shown in square brackets `[]` Note that `GpuMap` is a derivation of `RayMapper`.

| `RayMapper`                         | Affected layers                                                                               |
| ----------------------------------- | --------------------------------------------------------------------------------------------- |
| `RayMapperOccupancy`                | `occupancy`, `[mean]`, `[traversal]`, `[touch_time]`, `[incident_normal]`                     |
| `GpuMap`                            | Equivalent to `RayMapperOccupancy`                                                            |
| `RayMapperNdt` occupancy model      | `occupancy`, `mean`, `covariance,`, `[traversal]`, `[touch_time]`, `[incident_normal]`        |
| `RayMapperNdt` traversability model | As occupancy model, plus: `intensity`, `hit_miss_count`. Requires rays with intensity values. |
| `GpuNdtMap`                         | Equivalent as `RayMapperNdt`                                                                  |
| `RayMapperTsdf`                     | `tsdf`                                                                                        |
| `GpuTsdfMap`                        | Equivalent to `RayMapperTsdf`                                                                 |
| `RayMapperSecondarySample`          | `secondary_samples` Has no GPU equivalent.                                                    |

Note there is no `RayMapper` which affects the `clearance` layer. Instead, use `ClearanceProcess` in `ohmgpu`. This is an experimental feature and is not considered sufficiently performance for general use.
