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

| Layer             | Internal structure | Byte size | Description                                                                       |
| ----------------- | ------------------ | --------- | --------------------------------------------------------------------------------- |
| `occupancy`       | `float`            | 4         | Log odds, probabilistic occupancy value.                                          |
| `mean`            | `VoxelMean`        | 8         | Sub voxel mean positioning.                                                       |
| `traversal`       | `float`            | 4         | Voxel distance traversal used to support the density model.                       |
| `covariance`      | `CovarianceVoxel`  | 24        | Triangular square root matrix, voxel covariance storage.                          |
| `intensity`       | `IntensityMeanCov` | 8         | Intensity mean and covariance for traversability mode (NDT-TM)                    |
| `hit_miss_count`  | `HitMissCount`     | 8         | Number of hits and misses on a voxel for NDT-TM                                   |
| `touch_time`      | `uint32_t`         | 4         | Approximate, quantised last update time (ms) of a voxel (not supported for TSDF). |
| `incident_normal` | `uint32_t`         | 4         | Quantised, packed approximate normal of the average ray incident for a voxel.     |
| `tsdf`            | `VoxelTsdf`        | 8         | Truncated signed distance fields voxel (experimental).                            |
| `clearance`       | `float`            | 4         | Distance to nearest occupied voxel (experimental).                                |

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
