# Performance Notes on Calculating Voxel Clearance Values

Here we note the techniques explored to general voxel clearance values on GPU and the associated performance impacts. The end goal has been to maximise performance whilst maintaining reasonable accuracy. A voxel's clearance value is the distance to its nearest occupied neighbour (or unknown neighbour depending on flags) within the specified search range.

## Techniques

Three primary techniques have been evaluated:

- Neighbour propagation
- Open list flood fill
- ROI propagation (Region of Interest propagation)


### Neighbour Propagation

This was the first technique and is essentially an iterative flood fill technique with no voxel contention. The region of interest (ROI) is defined as the map chunk/region we wish to generate clearance values for. This is padded by an appropriate number of voxels to reach the desired search range and the padded extents define the GPU working dimensions. Two buffers of voxels sized to the padded region are maintained and alternated to ensure new data do not overwrite old. The algorithm then proceeds:

1. Seed the first buffer with each voxel writing its own 3D index within the working region and 0 if it's not an obstruction (is free) or 1 if it is an obstruction (occupied or unknown and unknown flagged as obstacles).
2. Propagate obstacles with each voxel inspecting its face neighbours and selecting the nearest obstruction. Keep current value if it is a closer obstruction.
3. Iterate an appropriate number of times to fill out to the search range.
    1. Alternate read/write buffers each time.
4. Migrate results back to the CPU.

Pros:

- No contention
- Fixed iterations based on search range
- Fixed performance regardless of data content

Cons:

- Scalability: higher range searches make for higher GPU usage.
- High border overhead

### Open List Flood Fill

The next technique attempted. There an open list is generated from each obstructing voxel at processed at each iteration. This continued until the open list is empty. The ROI is padded just as in Neighbour Propagation and the padded region defines the working region on GPU.

1. Seed the working voxels similar to neighbour propagation, however the coordinates written are relative to the target voxel.
  1. Only every writes (0, 0, 0, 0) or (0, 0, 0, 1)
  2. Each obstruction pushes its face neighbours into an open list.
2. Process the open list, writing new obstructions to the open list voxels.
  1. Expand the open list with neighbours currently tracking farther obstacles or currently clear and within the search range.
3. Iterate on the open list until empty (or hard capped).

Pros:

- Reduced memory usage: working memory uses char4 rather than int4.
  - Must use 32-bit data type to support atomic CAS operations
- Lower GPU load: can fix number of workers
- Better scalability
- Less GPU thread overhead

Cons:

- CPU needed to determine whether another iteration is required and the number of workers.
- Higher number of GPU invocations, especially in worst case scenario
- Contention on writing to GPU nodes
- Limited search range because of char4 usage.

### ROI Propagation

A variant of Neighbour Propagation where we only have propagation iterations operate on the true ROI, not the padded region. This reduces the number of GPU threads to a fixed number and limits the maximum number of iterations required to cover the search range. Obstacles from outside the ROI, but within the search range have a separate seeding cycle which has memory contention, but greatly reduces the amount of work done. Uses alternating buffers just like Neighbour Propagation.

1. Seed the ROI (not padded) with each voxel writing *relative* coordinates to its nearest obstruction and flagging if it is an obstruction.
    1. Only every writes (0, 0, 0, 0) or (0, 0, 0, 1)
2. Seed obstacles from outside the ROI into nearest border voxels of the ROI
    1. Write to the nearest ROI border voxel and its face neighbour border voxels.
    2. This part has contention.
3. Propagate obstructions by having each ROI voxel inspect its face neighbours (and itself) and choose the closest obstruction.
4. Iterate an appropriate number of times to fill out to the search range or cover ROI from end to end (whichever is smaller)
    1. Alternate read/write buffers each time.
5. Migrate results back to the CPU.

Pros:

- Hybrid approach
- Far less work to do => best performance
- Reduced memory usage: working memory uses char4 rather than int4.
  - Must use 32-bit data type to support atomic CAS operations
- Fixed maximum number of iterations
- Reduce cost of scaling search range

Cons:

- Contention during seeding.
- Limited search range because of char4 usage.
- Seeding effects a down sampling which risks data loss.
  - This has been mitigated by seeding to ROI border neighbours as well as the closest ROI border voxel.

## Technique Evaluation

Evaluation using QCAT tower map data from 30-40s.

- Map resolution: 0.25
- Region size: (32, 32, 32) => (8m, 8m, 8m)

Technique             | Range | Total Time  | Average GPU Time/Region
--------------------- | ----- | ----------- | -----------------------
Neighbour propagation | 5m    | 76.6s       | 60.0ms
Neighbour propagation | 7.75m | 229.8s      | 191.7ms
Open list             | 5m    | 72.7s       | 60.7ms
Open list             | 7.75m | 108.0s      | 90.8ms
ROI propagation       | 5m    | 21.6s       | 18ms
ROI propagation       | 7.5m  | 30s         | 25ms
ROI propagation       | 15.5m | 78.5s       | 65.4ms
