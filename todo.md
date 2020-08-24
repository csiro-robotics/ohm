# TODO

- The default hit/miss and min/max for ohmpop are not tuned for NDT (based on VLP-16 input)
  - Max is certainly too low and will result in erosion
- Revisit GPU update to use VoxelOccupancy.h
- GPU optimisation pass. Since memroy access is so irregular, best to look at avoiding branching
  - Look for effects of different GPU hardware
  - Look for differences between OpenCL/CUDA
- Revisit default batch size for GPU processing, especially for NDT
  - This hasn't been checked for a while, but can have an impact on performance as we keep the GPU more efficently busy
- Optimise Heightmap genertion to avoid Voxel interface
