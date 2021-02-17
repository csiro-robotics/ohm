# TODO

- The default hit/miss and min/max for ohmpop are not tuned for NDT (based on VLP-16 input)
  - Max is certainly too low and will result in erosion
- Revisit GPU update to use VoxelOccupancy.h
- GPU optimisation pass. Since memory access is so irregular, best to look at avoiding branching
  - Look for effects of different GPU hardware
  - Look for differences between OpenCL/CUDA
- Convert to RAII (new/delete have been targetted)
- Improve background compression. Replace timebased to compress less frequently with a target high watermark

