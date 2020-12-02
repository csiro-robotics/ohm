# TODO

- The default hit/miss and min/max for ohmpop are not tuned for NDT (based on VLP-16 input)
  - Max is certainly too low and will result in erosion
- Revisit GPU update to use VoxelOccupancy.h
- GPU optimisation pass. Since memory access is so irregular, best to look at avoiding branching
  - Look for effects of different GPU hardware
  - Look for differences between OpenCL/CUDA
- Convert to RAII (new/delete have been targetted)
- Improve background compression. Replace timebased to compress less frequently with a target high watermark

## Multilayer heightmap generation

- Change floodfill approach to perform layered generation
  - Tightly bound? That is only floodfill always generates a layered map and is the only way to do so?
- Changes to support layered genertation
  - Closed list maintenance (visited list) needs to change to support all revisiting at different heights
  - Heightmap output needs to support multiple heights
  - Seeding may sufferambiguities. Aim to handle by using different logic for the initial key: prefer below the reference point.
