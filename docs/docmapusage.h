// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef DOCMAPUSAGE_H
#define DOCMAPUSAGE_H

namespace ohm
{
  /*!

  @page docusage Occupancy map usage

  This section details general usage of an `OccupancyMap` including map generation and data access. The page is
  split into sections for CPU map access and GPU map generation.

  # CPU map usage

  The `OccupancyMap` class is used to contain the occupancy map data. This class is always used at some level
  regardless of how the map is generated or populated. For example, the `GpuMap` wraps an `OccupancyMap` to
  support populating the map in GPU, but the underlying map object is the same.

  The data within the map are stored in @ref MapChunk "chunks" which represent contigous blocks of voxel memory
  corresponding to fixed 3D regions of the map. Chunks are created as needed. This dense memory layout is what makes GPU
  update of the map possible. The additional overhead of using a dense memory layout is mitigated by using
  @ref VoxelBlockCompressionQueue "background compression" of voxel data when not in use.

  The map object stores the map data as well as the parameterisation of the map - for example the
  @ref OccupancyMap::occupancyThresholdProbability() "occupancy threshold" and
  @ref OccupancyMap::resolution() "resolution". The map object is also responsible for mapping between global
  coordinates and @ref Key "voxel keys" via the @ref OccupancyMap::voxelKey() "voxelKey()"" function.

  A map is created with a fixed @ref OccupancyMap::resolution() "resolution" and fixed chunk size by specifying the
  voxel extents on each axis for a chunk. The following code creates an empty occupancy map at a resolution of 0.25
  units per voxel and 32x32x32 voxels in each chunk.

  @code{.cpp}
  ohm::OccupancyMap map(0.25, glm::u8vec3(32));
  @endcode

  Data can be added to the may by using a @c RayMapper . Note that map object does not feature any methods for adding
  data to the map. The simplest mapper implemntation is the @c RayMapperOccupancy. Using this mapper, origin/sample
  pairs (rays) are added to the map with the sample voxel occupancy increased and the occupancy for voxels along the ray
  decreased. This is shown in the code snipped below.

  @code{.cpp}

  // A virtual utility class used in the examples to provide data for an occcupancy map.
  class DataProvider
  {
  public:
    virtual ~DataProvider() = default;
    // Get the next batch of rays to add to the map.
    // @param rays Populated with the next batch of origin/sample pairs (rays) to add to the map.
    // @return True while there are more rays to add.
    virtual bool nextBatch(std::vector<glm::dvec3> &rays) = 0;
  }

  void populateMap(DataProvider &provider)
  {
    ohm::OccupancyMap map(0.25);          // Create a map
    ohm::RayMapperOccupancy mapper(&map); // Create a mapper for the map.
    std::vector<glm::dvec3> rays;         // Ray set storage.

    // While data are available.
    while (provider.nextBatch(rays))
    {
      // Add data to the map.
      mapper.integrateRays(rays.data(), rays.size());
    }
  }
  @endcode

  Data can then be queried about invidisual voxels using a combination of @ref Key "voxel keys" and @c Voxel objects.
  The code below queries whether the voxel containing a given spatial position is occupied.

  @code{.cpp}
  bool isVoxelAtPositionOccupied(const ohm::OccupancyMap &map, const glm::dvec3 &position)
  {
    // First create a Voxel object which will reference the voxel occupancy data.
    // - We declare the template type as const float specifying that we expect voxel data to be a float per voxel.
    // - We use the map layout to resolve the map layer which contains occupancy data (more on this below).
    Voxel<const float> voxel_occupancy(&map, map.layout().occupancyLayer());

    // Validate that the Voxel is referencing a valid layer.
    if (!voxel_occupancy.isLayerValid())
    {
      // Layer is invalid - there is no occupancy layer.
      return false;
    }

    // Next resolve a key for the voxel containing position.
    const ohm::Key key = map.voxelKey(position);

    // Set the key for the Voxel object to reference.
    voxel_occupancy.setKey(key);

    // Ensure the key reference is valid within the map. An invalid voxel indicates the voxel has not been observed.
    if (!voxel_occupancy.isValid())
    {
      return false;
    }

    // Get the voxel occupancy value.
    float occupancy_value;
    // We use a read function to address C++ memory access standards.
    voxel_occupancy.read(&occupancy_value);

    // Finally check if the value is above the occupancy threshold.
    return occupancy_value >= map.occupancyThresholdValue();
  }
  @endcode

  The example above introduces the several concepts including the @ref MapLayout "map layout". The example
  demonstrates how to validate and access the data for a particular @ref MapLayer "voxel layer", in this case the
  occupancy layer. Note that there are convinience functions for this available in the @ref voxeloccupancy section.

  ## The MapLayout

  An @c OccupancyMap has an associated @c MapLayout which specifies the data available in the map and how that data
  are laid out. The @c MapLayout specifies a set of @c MapLayer objects and each layer identifies some data associated
  with each voxel. That is, each voxel may have multiple data types associated with it and for each data type the @c
  MapChunk stores a contiguous memory allocation for that data.

  By default, an @c OccupancyMap contains a @c float layer which stores the occupancy value for each voxel. Maps
  constructed with the @c kVoxelMean flag also contain a layer which tracks a subvoxel position which
  is represents an approximate mean value of all samples which have contributed to that voxel. This layer adds a
  @c VoxelMean structure for each voxel. The @c NdtMap also adds a @ref CovarianceVoxel "covariance layer". Additional
  user data may also be added using the @c MapLayout and @c MapLayer API.

  Voxel data should generally be accessed using the @c Voxel template class. This class is designed to
  handle referencing a particular voxel layer and to validate the data size against the voxel layer size. The @c Voxel
  template type is used to specify both the data type and read only vs read/write access.

  For example, the code below shows how to access the @c VoxelMean data including some invalid access patterns.

  @code{.cpp}
  void meanExample(ohm::OccupancyMap &map)
  {
    // Manually resolve the voxel layer index. This is also available in ohm::MapLayout::meanLayer() .
    // Resolve by name. This name is also available as ohm::default_layer::meanLayerName() .
    const MapLayer *layer = map.layout().layer("mean");

    // Validate layer.
    if (!layer)
    {
      return;
    }

    // Create a Voxel object for read/write access to the layer.
    Voxel<VoxelMean> mean_rw(&map, layer->layerIndex());
    // Create a Voxel object for read only access. Note the template type is `const`
    Voxel<const VoxelMean> mean_read(&map, layer->layerIndex());

    // Query the voxel at the origin. This cannot create the voxel chunk.
    mean_read.setKey(map.voxelKey(glm::dvec3(0)));
    if (mean_read.isValid())
    {
      // read and report the number of points contributing to the mean.
      VoxelMean mean;
      mean_read.read(&mean);
      std::cout << "The voxel at the origin contains " << mean.count << " samples" << std::endl;
    }
    else
    {
      std::cout << "The voxel at the origin has not been created" << std::endl;
    }

    // We copy the mean_rw key from the mean_read object. This can be more efficient in tight loops
    // as some data lookups can be skipped.
    // This call can create the MapChunk for the key, whereas mean_read.setKey() could not.
    mean_rw.setKey(mean_read);
    // The validity check can only fail if mean_rw.isLayerValid() is false. Conversely, it will always be valid so
  long as
    // the map and layer references are valid.
    if (mean_rw.isValid())
    {
      // read and report the number of points contributing to the mean.
      VoxelMean mean;
      mean_rw.read(&mean);
      std::cout << "The voxel at the origin contains " << mean.count << " samples" << std::endl;
      // Reset the number of samples to zero.
      mean.count = 0;
      mean_rw.write(mean);
    }
  }
  @endcode

  # NDT map

  The @c NdtMap is an extension of the @c OccupancyMap which adds normal distribution transforms
  semantics. This adds a covariance representation to each voxel which can be used to represent a "surfel" within the
  voxel. See that class for more details. The @c NdtMap should always be populated (in CPU) using the
  @c RayMapperNdt.

  # Iterating a map

  Once a map has been populated, it is possible to iterate the voxels in the map using either a using range based for
  loop over the map or using a @c OccupancyMap::iterator . In either case, this will iterate the @ref MapChunk "chunks"
  in the map in an undefined order and iterate each voxel within the chunk. Iteration of voxels within a chunk starts
  from the chunk's @c MapChunk::firstValidKey() which is maintained as the first voxel in the chunk memory which has
  been touched.

  Iterating with a range based for loop or deferencing the @c OccupancyMap::iterator provides yields a @c Key for the
  current voxel. The data associated with the voxel must be resolved using the @c Voxel template class. The
  @c OccupancyMap::iterator has additional, non-standard iterator functions which provide access to the target
  @c MapChunk and @c OccupancyMap . Below is an example of iterating a map.

  @code{.cpp}
  // A structure detailing some map statistics
  struct MapStats
  {
    // Smallest point count for an occupied voxel.
    unsigned min_point_count{0};
    // Largest point count for an occupied voxel.
    unsigned max_point_count{0};
    // Total number of samples contributing to the map.
    uint_64 total_samle_count{0};
    // Average point count for an occupied voxel.
    unsinged average_point_count{0};
    // Number of occupied voxels.
    unsigned occupied_voxel_count{0};
    // Number of free voxels.
    unsigned free_voxel_count{0};
  };

  MapStats collectMapStats(const ohm::OccupancyMap &map)
  {
    MapStats stats;

    // Setup voxel data.
    ohm::Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
    ohm::Voxel<ohm::VoxelMean> mean(&map, map.layout().meanLayer());

    ohm::VoxelMean mean_value;

    for (auto iter = map.begin(); iter != map.end(); ++iter)
    {
      // Set the key for all voxel data accessors in one call.
      // Note: passing the iterator to setVoxelKey() instead of the key is more efficient as the MapChunk can be copied
      // from the iterator. A range based for loop will miss this minor efficiency gain.
      ohm::setVoxelKey(iter, occupancy, mean);
      if (occupancy.isValid()) // Should always be true - we are only iterating known voxels.
      {
        if (ohm::isOccupied(occupancy))
        {
          // Increment occupied voxels.
          ++stats.occupied_voxel_count;
          mean.read(&mean_value);

          stats.total_samle_count += mean_value.count;
          stats.min_point_count =
            (stats.occupied_voxel_count > 0) ? std::min(stats.min_point_count, mean_value.count) : mean_value.count;
          stats.max_point_count = std::max(stats.min_point_count, mean_value.count);
        }
        else if (ohm::isFree(voxel))
        {
          ++stats.free_voxel_count;
        }
      }
    }

    // Finalise average.
    if (stats.occupied_voxel_count)
    {
      stats.average_point_count = unsigned(stats.total_sample_count / stats.occupied_voxel_count);
    }

    return stats;
  }
  @endcode

  # GPU map

  GPU support is implemented in the `ohmgpucuda` and `ohmgpuocl` libraries, using CUDA and OpenCL respectively. While
  these are technically optional libraries, they are the focus of the ohm innovation. These libraries have the same
  API backed by the associated GPU SDK. The SDK selection is forced at compile time and cannot be switched at runtime.

  The ohm GPU API introduces the @c GpuMap class, which is a both a wrapper for an @c OccupancyMap and
  the @c RayMapper implementation which should be used to update the map. The code below shows how to use the @c GpuMap
  to populate an @c OccupancyMap .

  @code{.cpp}
  void populateGpuMap(DataProvider &provider)
  {
    ohm::OccupancyMap map(0.1);   // Create a map
    ohm::GpuMap gpu_map(&map);    // Create a GPU map
    std::vector<glm::dvec3> rays; // Ray set storage.

    // While data are available.
    while (provider.nextBatch(rays))
    {
      // Add data to the map.
      gpu_map.integrateRays(rays.data(), rays.size());
    }

    // Synchronise GPU data back to the CPU map memory.
    gpu_map.syncVoxels();
  }
  @endcode

  Note the call to `gpu_map.syncVoxels()`. The @c GpuMap does not automatically sync GPU changes back to CPU memory.
  This call ensures data synchronsiation and allows read access to the map on CPU following this call.

  The @c GpuMap relies on a @c GpuCache which keeps a copy of relevant @c MapChunk data in GPU memory. This cache
  keeps recently accessed chunks in GPU memory and will move data back to CPU memory once the cache is full and new
  chunks need to be modified. Changes in CPU will mark the chunk as dirty and the @c GpuCache will upload the updated
  data from CPU, however there is no resolution mechanism for merging simultaneous changes on CPU and GPU.

  For optimal performance the number of rays given to each @ref GpuMap::integrateRays() "integrateRays()" call may
  need to be tuned to the current platform. Batch sizes of 2048 or 4096 are recommended. Small batch sizes will be
  slower than performing the same update in CPU.

  Note that the @c GpuMap implementation will update the @c VoxelMean for maps which have a @c VoxelMean layer.

  ## GPU NDT Map

  The @c GpuNdtMap extends the @c GpuMap adding the NDT semantics to the GPU update. This @c
  GpuNdtMap must be used in place of the @c GpuMap object if NDT semantics are required. The NDT update is notably
  more expensive than the base GPU update.

  @code{.cpp}
  void populateGpuNdtMap(DataProvider &provider)
  {
    ohm::OccupancyMap map(0.1);   // Create a map
    ohm::GpuNdtMap ndt_map(&map); // Create GPU map with NDT support
    std::vector<glm::dvec3> rays; // Ray set storage.

    // While data are available.
    while (provider.nextBatch(rays))
    {
      // Add data to the map.
      gpu_map.integrateRays(rays.data(), rays.size());
    }

    // Synchronise GPU data back to the CPU map memory.
    gpu_map.syncVoxels();
  }
  @endcode

  */
}  // namespace ohm

#endif  // DOCMAPUSAGE_H
