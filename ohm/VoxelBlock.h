// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXELBLOCK_H
#define VOXELBLOCK_H

#include "OhmConfig.h"

#include "Mutex.h"

#include <glm/fwd.hpp>
#include <glm/vec3.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>

namespace ohm
{
  class MapLayer;
  class VoxelBlockCompressionQueue;
  struct OccupancyMapDetail;

  /// A utility class used to track the memory for a dense voxel layer in a @c MapChunk. This class ensures voxel memory
  /// is uncompressed when requested and compressed using the background compression thread when no longer needed.
  ///
  /// Internally the @c VoxelBlock allocates or decompresses voxel memory for its associated @c MapLayer
  /// (@c layerInfo()) when @c retain() is called. It then maintains a reference count for the number of @c retain()
  /// calls ensuring uncompressed voxel data remain valid until all references are by calling @c release(). The block is
  /// then passed to the background compression thread when the last reference is released. The level of compression
  /// can be globally set using the static @c setCompressionControls() function.
  ///
  /// Typically, @c retain() and @c release() should not be called directly. Instead user code should use the @c Voxel
  /// data access object. This object manages multiple aspects of voxel data access including ensuring @c retain() and
  /// @c release() are called as needed.
  ///
  /// When a @c VoxelBlock is pushed onto the background compression queue, it is assigned a release time before which
  /// it cannot be compressed. The background thread cannot compress the block before this time elapses. The time is
  /// updated on each release, ensuring that the block is not compressed until some time after the most recent release.
  /// This is currently based on wall clock time and may change in future.
  ///
  /// The block also deals with cases where the background thread is in the process of compressing the voxel data while
  /// the reference count is non zero or when the background thread is processing the block when the map chunk is
  /// deleted.
  class ohm_API VoxelBlock
  {
    friend VoxelBlockCompressionQueue;

  public:
    /// Deleter object for correctly releasing a @c VoxelBlock
    class ohm_API Deleter
    {
    public:
      /// Deletion operator calling through to @c VoxelBlock::destroy()
      /// @param block The object to destroy.
      inline void operator()(VoxelBlock *block) { block->destroy(); }
    };

    /// Unique pointer definition for a @c VoxelBlock . Ensures correct custom deletion.
    using Ptr = std::unique_ptr<VoxelBlock, Deleter>;

    /// The mutex used to protect threaded access.
    using Mutex = ohm::SpinMutex;
    /// The clock to use for tracking release time.
    using Clock = std::chrono::steady_clock;

    /// Flags marking the @c VoxelBlock status.
    enum Flag
    {
      /// Memory buffer currently holds uncompressed voxel data.
      kFUncompressed = (1 << 0),
      /// Block is queued for compression.
      kFCompressionQueued = (1 << 1),
      /// Block is to be deleted. Only set when the block should be deleted but is currently on the compression thread.
      kFMarkedForDeath = (1 << 2)
    };

    /// Compression level options
    enum CompressionLevel
    {
      /// Use the fastest compression (default).
      kCompressFast,
      /// Used balanced size/speed compression
      kCompressBalanced,
      /// Use maximum compression (slowest)
      kCompressMax
    };

    /// Compression type.
    enum CompressionType
    {
      /// ZLib deflate.
      kCompressDeflate,
      /// GZip compression.
      kCompressGZip
    };

    /// Static compression controls.
    struct CompressionControls
    {
      /// Minimum initial buffer size used when compressing a voxel block.
      unsigned minimum_buffer_size = 0;
      /// Voxel block compression level.
      CompressionLevel compression_level = kCompressFast;
      /// Voxel block compression technique.
      CompressionType compression_type = kCompressDeflate;
    };

    /// Get the current compression controls.
    /// @param[out] controls A non-null pointer to the structure in which current compression settings are returned.
    static void getCompressionControls(CompressionControls *controls);
    /// Set the voxel block compression controls. Should only be called before maps are created and voxel compression
    /// begins.
    /// @param controls New compression settings.
    static void setCompressionControls(const CompressionControls &controls);

    /// Create a voxel block within the given @p map for the given @p layer_index.
    ///
    /// @note At the time of the call, there are cases where the @p layer will not match the layout in @p map. This
    /// occurs when updating the layout. The calling code must rectify this situation before other code may use the
    /// @c VoxelBlock .
    /// @param map Details of the occupancy map to which the block belongs.
    /// @param layer The @p MapLayer which the voxel block represents.
    VoxelBlock(const OccupancyMapDetail *map, const MapLayer &layer);

  private:
    /// Hidden destructor for dealing with the processing queue safely.
    /// Use destroy().
    ~VoxelBlock();

  public:
    /// Delete and destroy this object; use in place of <tt>delete voxel_block</tt>.
    ///
    /// This will either immediately delete the @c VoxelBlock or mark it for deletion by the compression thread.
    /// As such, the object instance should no longer be used after making this call, but it may persist until the
    /// background thread releases it.
    void destroy();

    inline static void destroy(VoxelBlock *block) { block->destroy(); }

    /// Size of a single voxel in the map.
    /// @return The size of a voxel in bytes.
    size_t perVoxelByteSize() const;

    /// Uncompressed data size.
    /// @return The uncompressed size of the voxel map in bytes.
    inline size_t uncompressedByteSize() const { return uncompressed_byte_size_; }

    /// Retain the uncompressed voxel memory until a corresponding @c release() call. Not recommended; use
    /// @c voxelBuffer().
    ///
    /// This call may block while the voxel memory is uncompressed or allocated an initialised.
    void retain();

    /// Release the uncompressed voxel memory until a corresponding @c release() call. Not recommended; use
    /// @c voxelBuffer().
    void release();

    /// Compress the voxel data into @p compression_buffer. Writes the current voxel bytes when already compressed.
    ///
    /// @note This may initialise the buffer if empty, so that this method can be used for serialisation of the map to
    ///   disk.
    /// @param[in,out] compression_buffer Buffer to write compression data into. Resized to the compressed data size.
    void compressInto(std::vector<uint8_t> &compression_buffer);

    /// Query the time after which the background thread may compress the @c VoxelBlock.
    /// @return The time after which the block may be compressed.
    const Clock::time_point releaseAfter() const;

    /// @internal
    /// Direct access to the voxel bytes. Should be retained first.
    /// @return Voxel bytes.
    uint8_t *voxelBytes();

    /// @internal
    /// Direct access to the voxel bytes. Should be retained first.
    /// @return Voxel bytes.
    const uint8_t *voxelBytes() const;

    /// @internal
    /// Internal function for updating the layer index value when remapping layouts.
    void updateLayerIndex(unsigned layer_index);

  private:
    /// Check if compression is required. Needs compression if compression is enabled and buffer is currently
    /// uncompressed. Mutex is not locked.
    bool needsCompression() const;

    /// Queue compression of voxel data. Should only be called when @c needsCompression() is true.
    void queueCompression(std::unique_lock<Mutex> &guard);

    /// Perform the compression operation into @p compression_buffer without locking the mutex . This is called from
    /// @c compressInto() after locking the @c access_guard_ .
    ///
    /// @param compression_buffer The buffer to compress into. Final size will exactly match the compressed data size
    ///   though the capacity may be larger.
    /// @return True if compressio into @p compression_buffer succeeded.
    bool compressUnguarded(std::vector<uint8_t> &compression_buffer);
    /// Decompress voxel data into @p expanded_buffer without locking the mutex. This is called from @c retain() after
    /// the mutex is locked.
    /// @param expanded_buffer The buffer to populate with uncompressed data.
    /// @return True on successfully decompressing.
    bool uncompressUnguarded(std::vector<uint8_t> &expanded_buffer);
    /// Initialise the given buffer to uncompressed voxel data. The voxel data is cleared to the appropriate pattern
    /// for the voxel layer.
    /// @param expanded_buffer The buffer to initialised.
    /// @param layer The layer used to initialise the memory. Must be explicitly passed to handle map layout changes.
    void initUncompressed(std::vector<uint8_t> &expanded_buffer, const MapLayer &layer);
    /// Swap the voxel bytes with the given compressed voxel bytes, but only if there are currently no retained
    /// references. This is for use byte the @c VoxelBlockCompressionQueue.
    /// @param compressed_voxels The compressed voxel data.
    /// @return True on success when there are no retained references.
    bool setCompressedBytes(const std::vector<uint8_t> &compressed_voxels);

    /// Voxel data.
    ///
    /// This data can be in one of three states:
    /// 1. Empty implying no changes have been made from the default initialised values.
    /// 2. Uncompressed when not empty and `flags_ & kFUncompressed` set.
    /// 3. Compressed when not emtpy and `flags_ & kFUncompressed` clear.
    std::vector<uint8_t> voxel_bytes_;
    /// Data access mutex
    mutable Mutex access_guard_;
    /// Number of oustandting @c retain() calls. Cannot be compressed while no zero.
    std::atomic_uint32_t reference_count_{ 0 };
    /// Block status @c Flag values.
    std::atomic_uint32_t flags_{ 0 };
    /// Timepoint after which the block may be compressed.
    Clock::time_point release_after_;
    /// The owning occupancy map detail.
    const OccupancyMapDetail *map_ = nullptr;
    /// The index into the @c MapLayout represented by this voxel data.
    unsigned layer_index_ = 0;
    /// Byte size of this voxel block when uncompressed.
    size_t uncompressed_byte_size_ = 0;
  };

  inline uint8_t *VoxelBlock::voxelBytes() { return voxel_bytes_.data(); }

  inline const uint8_t *VoxelBlock::voxelBytes() const { return voxel_bytes_.data(); }
}  // namespace ohm

#endif  // VOXELBLOCK_H
