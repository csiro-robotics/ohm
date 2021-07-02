// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmGpuConfig.h"

#include "GpuMap.h"

#include <ohm/RaysQuery.h>

namespace ohm
{
struct RaysQueryDetailGpu;

/// A GPU implementation of the @c RaysQuery .
///
/// The GPU overheads mean that this will be slower than the CPU implementation for low counts. GPU begins to outperform
/// CPU when querying rays on the order of 2000 or more with significant benefits starting at 5000-10000 rays with ray
/// lengths approximately 100 times the voxel resolution. Doubling the ray length may require an order of magnitude
/// more sample rays to maintain GPU timing benefits.
///
/// The actual timing results will vary depending on GPU API, GPU architecture, voxel size, query ray lengths and map
/// environment.
class ohmgpu_API RaysQueryGpu : public RaysQuery
{
protected:
  /// Constructor used for inherited objects. This supports deriving @p RaysQueryDetail into
  /// more specialised forms.
  /// @param detail pimple style data structure. When null, a @c RaysQueryDetail is allocated by
  /// this method.
  explicit RaysQueryGpu(RaysQueryDetailGpu *detail);

public:
  /// Constructor. The map and rays must be set before using.
  RaysQueryGpu();

  /// Destructor.
  ~RaysQueryGpu() override;

protected:
  void onSetMap() override;
  bool onExecute() override;
  bool onExecuteAsync() override;
  void onReset(bool hard_reset) override;

  /// Synchronise GPU results
  void sync();

  /// Internal pimpl data access.
  /// @return Pimpl data pointer.
  RaysQueryDetailGpu *imp();
  /// Internal pimpl data access.
  /// @return Pimpl data pointer.
  const RaysQueryDetailGpu *imp() const;
};
}  // namespace ohm
