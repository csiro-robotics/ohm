// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmGpuConfig.h"

#include <ohm/RaysQuery.h>

namespace ohm
{
struct RaysQueryDetailGpu;

/// A GPU implementation of the @c RaysQuery .
class ohmgpu_API RaysQueryGpu : public RaysQuery
{
protected:
  /// Constructor used for inherited objects. This supports deriving @p RaysQueryDetail into
  /// more specialised forms.
  /// @param detail pimple style data structure. When null, a @c RaysQueryDetail is allocated by
  /// this method.
  explicit RaysQueryGpu(RaysQueryDetailGpu *detail);

public:
  RaysQueryGpu();

  /// Destructor.
  ~RaysQueryGpu() override;

protected:
  bool onExecute() override;
  bool onExecuteAsync() override;
  void onReset(bool hard_reset) override;

  void cacheGpuProgram();
  void enqueueRegions();
  void finaliseBatch();
  void sync();

  /// Internal pimpl data access.
  /// @return Pimpl data pointer.
  RaysQueryDetailGpu *imp();
  /// Internal pimpl data access.
  /// @return Pimpl data pointer.
  const RaysQueryDetailGpu *imp() const;
};
}  // namespace ohm