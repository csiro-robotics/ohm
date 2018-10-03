// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// Copyright (c) 2018
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMVOXELLAYOUT_H
#define OHMVOXELLAYOUT_H

#include "OhmConfig.h"

#include "DataType.h"

#include <cinttypes>

namespace ohm
{
  struct VoxelLayoutDetail;

  /// Common implementation for accessing the voxel structure within a @c MapLayer.
  ///
  /// This class is setup to define the same implementation for a writable @c VoxelLayout and a read only
  /// @c VoxelLayoutConst.
  ///
  /// See @c MapLayout for more details.
  ///
  /// @tpara T Either <tt>VoxelLayoutDetail</tt> or <tt>const VoxelLayoutDetail</tt> from the private API.
  template <typename T>
  class VoxelLayoutT
  {
  public:
    /// Create a new layout structure around the given data.
    /// @param detail The @c VoxelLayoutDetail.
    VoxelLayoutT(T *detail);

    /// Copy constructor supporting casting from non-const to const @c VoxelLayoutDetail.
    /// @param detail The @c VoxelLayoutDetail.
    template <typename U>
    inline VoxelLayoutT(U *detail = nullptr) : detail_(detail) {}

    /// Copy constructor.
    /// @param other Object to shallow copy.
    VoxelLayoutT(const VoxelLayoutT<T> &other);
    /// Move constructor.
    /// @param other Object to move.
    VoxelLayoutT(VoxelLayoutT<T> &&other) noexcept;

    /// Query the name of the member at @p memberIndex.
    /// @param member_index The index of the member to query.
    /// @return The member name or null if @p memberIndex is out of range.
    const char *memberName(size_t member_index) const;

    /// Query the byte offset to the member at @p memberIndex.
    /// @param member_index The index of the member to query.
    /// @return The byte offset to @p memberIndex or zero if out of range.
    size_t memberOffset(size_t member_index) const;

    /// Query the data type of the member at @p memberIndex.
    /// @param member_index The index of the member to query.
    /// @return The @c DataType of @p memberIndex or @c DataType::Unknown if out of range.
    DataType::Type memberType(size_t member_index) const;

    /// Query the byte size to the member at @p memberIndex.
    /// @param member_index The index of the member to query.
    /// @return The byte size to @p memberIndex or zero if out of range.
    size_t memberSize(size_t member_index) const;

    /// Query the desired clear value for the member at member at @p memberIndex.
    /// @param member_index The index of the member to query. Must be in range [0, @c memberCount()).
    /// @return The clear value for @p memberIndex.
    uint64_t memberClearValue(size_t member_index) const;

    /// Get a pointer to the given @p memberIndex given a base voxel pointer at @p mem.
    /// @param member_index The index of the member to query. Must be in range [0, @c memberCount()).
    /// @param mem The address of the voxel data.
    /// @return A pointer to <tt>mem + @c memberOffset(memberIndex)</tt>.
    const void *memberPtr(size_t member_index, const void *mem) const;
    /// @overload
    void *memberPtr(size_t member_index, void *mem) const;

    /// Query the size of the voxel structure defined by this object.
    /// @return The byte size of the defined voxel structure.
    size_t voxelByteSize() const;

    /// Query the number of registered data members.
    /// @retunrn The number of members.
    size_t memberCount() const;

    /// Get a pointer to a voxel member as a specific data type @c D.
    /// 
    /// Validates that the size of @c D matches the @c memberSize().
    ///
    /// @tparam D The target data type.
    /// @param member_index The index of the member to query. Must be in range [0, @c memberCount()).
    /// @param mem The address of the voxel data.
    /// @return A pointer to the member of @p mem cast as @c D.
    template <typename D>
    inline const D *memberData(size_t member_index, const void *mem) const
    {
      if (sizeof(D) == memberSize(member_index))
      {
        return reinterpret_cast<const D *>(memberPtr(member_index, mem));
      }
      return nullptr;
    }

    /// @overload
    template <typename D>
    inline D *memberData(size_t member_index, void *mem) const
    {
      if (sizeof(D) == memberSize(member_index))
      {
        return reinterpret_cast<D *>(memberPtr(member_index, mem));
      }
      return nullptr;
    }

    /// Internal data access.
    /// @return Internal details.
    inline T *detail() const { return detail_; }

  protected:
    T *detail_;
  };

  /// Template instantiation.
  extern template class VoxelLayoutT<VoxelLayoutDetail>;
  /// Template instantiation.
  extern template class VoxelLayoutT<const VoxelLayoutDetail>;

  /// A writable voxel layout for use in @p MapLayout.
  class VoxelLayout : public VoxelLayoutT<VoxelLayoutDetail>
  {
  public:
    /// Constructor: invalid object.
    VoxelLayout();

    /// Constructor wrapping @p detail.
    /// @param detail Underlying voxel data structure.
    VoxelLayout(VoxelLayoutDetail *detail);

    /// Copy constructor (shallow).
    /// @param other Object to shallow copy.
    VoxelLayout(const VoxelLayout &other);

    /// Add a member to the structure.
    ///
    /// The name is added at a byte offset of @c voxelByteSize(), with some data alignment padding.
    /// Padding is added to ensure a particular alignment depending on the data size of @p type as show below.
    ///
    /// Type Size (bytes) | Supported Minimum Alignment (bytes)
    /// ----------------- | -----------------------------------
    /// 1                 | 1
    /// 2                 | 2
    /// 3                 | 4
    /// 4                 | 4
    /// 5+                | 8
    ///
    /// That is, padding is added to ensure the @c memberOffset() of the added member matches that indicated above.
    ///
    /// @param name The data member. Should be unique, but not checked.
    /// @param type The member data type.
    /// @param clear_value The value used with memset to initialise the data member.
    void addMember(const char *name, DataType::Type type, uint64_t clear_value);

    /// Assignment operator.
    /// @param other Object to shallow copy.
    inline VoxelLayout &operator=(const VoxelLayout &other) { detail_ = other.detail(); return *this; }
  };

  /// A readonly voxel layout for use in @p MapLayout.
  class VoxelLayoutConst : public VoxelLayoutT<const VoxelLayoutDetail>
  {
  public:
    /// Constructor: invalid object.
    VoxelLayoutConst();

    /// Constructor wrapping @p detail.
    /// @param detail Underlying voxel data structure.
    VoxelLayoutConst(const VoxelLayoutDetail *detail);

    /// Copy constructor (shallow).
    /// @param other Object to shallow copy.
    VoxelLayoutConst(const VoxelLayoutConst &other);

    /// Copy constructor from writable object (shallow).
    /// @param other Object to shallow copy.
    VoxelLayoutConst(const VoxelLayout &other);

    /// Assignment operator.
    /// @param other Object to shallow copy.
    inline VoxelLayoutConst &operator=(const VoxelLayoutConst &other) { detail_ = other.detail(); return *this; }

    /// Assignment operator from writable object.
    /// @param other Object to shallow copy.
    inline VoxelLayoutConst &operator=(const VoxelLayout &other) { detail_ = other.detail(); return *this; }
  };


  template <typename T>
  VoxelLayoutT<T>::VoxelLayoutT(T *detail)
    : detail_(detail)
  {
  }


  template <typename T>
  VoxelLayoutT<T>::VoxelLayoutT(const VoxelLayoutT &other)
    : detail_(other.detail_)
  {
  }


  template <typename T>
  VoxelLayoutT<T>::VoxelLayoutT(VoxelLayoutT &&other) noexcept
    : detail_(other.detail_)
  {
    other.detail_ = nullptr;
  }
}

#endif // OHMVOXELLAYOUT_H
