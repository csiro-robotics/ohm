//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OHM_KEYLIST_H
#define OHM_KEYLIST_H

#include "OhmConfig.h"

#include "Key.h"

#include <vector>

namespace ohm
{
/// A container class encapsulating a set of @c Key objects.
class ohm_API KeyList
{
public:
  /// Iterator for an @c KeyList.
  class ohm_API iterator  // NOLINT
  {
  public:
    /// Empty constructor: creates an undefined iterator object.
    inline iterator() = default;
    /// Copy constuctor.
    /// @param other Iterator to copy.
    inline iterator(const iterator &other) = default;
    /// Internal constructor used to iterator the given set of keys.
    /// @param key The first key in the set.
    inline explicit iterator(std::vector<Key>::iterator key)
      : key_(key)
    {}

    /// Dereference the @c iterator into an @c Key.
    /// Iterator must be valid to use or behaviour is undefined.
    /// @return A reference to the current @c Key.
    inline Key &operator*() const { return *key_; }

    /// Dereference the @c iterator into an @c Key.
    /// Iterator must be valid to use or behaviour is undefined.
    /// @return A reference to the current @c Key.
    inline Key *operator->() const { return &*key_; }

    /// Increment to the next key (prefix).
    /// @return A reference to @c this.
    inline iterator &operator++()
    {
      ++key_;
      return *this;
    }
    /// Increment to the next key (postfix).
    /// @return A reference to @c this.
    inline iterator operator++(int)
    {
      iterator i(key_);
      ++key_;
      return i;
    }

    /// Decrement to the previous key (prefix).
    /// @return A reference to @c this.
    inline iterator &operator--()
    {
      --key_;
      return *this;
    }
    /// Decrement to the previous key (postfix).
    /// @return A reference to @c this.
    inline iterator operator--(int)
    {
      iterator i(key_);
      --key_;
      return i;
    }

    /// Internal iterator access.
    /// @return The underlying iterator.
    inline const std::vector<Key>::iterator &internalIterator() const { return key_; }

    /// Compare this iterator to @p other for equality.
    /// @return True when both iterators reference the same key.
    inline bool operator==(const iterator &other) { return key_ == other.key_; }
    /// Compare this iterator to @p other for inequality.
    /// @return True when the iterators do not reference the same key.
    inline bool operator!=(const iterator &other) { return key_ != other.key_; }

  private:
    std::vector<Key>::iterator key_;
  };

  /// Iterator referencing an @c KeyList referencing a @c const @c Key.
  class ohm_API const_iterator  // NOLINT
  {
  public:
    /// Empty constructor: creates an undefined iterator object.
    inline const_iterator() = default;
    /// Copy constuctor.
    /// @param other Iterator to copy.
    inline const_iterator(const const_iterator &other) = default;
    /// Copy constuctor from a non-const @c iterator.
    /// @param other Iterator to copy.
    inline explicit const_iterator(const iterator &other) { key_ = other.internalIterator(); }
    /// Internal constructor used to iterator the given set of keys.
    /// @param key The first key in the set.
    inline explicit const_iterator(std::vector<Key>::const_iterator key)
      : key_(key)
    {}

    /// Dereference the @c const_iterator into a @c const @c Key.
    /// Iterator must be valid to use or behaviour is undefined.
    /// @return A reference to the current @c Key.
    inline const Key &operator*() const { return *key_; }
    /// Dereference the @c const_iterator into a @c const @c Key.
    /// Iterator must be valid to use or behaviour is undefined.
    /// @return A reference to the current @c Key.
    inline const Key *operator->() const { return &*key_; }

    /// Increment to the next key (prefix).
    /// @return A reference to @c this.
    inline const_iterator &operator++()
    {
      ++key_;
      return *this;
    }
    /// Increment to the next key (postfix).
    /// @return A reference to @c this.
    inline const_iterator operator++(int)
    {
      const_iterator i(key_);
      ++key_;
      return i;
    }

    /// Decrement to the previous key (prefix).
    /// @return A reference to @c this.
    inline const_iterator &operator--()
    {
      --key_;
      return *this;
    }
    /// Decrement to the previous key (postfix).
    /// @return A reference to @c this.
    inline const_iterator operator--(int)
    {
      const_iterator i(key_);
      --key_;
      return i;
    }

    /// Compare this iterator to @p other for equality.
    /// @return True when both iterators reference the same key.
    inline bool operator==(const const_iterator &other) { return key_ == other.key_; }
    /// Compare this iterator to @p other for inequality.
    /// @return True when the iterators do not reference the same key.
    inline bool operator!=(const const_iterator &other) { return key_ != other.key_; }

  private:
    std::vector<Key>::const_iterator key_;
  };

  /// Create a key list supporting sized to the given @p initialCount.
  /// @param initial_count The initial size for the key list (see @c resize()).
  explicit KeyList(size_t initial_count = 0);
  /// Destructor.
  ~KeyList();

  /// Create an @c interator to the first element in the key list.
  /// @return An iterator to the first key in the list.
  inline iterator begin() { return iterator(keys_.begin()); }
  /// Create an @c const_interator to the first element in the key list.
  /// @return An iterator to the first key in the list.
  inline const_iterator begin() const { return const_iterator(keys_.begin()); }
  /// Create the end @c iterator, marking the end of iteration. The iterator is not valid for deferencing.
  /// @return An end style iterator.
  inline iterator end() { return iterator(keys_.end()); }
  /// Create the end @c const_iterator, marking the end of iteration. The iterator is not valid for deferencing.
  /// @return An end style iterator.
  inline const_iterator end() const { return const_iterator(keys_.end()); }

  /// Reserve space to contain @p capacity keys in the set.
  /// @param capacity The desired capacity.
  void reserve(size_t capacity);
  /// Resize the key set to contain @p count keys.
  ///
  /// The capacity is increased if required, but will not be decreased.
  ///
  /// @param count The number of keys to hold.
  void resize(size_t count);

  /// Clear the key list to contain zero elements.
  /// This does not free resources.
  inline void clear() { keys_.clear(); }

  /// Check if the key list is empty, containing no elements.
  /// @return True if the key list is empty.
  inline bool empty() const { return keys_.empty(); }
  /// An alias for @c empty().
  /// @return True if the key list is empty.
  inline bool isEmpty() const { return keys_.empty(); }

  /// Query the capacity of the key list.
  /// @return The maximum number of keys the list can currently hold without re-allocation.
  inline size_t capacity() const { return keys_.capacity(); }
  /// Query the number of items currently in the list.
  /// @return The number of keys available in the list.
  inline size_t size() const { return keys_.size(); }
  /// Query the number of items currently in the list.
  /// @return The number of keys available in the list.
  inline size_t count() const { return keys_.size(); }

  /// Direct access to the underlying key list array. Use with care.
  /// @return A pointer to the memory used to store the keys.
  inline Key *data() { return keys_.data(); }
  /// @overload
  inline const Key *data() const { return keys_.data(); }

  /// Request the key at index @p i (unsafe). The request is not bounds checked and the user
  /// is responsible for ensuring <tt>i < count()</tt>.
  /// @param i The index of the requested key. Must be in the range <tt>[0, count())</tt>.
  /// @return The key at index @p i.
  inline Key &at(size_t i) { return keys_[i]; }
  /// @overload
  inline const Key &at(size_t i) const { return keys_[i]; }

  /// Array style access to the key at index @p i. Semantically equivalent to @c at().
  /// @param i The index of the requested key. Must be in the range <tt>[0, count())</tt>.
  /// @return The key at index @p i.
  inline Key &operator[](size_t i) { return keys_[i]; }
  /// @overload
  inline const Key &operator[](size_t i) const { return keys_[i]; }

  /// Add a key to the end of the list.
  /// The list will grow if required, reallocating the underlying memory.
  /// @param key The key to add.
  void emplace_back(const Key &key);  // NOLINT

  /// Add a key to the end of the list and return a reference to the new key.
  /// The list will grow if required, reallocating the underlying memory.
  ///
  /// The returned key reference is only valid so long as the key list memory remains the same.
  ///
  /// @return A reference to the added key.
  Key &add();

  /// Add a key to the end of the list.
  /// The list will grow if required, reallocating the underlying memory.
  /// @param key The key to add.
  inline void add(const Key &key) { return emplace_back(key); }

private:
  std::vector<Key> keys_;
};
}  // namespace ohm

#endif  // OHM_KEYLIST_H
