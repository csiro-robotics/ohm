//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OCCUPANCYKEYLIST_H_
#define OCCUPANCYKEYLIST_H_

#include "ohmconfig.h"

#include "occupancykey.h"

namespace ohm
{
  /// A container class encapsulating a set of @c OccupancyKey objects.
  class ohm_API OccupancyKeyList
  {
  public:
    /// Iterator for an @c OccupancyKeyList.
    class ohm_API iterator
    {
    public:
      /// Empty constructor: creates an undefined iterator object.
      inline iterator() {}
      /// Copy constuctor.
      /// @parma other Iterator to copy.
      inline iterator(const iterator &other)
        : _key(other._key)
      {}
      /// Internal constructor used to iterator the given set of keys.
      /// @param key The first key in the set.
      inline iterator(OccupancyKey *key)
        : _key(key)
      {}

      /// Dereference the @c iterator into an @c OccupancyKey.
      /// Iterator must be valid to use or behaviour is undefined.
      /// @return A reference to the current @c OccupancyKey.
      inline OccupancyKey &operator*() const { return *_key; }

      /// Dereference the @c iterator into an @c OccupancyKey.
      /// Iterator must be valid to use or behaviour is undefined.
      /// @return A reference to the current @c OccupancyKey.
      inline OccupancyKey *operator->() const { return _key; }

      /// Increment to the next key (prefix).
      /// @return A reference to @c this.
      inline iterator &operator++()
      {
        ++_key;
        return *this;
      }
      /// Increment to the next key (postfix).
      /// @return A reference to @c this.
      inline iterator operator++(int)
      {
        iterator i(_key);
        ++_key;
        return i;
      }

      /// Decrement to the previous key (prefix).
      /// @return A reference to @c this.
      inline iterator &operator--()
      {
        --_key;
        return *this;
      }
      /// Decrement to the previous key (postfix).
      /// @return A reference to @c this.
      inline iterator operator--(int)
      {
        iterator i(_key);
        --_key;
        return i;
      }

      /// Compare this iterator to @p other for equality.
      /// @return True when both iterators reference the same key.
      inline bool operator==(const iterator &other) { return _key == other._key; }
      /// Compare this iterator to @p other for inequality.
      /// @return True when the iterators do not reference the same key.
      inline bool operator!=(const iterator &other) { return _key != other._key; }

    private:
      OccupancyKey *_key;
    };

    /// Iterator referencing an @c OccupancyKeyList referencing a @c const @c OccupancyKey.
    class ohm_API const_iterator
    {
    public:
      /// Empty constructor: creates an undefined iterator object.
      inline const_iterator() {}
      /// Copy constuctor.
      /// @parma other Iterator to copy.
      inline const_iterator(const const_iterator &other)
        : _key(other._key)
      {}
      /// Copy constuctor from a non-const @c iterator.
      /// @parma other Iterator to copy.
      inline const_iterator(const iterator &other)
        : _key(&*other)
      {}
      /// Internal constructor used to iterator the given set of keys.
      /// @param key The first key in the set.
      inline const_iterator(const OccupancyKey *key)
        : _key(key)
      {}

      /// Dereference the @c const_iterator into a @c const @c OccupancyKey.
      /// Iterator must be valid to use or behaviour is undefined.
      /// @return A reference to the current @c OccupancyKey.
      inline const OccupancyKey &operator*() const { return *_key; }
      /// Dereference the @c const_iterator into a @c const @c OccupancyKey.
      /// Iterator must be valid to use or behaviour is undefined.
      /// @return A reference to the current @c OccupancyKey.
      inline const OccupancyKey *operator->() const { return _key; }

      /// Increment to the next key (prefix).
      /// @return A reference to @c this.
      inline const_iterator &operator++()
      {
        ++_key;
        return *this;
      }
      /// Increment to the next key (postfix).
      /// @return A reference to @c this.
      inline const_iterator operator++(int)
      {
        const_iterator i(_key);
        ++_key;
        return i;
      }

      /// Decrement to the previous key (prefix).
      /// @return A reference to @c this.
      inline const_iterator &operator--()
      {
        --_key;
        return *this;
      }
      /// Decrement to the previous key (postfix).
      /// @return A reference to @c this.
      inline const_iterator operator--(int)
      {
        const_iterator i(_key);
        --_key;
        return i;
      }

      /// Compare this iterator to @p other for equality.
      /// @return True when both iterators reference the same key.
      inline bool operator==(const const_iterator &other) { return _key == other._key; }
      /// Compare this iterator to @p other for inequality.
      /// @return True when the iterators do not reference the same key.
      inline bool operator!=(const const_iterator &other) { return _key != other._key; }

    private:
      const OccupancyKey *_key;
    };

    /// Create a key list supporting sized to the given @p initialCount.
    /// @param initialCount The initial size for the key list (see @c resize()).
    OccupancyKeyList(size_t initialCount = 0);
    /// Destructor.
    ~OccupancyKeyList();

    /// Create an @c interator to the first element in the key list.
    /// @return An iterator to the first key in the list.
    inline iterator begin() { return iterator(_keys); }
    /// Create an @c const_interator to the first element in the key list.
    /// @return An iterator to the first key in the list.
    inline const_iterator begin() const { return const_iterator(_keys); }
    /// Create the end @c iterator, marking the end of iteration. The iterator is not valid for deferencing.
    /// @return An end style iterator.
    inline iterator end() { return iterator(_keys + _count); }
    /// Create the end @c const_iterator, marking the end of iteration. The iterator is not valid for deferencing.
    /// @return An end style iterator.
    inline const_iterator end() const { return const_iterator(_keys + _count); }

    /// Reserve space to contain @p capacity keys in the set.
    /// @param capacity The desired capcacity.
    void reserve(size_t capacity);
    /// Resize the key set to contain @p count keys.
    ///
    /// The capacity is increased if required, but will not be decreased.
    ///
    /// @param count The number of keys to hold.
    void resize(size_t count);

    /// Clear the key list to contain zero elements.
    /// This does not free resources.
    inline void clear() { _count = 0; }

    /// Check if the key list is empty, containing no elements.
    /// @return True if the key list is empty.
    inline bool empty() const { return _count == 0; }
    /// An alias for @c emtpy().
    /// @return True if the key list is empty.
    inline bool isEmpty() const { return _count == 0; }

    /// Query the capacity of the key list.
    /// @return The maximum number of keys the list can currently hold without re-allocation.
    inline size_t capacity() const { return _capacity; }
    /// Query the number of items currently in the list.
    /// @return The number of keys available in the list.
    inline size_t size() const { return _count; }
    /// Query the number of items currently in the list.
    /// @return The number of keys available in the list.
    inline size_t count() const { return _count; }

    /// Direct access to the underlying key list array. Use with care.
    /// @return A pointer to the memory used to store the keys.
    inline OccupancyKey *data() { return _keys; }
    /// @overload
    inline const OccupancyKey *data() const { return _keys; }

    /// Request the key at index @p i (unsafe). The request is not bounds checked and the user
    /// is responsible for ensuring <tt>i < count()</tt>.
    /// @param i The index of the requested key. Must be in the range <tt>[0, count())</tt>.
    /// @return The key at index @p i.
    inline OccupancyKey &at(size_t i) { return _keys[i]; }
    /// @overload
    inline const OccupancyKey &at(size_t i) const { return _keys[i]; }

    /// Array style access to the key at index @p i. Semantically equivalent to @c at().
    /// @param i The index of the requested key. Must be in the range <tt>[0, count())</tt>.
    /// @return The key at index @p i.
    inline OccupancyKey &operator[](size_t i) { return _keys[i]; }
    /// @overload
    inline const OccupancyKey &operator[](size_t i) const { return _keys[i]; }

    /// Add a key to the end of the list.
    /// The list will grow if required, reallocating the underlying memory.
    /// @param key The key to add.
    void push_back(const OccupancyKey &key);

    /// Add a key to the end of the list and return a reference to the new key.
    /// The list will grow if required, reallocating the underlying memory.
    ///
    /// The returned key reference is only valid so long as the key list memory remains the same.
    ///
    /// @return A reference to the added key.
    OccupancyKey &add();

    /// Add a key to the end of the list.
    /// The list will grow if required, reallocating the underlying memory.
    /// @param key The key to add.
    inline void add(const OccupancyKey &key) { return push_back(key); }

  private:
    OccupancyKey *_keys;
    size_t _capacity;
    size_t _count;
  };
}

#endif  // OCCUPANCYKEYLIST_H_
