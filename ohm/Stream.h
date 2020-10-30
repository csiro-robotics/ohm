// Copyright (c) 2015
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMSTREAM_H
#define OHMSTREAM_H

#include "OhmConfig.h"

#include <cstddef>

#define OHM_ZIP 1

namespace ohm
{
struct InputStreamPrivate;
struct OutputStreamPrivate;
struct StreamPrivate;


/// Flags for use with @c InputStream and @c OutputStream.
enum StreamFlag
{
  /// Compression is enabled.
  kSfCompress = (1 << 0),
};


/// Base class for @c InputStream and @c OutputStream used to encapsulate file access and compression.
/// The stream is tailored towards file access, not a generalised stream.
class Stream
{
public:
  /// Returns true if the stream (file) is open.
  /// @return True when open.
  virtual bool isOpen() const = 0;

  /// Returns the file path given when the stream was opened.
  /// @return The file path, or an empty string when not open.
  const char *filePath() const;

  /// Returns the current @c StreamFlag settings.
  /// @return Current flags.
  unsigned flags() const;

  /// Opens the stream to @p filePath.
  /// @param file_path The relative or absolute file path.
  /// @param flags @c StreamFlag values to open with.
  /// @return True on success.
  bool open(const char *file_path, unsigned flags = 0u);

  /// Closes the file if open. This forces a @c flush().
  void close();

  /// Flushes the file. This forces completion of compression and writes out for output streams.
  /// Input streams generally do nothing on a flush.
  ///
  /// Writing with compression enabled after a flush may cause undefined behaviour.
  virtual void flush() = 0;

  /// Seeks to the given stream position. Use with care, as it can invalidate the compression/decompression
  /// state.
  ///
  /// It is generally best to only seek when not using compression, either on the stream or when using
  /// raw reading and writing (uncompressed). A @c flush() is also best performed before seeking.
  /// @param pos Absolute position to seek to.
  void seek(size_t pos);

  /// Tells the current stream position. This may be undefined when using compression.
  ///
  /// The position may be invalid when using compression as there may be data pending
  /// in the compression buffer. A @c flush() may force an accurate position, but can
  /// invalidate further compression.
  /// @return The current absolute stream position.
  size_t tell();

protected:
  /// Called from @p open() to perform the open operation.
  /// @param file_path The relative or absolute file path.
  /// @param flags @c StreamFlag values to open with.
  /// @return True on success.
  virtual bool doOpen(const char *file_path, unsigned flags) = 0;

  /// Called to close the file (after a @c flush()).
  virtual void doClose() = 0;

  /// Called to perform the @c seek() operation.
  /// @param pos Absolute position to seek to.
  virtual void doSeek(size_t pos) = 0;

  /// Called to perform the @c tell() operation.
  /// @return The current absolute stream position.
  virtual size_t doTell() = 0;

  /// Constructor.
  /// @param imp The implementation pointer, set to @c imp_. Derivations must clean it up.
  Stream(StreamPrivate *imp);
  /// Empty virtual destructor.
  virtual ~Stream();

  /// The stream implementation pointer. Derivations must clean this up.
  StreamPrivate *imp_;
};


/// An input file stream supporting compression.
///
/// Compression may be enabled on @c open() using @c SF_Compress, or later by calling
/// @c setCompressedFlag().
///
/// Reading compressed and uncompressed data is supported, with judicious use. Uncompressed data
/// can be read only if there are no compressed data pending in the compression buffer.
/// Essentially, uncompressed data can be read:
/// - Before any compressed data are read.
/// - After finishing reading all compressed data.
//.
/// It is not advised to read more than one compressed data section.
class InputStream : public Stream
{
public:
  /// Constructor, optionally opening a file.
  ///
  /// @c open() is called only if @p filePath is specified. Success is determined by
  /// checking @c isOpen().
  ///
  /// @param file_path Optionally specifies the file to open.
  /// @param flags The @c StreamFlag set to open with.
  InputStream(const char *file_path = nullptr, unsigned flags = 0u);

  /// Destructor. Flushes and closes the file.
  ~InputStream() override;

  /// Enables reading compressed data after opening.
  ///
  /// Intended for use when compression state is determined by data in a file header.
  /// In this case, the header is read uncompressed, then the stream is switched to
  /// reading compressed data.
  void setCompressedFlag();

  /// Read from the file. Decompression enabled if supported.
  /// @param buffer Buffer to read into.
  /// @param max_bytes Maximum number of bytes to read. @p buffer must be large enough for this content.
  /// @return The number of bytes read. Zero indicates no bytes available, or an decompression error.
  unsigned read(void *buffer, unsigned max_bytes);

  /// Read raw bytes from the file, no decompression.
  ///
  /// This may only be used before calls to @c read(), or after a call to
  /// @c flush().
  ///
  /// @param buffer Buffer to read into.
  /// @param max_bytes Maximum number of bytes to read. @p buffer must be large enough for this content.
  /// @return The number of bytes read. Zero indicates no bytes available, or an decompression error.
  unsigned readRaw(void *buffer, unsigned max_bytes);

  /// Returns true if the stream (file) is open.
  /// @return True when open.
  bool isOpen() const override;

  /// No operation required.
  void flush() override;

protected:
  /// Called from @p open() to perform the open operation.
  /// @param file_path The relative or absolute file path.
  /// @param flags @c StreamFlag values to open with.
  /// @return True on success.
  bool doOpen(const char *file_path, unsigned flags) override;

  /// Called to close the file (after a @c flush()).
  void doClose() override;

  /// Called to perform the @c seek() operation.
  /// @param pos Absolute position to seek to.
  void doSeek(size_t pos) override;

  /// Called to perform the @c tell() operation.
  /// @return The current absolute stream position.
  size_t doTell() override;

  /// Conversion to the underlying implementation.
  /// @return The underlying implementation.
  InputStreamPrivate *imp();

  /// Conversion to the underlying implementation.
  /// @return The underlying implementation.
  const InputStreamPrivate *imp() const;
};

/// An output file stream supporting compression.
///
/// Compression may be enabled on @c open() using @c SF_Compress, or later by calling
/// @c setCompressedFlag().
///
/// Writing compressed and uncompressed data is supported, with judicious use. Uncompressed data
/// can be written only if there are no compressed data pending in the compression buffer.
/// Essentially, uncompressed data can be written:
/// - Before any compressed data are written.
/// - After finishing reading all compressed data.
//.
/// It is not advised to write more than one compressed data section.
class OutputStream : public Stream
{
public:
  /// Constructor, optionally opening a file.
  ///
  /// @c open() is called only if @p filePath is specified. Success is determined by
  /// checking @c isOpen().
  ///
  /// @param file_path Optionally specifies the file to open.
  /// @param flags The @c StreamFlag set to open with.
  OutputStream(const char *file_path = nullptr, unsigned flags = 0u);

  /// Destructor. Flushes and closes the file.
  ~OutputStream() override;

  /// Write bytes to the file, compression enabled.
  unsigned write(const void *buffer, unsigned max_bytes);

  /// Write bytes to the file, with no compression.
  ///
  /// This may only be used before calls to @c write(), or after a call to
  /// @c flush().
  unsigned writeUncompressed(const void *buffer, unsigned max_bytes);

  /// Returns true if the stream (file) is open.
  /// @return True when open.
  bool isOpen() const override;

  /// Flushes the file. This forces completion of compression and flushes the file.
  ///
  /// Writing with compression enabled after a flush may cause undefined behaviour.
  void flush() override;

private:
  /// Called from @p open() to perform the open operation.
  /// @param file_path The relative or absolute file path.
  /// @param flags @c StreamFlag values to open with.
  /// @return True on success.
  bool doOpen(const char *file_path, unsigned flags) override;

  /// Called to close the file (after a @c flush()).
  void doClose() override;

  /// Called to perform the @c seek() operation.
  /// @param pos Absolute position to seek to.
  void doSeek(size_t pos) override;

  /// Called to perform the @c tell() operation.
  /// @return The current absolute stream position.
  size_t doTell() override;

  /// Conversion to the underlying implementation.
  /// @return The underlying implementation.
  OutputStreamPrivate *imp();

  /// Conversion to the underlying implementation.
  /// @return The underlying implementation.
  const OutputStreamPrivate *imp() const;
};
}  // namespace ohm

#endif  // OHMSTREAM_H
