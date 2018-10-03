// Copyright (c) 2015
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Stream.h"

#ifdef OHM_ZIP
#include <zlib.h>
#endif // OHM_ZIP

#include <cstring>
#include <fstream>
#include <string>

namespace ohm
{
#ifdef OHM_ZIP
  struct Compression
  {
    z_stream stream;
    Byte *buffer;
    unsigned buffer_size;
    bool initialised;

    Compression(unsigned buffer_size = 16 * 1024u);
    ~Compression();

    int initDeflate();
    int doneDeflate();

    int initInflate();
    int doneInflate();
  };


  Compression::Compression(unsigned buffer_size)
    : buffer(nullptr)
    , buffer_size(buffer_size)
    , initialised(false)
  {
    memset(&stream, 0u, sizeof(stream));
    buffer = new Byte[buffer_size];
  }


  Compression::~Compression()
  {
    delete[] buffer;
  }


  int Compression::initDeflate()
  {
    if (!initialised)
    {
      memset(&stream, 0u, sizeof(stream));
      return deflateInit(&stream, Z_DEFAULT_COMPRESSION);
    }
    return Z_OK;
  }


  int Compression::doneDeflate()
  {
    if (initialised)
    {
      initialised = false;
      return deflateEnd(&stream);
    }
    return Z_OK;
  }


  int Compression::initInflate()
  {
    if (!initialised)
    {
      memset(&stream, 0u, sizeof(stream));
      return inflateInit(&stream);
    }
    return Z_OK;
  }


  int Compression::doneInflate()
  {
    if (initialised)
    {
      initialised = false;
      return inflateEnd(&stream);
    }
    return Z_OK;
  }

#endif // OHM_ZIP

  struct StreamPrivate
  {
    std::string file_path;
    unsigned flags;
  };

  struct InputStreamPrivate : StreamPrivate
  {
    std::ifstream in;
#ifdef OHM_ZIP
    Compression compress;
#endif // OHM_ZIP
  };

  struct OutputStreamPrivate : StreamPrivate
  {
    std::ofstream out;
#ifdef OHM_ZIP
    Compression compress;
    bool needs_flush;
#endif // OHM_ZIP
  };
}

using namespace ohm;


const char *Stream::filePath() const
{
  return imp_->file_path.c_str();
}


unsigned Stream::flags() const
{
  return imp_->flags;
}


bool Stream::open(const char *file_path, unsigned flags)
{
  close();
  return doOpen(file_path, flags);
}


void Stream::close()
{
  if (isOpen())
  {
    flush();
    doClose();
  }
}


void Stream::seek(size_t pos)
{
  flush();
  doSeek(pos);
}


size_t Stream::tell()
{
  return doTell();
}


Stream::Stream(StreamPrivate *imp)
  : imp_(imp)
{
  imp_->flags = 0u;
}


Stream::~Stream()
{
}


InputStream::InputStream(const char *file_path, unsigned flags)
  : Stream(new InputStreamPrivate)
{
  if (file_path && file_path[0])
  {
    doOpen(file_path, flags);
  }
}


InputStream::~InputStream()
{
  delete imp();
}


void InputStream::setCompressedFlag()
{
#ifdef OHM_ZIP
  if (!isOpen())
  {
    return;
  }

  unsigned &flags = imp()->flags;
  bool changed = !(flags & SfCompress);
  flags |= SfCompress;
  if (changed)
  {
    imp()->compress.initInflate();
  }
#endif // OHM_ZIP
}


unsigned InputStream::read(void *buffer, unsigned max_bytes)
{
#ifdef OHM_ZIP
  if (imp()->flags & SfCompress)
  {
    InputStreamPrivate &imp = *this->imp();
    int ret;
    unsigned have;

    imp.compress.stream.avail_out = max_bytes;
    imp.compress.stream.next_out = static_cast<unsigned char *>(buffer);

    do
    {
      if (!imp.compress.stream.avail_in)
      {
        // No data currently available. Prime the buffer.
        const unsigned read_bytes = readRaw(imp.compress.buffer, imp.compress.buffer_size);
        imp.compress.stream.avail_in = read_bytes;
        imp.compress.stream.next_in = imp.compress.buffer;

        if (!imp.compress.stream.avail_in)
        {
          // Nothing available.
          break;
        }
      }

      ret = inflate(&imp.compress.stream, Z_NO_FLUSH);

      if (ret != Z_OK && ret != Z_STREAM_END)
      {
        return 0;
      }
    }
    while (imp.compress.stream.avail_out);

    have = max_bytes - imp.compress.stream.avail_out;
    return have;
  }
#endif // OHM_ZIP
  return readRaw(buffer, max_bytes);
}


unsigned InputStream::readRaw(void *buffer, unsigned max_bytes)
{
  std::istream &in = imp()->in;
  const std::ifstream::pos_type initial_pos = in.tellg();
  in.read(static_cast<char *>(buffer), max_bytes);
  const std::ifstream::pos_type end_pos = in.tellg();
  unsigned read = unsigned(end_pos - initial_pos);
  return read;
}


bool InputStream::isOpen() const
{
  return imp()->in.is_open();
}


void InputStream::flush()
{
}


bool InputStream::doOpen(const char *file_path, unsigned flags)
{
  imp()->in.open(file_path, std::ios_base::binary);
  imp()->file_path = file_path;
#ifndef OHM_ZIP
  flags &= ~SF_Compress;
#endif // OHM_ZIP
  imp()->flags = flags;
#ifdef OHM_ZIP
  if (flags & SfCompress)
  {
    imp()->compress.initInflate();
  }
#endif // OHM_ZIP
  return imp()->in.is_open();
}


void InputStream::doClose()
{
#ifdef OHM_ZIP
  if (imp_->flags & SfCompress)
  {
    imp()->compress.doneInflate();
  }
#endif // OHM_ZIP
  imp()->in.close();
  imp_->flags = 0;
  imp_->file_path = std::string();
}


void InputStream::doSeek(size_t pos)
{
  imp()->in.seekg(pos, std::ios_base::beg);
}


size_t InputStream::doTell()
{
  return imp()->in.tellg();
}


InputStreamPrivate *InputStream::imp()
{
  return static_cast<InputStreamPrivate *>(imp_);
}


const InputStreamPrivate *InputStream::imp() const
{
  return static_cast<const InputStreamPrivate *>(imp_);
}


OutputStream::OutputStream(const char *file_path, unsigned flags)
  : Stream(new OutputStreamPrivate)
{
  if (file_path && file_path[0])
  {
    doOpen(file_path, flags);
  }

#ifdef OHM_ZIP
  imp()->needs_flush = false;
#endif // OHM_ZIP
}


OutputStream::~OutputStream()
{
  close();
  delete imp();
}



unsigned OutputStream::write(const void *buffer, unsigned max_bytes)
{
#ifdef OHM_ZIP
  if (imp()->flags & SfCompress)
  {
    OutputStreamPrivate &imp = *this->imp();
    int ret;

    imp.compress.stream.next_in = (Bytef *)buffer;
    imp.compress.stream.avail_in = max_bytes;

    if (imp.compress.stream.avail_out == 0)
    {
      imp.compress.stream.avail_out = imp.compress.buffer_size;
      imp.compress.stream.next_out = imp.compress.buffer;
    }

    do
    {
      ret = deflate(&imp.compress.stream, Z_NO_FLUSH);
      if (ret != Z_OK)
      {
        return ~unsigned(0u);
      }
      imp.needs_flush = true;

      if (imp.compress.stream.avail_out == 0)
      {
        imp.out.write(reinterpret_cast<char *>(imp.compress.buffer), imp.compress.buffer_size);
        imp.compress.stream.avail_out = imp.compress.buffer_size;
        imp.compress.stream.next_out = imp.compress.buffer;
      }
    }
    while (imp.compress.stream.avail_in);

    return max_bytes;
  }
#endif // OHM_ZIP
  return writeUncompressed(buffer, max_bytes);
}


unsigned OutputStream::writeUncompressed(const void *buffer, unsigned max_bytes)
{
  imp()->out.write(static_cast<const char *>(buffer), max_bytes);
  if (imp()->out.good())
  {
    return max_bytes;
  }

  return 0;
}


bool OutputStream::isOpen() const
{
  return imp()->out.is_open();
}


void OutputStream::flush()
{
  OutputStreamPrivate &imp = *this->imp();
#ifdef OHM_ZIP
  // Finish deflating.
  if (imp.needs_flush && imp.flags & SfCompress)
  {
    int ret;
    imp.compress.stream.next_in = nullptr;
    imp.compress.stream.avail_in = 0;
    unsigned have = imp.compress.buffer_size - imp.compress.stream.avail_out;
    do
    {
      ret = deflate(&imp.compress.stream, Z_FINISH);
      have = imp.compress.buffer_size - imp.compress.stream.avail_out;
      if (have)
      {
        imp.out.write(reinterpret_cast<char *>(imp.compress.buffer), have);
      }

      imp.compress.stream.avail_out = imp.compress.buffer_size;
      imp.compress.stream.next_out = imp.compress.buffer;
    }
    while (ret == Z_OK);

    imp.needs_flush = false;
  }
#endif // OHM_ZIP
  imp.out.flush();
}


bool OutputStream::doOpen(const char *file_path, unsigned flags /*= 0u*/)
{
  imp()->out.open(file_path, std::ios_base::binary);
  imp()->file_path = file_path;
#ifndef OHM_ZIP
  flags &= ~SF_Compress;
#endif // OHM_ZIP
  imp()->flags = flags;
#ifdef OHM_ZIP
  if (flags & SfCompress)
  {
    imp()->compress.initDeflate();
  }
#endif // OHM_ZIP
  return imp()->out.is_open();
}


void OutputStream::doClose()
{
#ifdef OHM_ZIP
  if (imp_->flags & SfCompress)
  {
    imp()->compress.doneDeflate();
  }
#endif // OHM_ZIP
  imp()->out.close();
  imp_->flags = 0;
  imp_->file_path = std::string();
}


void OutputStream::doSeek(size_t pos)
{
  imp()->out.seekp(pos, std::ios_base::beg);
}


size_t OutputStream::doTell()
{
  return imp()->out.tellp();
}


OutputStreamPrivate   *OutputStream::imp()
{
  return static_cast<OutputStreamPrivate *>(imp_);
}


const OutputStreamPrivate   *OutputStream::imp() const
{
  return static_cast<const OutputStreamPrivate *>(imp_);
}
