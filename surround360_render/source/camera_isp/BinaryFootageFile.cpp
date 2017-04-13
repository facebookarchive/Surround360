#include "BinaryFootageFile.h"

extern "C" {
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <libgen.h>
}

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "CvUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

using namespace surround360;
using namespace std;

BinaryFootageFile::BinaryFootageFile(const string& filePath)
  : fileDescriptor(-1),
    baseAddress(nullptr),
    mappingSize(0),
    filename(basename(const_cast<char*>(filePath.c_str()))),
    directory(dirname(const_cast<char*>(filePath.c_str()))) {
}

int BinaryFootageFile::openFile() {
  ostringstream pathStream;
  pathStream << directory << "/" << filename;
  const string completePath(pathStream.str());

  int fd = ::open(completePath.c_str(), O_RDONLY);
  if (fd == -1) {
    ostringstream errStream;
    errStream << "Error opening file " << directory
              << "/" << filename
              << ": " << strerror(errno);
    throw runtime_error(errStream.str());
  }

  return fd;
}

size_t BinaryFootageFile::getFileSize() const {
  struct stat fileInfo;
  int err = fstat(fileDescriptor, &fileInfo);
  if (err < 0) {
    ::close(fileDescriptor);
    fileDescriptor = -1;

    ostringstream errStream;
    errStream << "Error retrieving stat() information for file "
              << directory << "/" << filename
              << ": " << strerror(errno);
    throw runtime_error(errStream.str());
  }

  return fileInfo.st_size;
}

void* BinaryFootageFile::mapFile() {
  void* addr = mmap(
    nullptr,
    mappingSize,
    PROT_READ,
    MAP_FILE | MAP_PRIVATE,
    fileDescriptor,
    0);
  if (addr == MAP_FAILED) {
    ostringstream errStream;
    errStream << "Error mmap'ing() file "
              << directory << "/" << filename
              << ": " << strerror(errno);
    throw runtime_error(errStream.str());
  }

  return addr;
}

void BinaryFootageFile::open() {
  fileDescriptor = openFile();
  mappingSize = getFileSize();
  baseAddress = mapFile();
  readMetadataHeader();
}

void BinaryFootageFile::unmapFile() {
  if (baseAddress != nullptr) {
    munmap(baseAddress, mappingSize);
    baseAddress = nullptr;
    mappingSize = 0;
  }
}

void BinaryFootageFile::close() {
  unmapFile();
  ::close(fileDescriptor);
  fileDescriptor = -1;
}

BinaryFootageFile::~BinaryFootageFile() {
  close();
}

void BinaryFootageFile::readMetadataHeader() {
  memcpy(&metadata, baseAddress, sizeof(MetadataHeader));
  cout << "Metadata:" << endl
       << "magic = " << hex << metadata.magic << dec << endl
       << "timestamp = " << metadata.timestamp << endl
       << "fileIndex = " << metadata.fileIndex << endl
       << "fileCount = " << metadata.fileCount << endl
       << "width = " << metadata.width << endl
       << "height = " << metadata.height << endl
       << "bpp = " << metadata.bitsPerPixel << endl
       << "numberOfCameras = " << metadata.numberOfCameras << endl;
}

bool BinaryFootageFile::isMetadataValid() const {
  return metadata.magic == 0xfaceb00c;
}

const std::string BinaryFootageFile::getFilename() const {
  return filename;
}

const uint8_t* BinaryFootageFile::getFrame(
  const uint32_t frameNumber, const uint32_t cameraNumber) const {
  // frame access pattern is sequential. when we reach next frame,
  // mark memory range corresponding to the previous frame as not
  // needed
  if (cameraNumber == 0 && frameNumber > 0) {
    discardFrameMemory(frameNumber - 1, cameraNumber);
  }
  const uint8_t* frameAddr =
    reinterpret_cast<const uint8_t*>(calculateFrameAddress(frameNumber, cameraNumber));

  return frameAddr;
}

const size_t BinaryFootageFile::getNumberOfCameras() const {
  return metadata.numberOfCameras;
}

const size_t BinaryFootageFile::getNumberOfFrames() const {
  const size_t kNumCameras = getNumberOfCameras();

  if (kNumCameras == 0) {
    return 0;
  }

  const size_t kSingleFrameSize = getFrameSize();
  const size_t kPageSize = 4096;
  return (mappingSize - kPageSize) / kSingleFrameSize / kNumCameras;
}

const size_t BinaryFootageFile::getFrameSize() const {
  return size_t(metadata.width * metadata.height * metadata.bitsPerPixel / CHAR_BIT);
}

const size_t BinaryFootageFile::getBitsPerPixel() const {
  return metadata.bitsPerPixel;
}

const void* BinaryFootageFile::calculateFrameAddress(const uint32_t frameNumber, const uint32_t cameraNumber) const {
  const size_t kMetadataSize(4096);
  const uint8_t* framesBase(reinterpret_cast<uint8_t*>(baseAddress) + kMetadataSize);
  const size_t kSingleFrameSize = getFrameSize();

  if (cameraNumber >= metadata.numberOfCameras) {
    throw runtime_error("Camera number out of range");
  }

  auto frameAddr = framesBase + (metadata.numberOfCameras * frameNumber + cameraNumber) * kSingleFrameSize;
  if (frameAddr >= (framesBase + mappingSize)) {
    ostringstream err;
    err << "Accessing frame " << frameNumber << " addr: "
        << hex << reinterpret_cast<uint64_t>(frameAddr) << dec
        << " is out of range for this file."
        << " Mapping is (" << hex
        << reinterpret_cast<uint64_t>(framesBase) << ", "
        << reinterpret_cast<uint64_t>(framesBase + mappingSize)
        << ")" << dec << endl;
    throw runtime_error(err.str());
  }

  return frameAddr;
}

void BinaryFootageFile::discardFrameMemory(
  const uint32_t frameNumber,
  const uint32_t cameraNumber) const {
  auto frameAddress = const_cast<void*>(calculateFrameAddress(frameNumber, cameraNumber));

  auto rangeSize = getFrameSize();
  int err = madvise(frameAddress, rangeSize, MADV_DONTNEED);

  if (err == -1) {
    ostringstream errStream;
    errStream << "Error madvising() file " << directory
              << "/" << filename
              << ": " << strerror(errno);
    throw runtime_error(errStream.str());
  }
}
