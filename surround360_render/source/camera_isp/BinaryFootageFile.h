#pragma once

#include <string>

namespace surround360
{
class BinaryFootageFile {
 public:
  struct MetadataHeader {
    uint32_t magic;
    uint32_t timestamp;
    uint32_t fileIndex;
    uint32_t fileCount;
    uint32_t width;
    uint32_t height;
    uint32_t bitsPerPixel;
    uint32_t numberOfCameras;
  } __attribute__((packed));

  BinaryFootageFile() = default;
  BinaryFootageFile(const std::string& filePath);
  ~BinaryFootageFile();

  void open();
  void close();
  const std::string getFilename() const;
  const uint8_t* getFrame(const uint32_t frameNumber, const uint32_t cameraNumber) const;
  const size_t getNumberOfCameras() const;
  const size_t getNumberOfFrames() const;
  const size_t getFrameSize() const;
  const size_t getBitsPerPixel() const;
  const MetadataHeader& getMetadata() const {
    return metadata;
  }

 private:
  void readMetadataHeader();
  int openFile();
  size_t getFileSize() const;
  bool isMetadataValid() const;
  void* mapFile();
  void unmapFile();
  const void* calculateFrameAddress(const uint32_t frameNumber, const uint32_t cameraNumber) const;
  void discardFrameMemory(const uint32_t frameNumber, const uint32_t cameraNumber) const;

 private:
  mutable int fileDescriptor;
  void* baseAddress;
  size_t mappingSize;
  const std::string filename;
  const std::string directory;
  MetadataHeader metadata;
};
}
