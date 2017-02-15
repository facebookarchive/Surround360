#include "BinaryFootageFile.hpp"
#include "Raw12Converter.hpp"
#include "CameraIspPipe.h"
#include <vector>
#include <future>
#include <functional>
#include <unordered_map>
#include <set>
#include <queue>
#include <iomanip>

extern "C" {
#include <sys/stat.h>
#include <sys/types.h>
}
using namespace surround360;
using namespace std;

int main(int argc, const char *argv[]) {
  static unordered_map<uint32_t, string> ispConfigurations;
  vector<BinaryFootageFile> footageFiles;
  const string jsonConfigDir(argv[1]);
  const string destinationDir(argv[2]);

  for (auto k = 3; k < argc; ++k) {
    footageFiles.emplace_back(argv[k]);
  }

  set<uint32_t> serialNumbers;

  for (auto& footageFile : footageFiles) {
    footageFile.open();

    clock_t t0, t1;
    t0 = clock();

    using futureType = std::future<void>;
    vector<futureType> taskHandles;
    vector<uint32_t> cameraIndexToSerial(footageFile.getNumberOfCameras());

    for (auto cameraIndex = 0; cameraIndex < footageFile.getNumberOfCameras(); ++cameraIndex) {
      auto taskHandle = std::async(
        std::launch::async,
        [=, &footageFile, &serialNumbers] {
          string json;
          for (auto frameIndex = 0; frameIndex < footageFile.getNumberOfFrames(); ++frameIndex) {
            auto frame = footageFile.getFrame(frameIndex, cameraIndex);
            const auto serial = reinterpret_cast<const uint32_t*>(frame)[1];

            if (frameIndex == 0) {
              serialNumbers.insert(serial);
              ostringstream dirStream;
              dirStream << destinationDir << "/" << serial;
              mkdir(dirStream.str().c_str(), 0755);
              const string fname(jsonConfigDir + "/" + to_string(serial) + ".json");
              ifstream ifs(fname, std::ios::in);

              json = string(
                (std::istreambuf_iterator<char>(ifs)),
                (std::istreambuf_iterator<char>()));
            }

            const auto width = footageFile.getMetadata().width;
            const auto height = footageFile.getMetadata().height;

            auto upscaled = Raw12Converter::convertFrame(frame, width, height);
            auto coloredImage = make_unique<vector<uint8_t>>(width * height * 3 * sizeof(uint16_t));
            CameraIspPipe isp(json, true, 16);

            isp.loadImage(reinterpret_cast<uint8_t*>(upscaled->data()), width, height);
            isp.getImage(reinterpret_cast<uint8_t*>(coloredImage->data()), true);

            Mat outputImage(height, width, CV_16UC3, coloredImage->data());
            ostringstream filenameStream;
            filenameStream << destinationDir << "/" << serial << "/"
                           << setfill('0') << setw(6) << frameIndex
                           << ".png";

            imwriteExceptionOnFail(filenameStream.str(), outputImage);
          }});
      taskHandles.push_back(move(taskHandle));
    }

    for (auto handleIdx = 0; handleIdx < taskHandles.size(); ++handleIdx) {
      taskHandles[handleIdx].get();
    }
  }

  size_t ordinal = 0;
  for (auto& serial : serialNumbers) {
    ostringstream oldDir, newDir;
    oldDir << destinationDir << "/" << serial;
    newDir << destinationDir << "/cam" << ordinal;

    const string oldDirname(oldDir.str());
    const string newDirname(newDir.str());

    rename(oldDirname.c_str(), newDirname.c_str());
    ++ordinal;
  }
}
