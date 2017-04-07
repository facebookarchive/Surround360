#include "render/RigDescription.h"
#include "render/ImageWarper.h"
#include "source/scanner_kernels/surround360.pb.h"

#include "scanner/api/kernel.h"
#include "scanner/api/op.h"
#include "scanner/util/memory.h"
#include "scanner/util/opencv.h"

#include <opencv2/video.hpp>

using namespace scanner;

namespace surround360 {

namespace {
// measured in radians from forward
float approximateFov(const Camera& camera, const bool vertical) {
  Camera::Vector2 a = camera.principal;
  Camera::Vector2 b = camera.principal;
  if (vertical) {
    a.y() = 0;
    b.y() = camera.resolution.y();
  } else {
    a.x() = 0;
    b.x() = camera.resolution.x();
  }
  return acos(max(
      camera.rig(a).direction().dot(camera.forward()),
      camera.rig(b).direction().dot(camera.forward())));
}

// measured in radians from forward
float approximateFov(const Camera::Rig& rig, const bool vertical) {
  float result = 0;
  for (const auto& camera : rig) {
    result = std::max(result, approximateFov(camera, vertical));
  }
  return result;
}
}

class ProjectSphericalKernelCPU : public VideoKernel {
 public:
  ProjectSphericalKernelCPU(const Kernel::Config& config)
      : VideoKernel(config),
        device_(config.devices[0]),
        work_item_size_(config.work_item_size) {
    args_.ParseFromArray(config.args.data(), config.args.size());

    // Initialize camera rig
    rig_.reset(new RigDescription(args_.camera_rig_path()));

    hRadians_ = 2 * approximateFov(rig_->rigSideOnly, false);
    vRadians_ = 2 * approximateFov(rig_->rigSideOnly, true);
  }

  void reset() override {
    is_reset_ = true;
  }

  void execute(const BatchedColumns& input_columns,
               BatchedColumns& output_columns) override {
    auto& frame_col = input_columns[0];
    auto& frame_info_col = input_columns[1];
    auto& camera_id_col = input_columns[2];
    check_frame_info(device_, frame_info_col);

    if (is_reset_) {
      // Use the new camera id to update the spherical projection parameters
      is_reset_ = false;

      camIdx_ = *((int*)camera_id_col.rows[0].buffer);
      const Camera& camera = rig_->rigSideOnly[camIdx_];

      // the negative sign here is so the camera array goes clockwise
      const int numCameras = 14;
      float direction = -float(camIdx_) / float(numCameras) * 2.0f * M_PI;
      leftAngle_ = direction + hRadians_ / 2;
      rightAngle_ = direction - hRadians_ / 2;
      topAngle_ = vRadians_ / 2;
      bottomAngle_ = -vRadians_ / 2;
    }

    i32 input_count = (i32)frame_col.rows.size();
    size_t output_image_width = args_.eqr_width() * hRadians_ / (2 * M_PI);
    size_t output_image_height = args_.eqr_height() * vRadians_ / M_PI;
    size_t output_image_size = output_image_width * output_image_height * 4;
    u8 *output_block =
        new_block_buffer(device_, output_image_size * input_count, input_count);

    // Output frame info
    scanner::proto::FrameInfo output_frame_info;
    output_frame_info.set_width(output_image_width);
    output_frame_info.set_height(output_image_height);
    output_frame_info.set_channels(4);
    u8 *output_frame_info_buffer =
        new_block_buffer(device_, output_frame_info.ByteSize(), input_count);
    output_frame_info.SerializeToArray(output_frame_info_buffer,
                                       output_frame_info.ByteSize());

    for (i32 i = 0; i < input_count; ++i) {
      cv::Mat input(frame_info_.height(), frame_info_.width(), CV_8UC3,
                    input_columns[0].rows[i].buffer);
      cv::Mat tmp;
      cv::cvtColor(input, tmp, CV_BGR2BGRA);

      cv::Mat projection_image(output_image_height, output_image_width, CV_8UC4,
                               output_block + i * output_image_size);

      surround360::warper::bicubicRemapToSpherical(
          projection_image, tmp, rig_->rigSideOnly[camIdx_], leftAngle_,
          rightAngle_, topAngle_, bottomAngle_);

      output_columns[0].rows.push_back(
          Row{projection_image.data, output_image_size});
      output_columns[1].rows.push_back(
          Row{output_frame_info_buffer, output_frame_info.ByteSize()});
    }
  }

 private:
  surround360::proto::ProjectSphericalArgs args_;
  std::unique_ptr<RigDescription> rig_;
  DeviceHandle device_;
  i32 work_item_size_;
  bool is_reset_ = true;

  float hRadians_;
  float vRadians_;

  int camIdx_;
  float leftAngle_;
  float rightAngle_;
  float topAngle_;
  float bottomAngle_;
};

REGISTER_OP(ProjectSpherical)
    .inputs({"frame", "frame_info", "camera_id"})
    .outputs({"projected_frame", "frame_info"});

REGISTER_KERNEL(ProjectSpherical, ProjectSphericalKernelCPU)
    .device(DeviceType::CPU)
    .num_devices(1);
}
