/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

#include <memory>
#include <vector>
#include <cstddef>
#include <mutex>
#include <cstdlib>
#include <malloc.h>
#include <iostream>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/freeglut_std.h>

#include <gtkmm.h>
#include <gdkmm.h>
#include <gtkmm/glarea.h>
#include <gdkmm/pixbuf.h>

#include "CameraIspPipe.h"
#include "AlignedAllocator.hpp"

namespace surround360 {
  class CameraView {
  private:
    Gtk::GLArea& m_glAreaRef;
    std::size_t m_width;
    std::size_t m_height;

    GLuint m_vao;
    GLuint m_verts;
    GLuint m_program[3];
    GLuint m_texture[2];
    GLuint m_fboID;

    using aligned_alloc = aligned_allocator<uint8_t, 64>;
    using aligned_vec_t = std::vector<uint8_t, aligned_alloc>;

    std::shared_ptr<aligned_vec_t> m_textureBuf;
    std::shared_ptr<aligned_vec_t> m_rawBuf;

    int currentlyShowing { -1 };

    GLuint m_uv;
    GLuint m_pbo[2];
    int m_pboIdx;
    GLuint m_histogramGeometry;
    std::vector<std::uint8_t> rawBytes;
    CameraIspPipe m_isp;
    std::unique_ptr<std::vector<float>> cameraRotations;
    std::vector<std::uint32_t> m_histogram;
    std::vector<float>         m_normalized;
  private:
    GLuint loadShaders(
      const std::string& vertexshader,
      const std::string& pixelshader);

    void reshape(GLuint progid);
    void initTextures();
    void init();
    void convertPreviewFrame();
    uint32_t bitsPerPixelForPreview;

  public:
    CameraView(Gtk::GLArea& glarea);
    ~CameraView();

    bool onRender(const Glib::RefPtr<Gdk::GLContext>& context);
    void onRealize();
    void onUnrealize();
    auto getRawBuffer() noexcept -> decltype(m_rawBuf){
      return m_rawBuf;
    }
    void update();
    inline void setRotations(std::unique_ptr<std::vector<float>>& rotations) {
      cameraRotations = std::move(rotations);
    }

    inline void setCurrentlyShowing(int idx) {
      currentlyShowing = idx;
    }

    void updatePreviewFrame(const void *bytes, const size_t size, const uint32_t bitsPerPixel);
  };
}
