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

namespace surround360 {
  template <typename T, int ALIGNMENT = 64>
  class aligned_allocator {
  public:
    using value_type = T;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using reference = value_type&;
    using const_reference = const value_type&;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    template<typename U>
    struct rebind {
      using other = aligned_allocator<U>;
    };

    inline explicit aligned_allocator() = default;
    inline ~aligned_allocator() = default;
    inline explicit aligned_allocator(const aligned_allocator& a) = default;

    inline pointer address(reference r) { return &r; }
    inline const_pointer address(const_reference r) { return &r; }

    inline pointer allocate(size_type sz, typename std::allocator<void>::const_pointer = 0) {
      auto x = memalign(ALIGNMENT, sizeof(T) * sz);
      std::cout << "memalign of size " << sz << " at " << std::hex << x << std::dec << std::endl;
      return reinterpret_cast<pointer>(x);
    }

    void deallocate(pointer p, size_type sz) {
      free(p);
    }
  };

  class CameraView {
  private:
    Gtk::GLArea& m_glAreaRef;
    std::size_t m_width;
    std::size_t m_height;

    GLuint m_vao;
    GLuint m_verts;
    GLuint m_program[2];
    GLuint m_texture[2];

    using aligned_alloc = aligned_allocator<uint8_t>;
    using aligned_vec_t = std::vector<uint8_t, aligned_alloc>;

    std::shared_ptr<aligned_vec_t> m_textureBuf;
    std::shared_ptr<aligned_vec_t> m_rawBuf;

    int currentlyShowing { -1 };

    GLuint m_uv;
    GLuint m_pbo[2];
    int m_pboIdx;
    GLuint histogramID;
    GLuint fboID;

  private:
    GLuint loadShaders(const char* vpath, const char *frgPath);
    void reshape(const unsigned int width, const unsigned int height);
    void initTextures();
    void initHistogram();    
    void init();
    CameraIspPipe m_isp;
    std::unique_ptr<std::vector<float>> cameraRotations;

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

    void updatePreviewFrame(void *bytes);
  };
}
