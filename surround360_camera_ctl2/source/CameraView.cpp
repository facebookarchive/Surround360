/**
 * Copyright (c) 2016-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE_camera_ctl file in the root directory of this subproject.
 */
#include "CameraView.hpp"
#include "PointGrey.hpp"
#include "Config.hpp"
#include <iostream>
#include <cstring>
#include <Shaders.hpp>

#include <fstream>
#include <string>
#include <cassert>
#include <malloc.h>
#include <x86intrin.h>

using namespace surround360;
using namespace std;

static const string defaultIsp =
R"(
{
  "CameraIsp" : {
    "serial" : 0,
    "name" : "PointGrey Grasshopper",
    "bitsPerPixel" : 16,
    "compandingLut" :  [[0.000, 0.000, 0.000],
                        [0.600, 0.600, 0.000],
                        [0.700, 0.700, 0.000],
                        [1.000, 1.000, 0.000]],
    "blackLevel" : [6939.000, 6939.000, 6939.000],
    "clampMin" : [0.000, 0.000, 0.000],
    "clampMax" : [0.962, 0.967, 0.967],
    "vignetteRollOff" : [[1.000, 1.000, 1.000],
                         [1.000, 1.000, 1.000],
                         [1.000, 1.000, 1.000],
                         [1.000, 1.000, 1.000],
                         [1.100, 1.100, 1.100]],
    "whiteBalanceGain" : [1.0, 1.0, 1.0],
    "stuckPixelThreshold" : 0,
    "stuckPixelDarknessThreshold" : 0.000,
    "stuckPixelRadius" : 0,
    "denoise" : 0.000,
    "denoiseRadius" : 0,
    "ccm" : [[1.18936, -0.39975, -0.03933],
             [0.03914, 1.54172, -0.53255],
             [-0.32338, -0.18724, 1.52166]],
    "sharpenning" : [0.000, 0.000, 0.000],
    "saturation" : 1.000,
    "contrast" : 1.000,
    "gamma" : [0.454, 0.454, 0.454],
    "bayerPattern" : "GBRG"
  }
})";

CameraView::CameraView(Gtk::GLArea& glarea)
  : m_glAreaRef(glarea),
    m_pboIdx(0),
    m_rawBuf(nullptr),
    rawBytes(nullptr),
    m_isp(defaultIsp, true, 8) {
}

static const GLfloat vertex_data[] = {
  -1.0f, -1.0f, 0.0f,
  -1.0f,  1.0f, 0.0f,
   1.0f, -1.0f, 0.0f,
   1.0f,  1.0f, 0.0f,
};

static const GLfloat uv_data[] = {
  0.0f, 1.0f,
  0.0f, 0.0f,
  1.0f, 1.0f,
  1.0f, 0.0f,
};

void CameraView::init() {
  auto cam = PointGreyCamera::getCamera(0);
  m_width = cam->frameWidth();
  m_height = cam->frameHeight();
  if (m_rawBuf == nullptr) {
    auto sz = m_width * m_height * 2u;
    m_rawBuf = std::make_shared<aligned_vec_t>(sz, 0);

    sz = m_width * m_height * 3u;
    m_textureBuf = std::make_shared<aligned_vec_t>(sz, 0);
  }

  m_glAreaRef.make_current();

  glEnable(GL_DEBUG_OUTPUT);

  glGenVertexArrays(1, &m_vao);
  glBindVertexArray(m_vao);

  glGenBuffers(1, &m_verts);
  glBindBuffer(GL_ARRAY_BUFFER, m_verts);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_data), vertex_data, GL_STATIC_DRAW);

  glGenBuffers(1, &m_uv);
  glBindBuffer(GL_ARRAY_BUFFER, m_uv);
  glBufferData(GL_ARRAY_BUFFER, sizeof(uv_data), uv_data, GL_STATIC_DRAW);

  glBindBuffer(GL_ARRAY_BUFFER, 0);

  m_program[0] = loadShaders(shaders::vshader, shaders::colorshader);
  m_program[1] = loadShaders(shaders::vshader, shaders::rawshader);

  m_isp.loadImage(&(*m_rawBuf)[0], m_width, m_height);
  m_isp.getImage(&(*m_textureBuf)[0], false);

  initTextures();

  initHistogram();
}

void CameraView::onRealize() {
  try {
    init();
  } catch (std::exception& e) {
    cerr << "Exception:" << e.what() << endl;
    throw e;
  }
}

void CameraView::onUnrealize() {
}

CameraView::~CameraView() {
}

static string readCode(const string& path) {
  ifstream codeStream(path, std::ios::in);
  string code;
  if (codeStream.is_open()) {
    string line = "";
    while (getline(codeStream, line)) {
      code += line + "\n";
    }
    codeStream.close();
  } else {
    cerr << "Can't load shader code from " << path << endl;
    throw "Can't load shader" + path;
  }
  return code;
}

GLuint CameraView::loadShaders(
  const string& vertexshader,
  const string& pixelshader) {
  GLuint vshader = glCreateShader(GL_VERTEX_SHADER);
  GLuint frgShader = glCreateShader(GL_FRAGMENT_SHADER);

  string vertexCode = vertexshader;
  string fragCode = pixelshader;

  char const* vSrc = vertexCode.c_str();
  char const* frgSrc = fragCode.c_str();

  glShaderSource(vshader, 1, &vSrc, nullptr);
  glCompileShader(vshader);

  GLint result = GL_FALSE;
  glGetShaderiv(vshader, GL_COMPILE_STATUS, &result);

  if (!result) {
    int len = 0;
    glGetShaderiv(vshader, GL_INFO_LOG_LENGTH, &len);

    vector<char> errs(len + 1);
    glGetShaderInfoLog(vshader, len, nullptr, &errs[0]);
    printf("Status: %d, Len: %d\n", result, len);
    printf("Vertex shader error: '%s'\n", &errs[0]);
  }

  result = GL_FALSE;
  glShaderSource(frgShader, 1, &frgSrc, nullptr);
  glCompileShader(frgShader);

  glGetShaderiv(frgShader, GL_COMPILE_STATUS, &result);
  if (!result) {
    int len = 0;
    glGetShaderiv(frgShader, GL_INFO_LOG_LENGTH, &len);

    vector<char> errs(len + 1);
    glGetShaderInfoLog(frgShader, len, nullptr, &errs[0]);
    printf("Pixel shader error: '%s'\n", &errs[0]);
  }

  result = GL_FALSE;

  GLuint progId = glCreateProgram();
  glAttachShader(progId, vshader);
  glAttachShader(progId, frgShader);
  glLinkProgram(progId);

  glGetProgramiv(progId, GL_LINK_STATUS, &result);
  if (!result) {
    auto len = 0;
    glGetProgramiv(progId, GL_INFO_LOG_LENGTH, &len);

    vector<char> errs(len + 1);
    glGetProgramInfoLog(progId, len, nullptr, &errs[0]);
    printf("Program error: '%s'\n", &errs[0]);
  }

  glDetachShader(progId, vshader);
  glDetachShader(progId, frgShader);
  glDeleteShader(vshader);
  glDeleteShader(frgShader);

  return progId;
}

void CameraView::reshape(const unsigned int width, const unsigned int height) {
  glMatrixMode(GL_PROJECTION);
  bool raw = CameraConfig::get().raw;
  auto location = glGetUniformLocation(m_program[raw ? 1 : 0], "transform");

  GLfloat scalex = 1.0f;
  GLfloat scaley = 1.0f;

  GLfloat transform[] = {
    scalex, 0.0f,   0.0f, 0.0f,
    0.0f,   scaley, 0.0f, 0.0f,
    0.0f,   0.0f,   1.0f, 0.0f,
    0.0f,   0.0f,   0.0f, 1.0f,
  };

  glUniformMatrix4fv(location, 1, GL_FALSE, transform);
  glMatrixMode(GL_MODELVIEW);
}

void CameraView::initHistogram() {
  GLuint fboID = 0;

  glGenFramebuffers(1, &fboID);
  glBindFramebuffer(GL_FRAMEBUFFER, fboID);
  glGenTextures(1, &histogramID);
  glBindTexture(GL_TEXTURE_1D, histogramID);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexImage1D(GL_TEXTURE_1D, 0,GL_LUMINANCE32F_ARB, 256, 0, GL_LUMINANCE, GL_FLOAT, NULL);
  glFramebufferTexture1D(GL_FRAMEBUFFER_EXT,
                         GL_COLOR_ATTACHMENT0_EXT,
                         GL_TEXTURE_1D,
                         histogramID,
                         0);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

}

void CameraView::initTextures() {
  /* Generate IDs and set params for the texture that will hold camera
     frames */
  glGenTextures(2, m_texture);

  glBindTexture(GL_TEXTURE_2D, m_texture[0]);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexImage2D(
    GL_TEXTURE_2D,
    0,
    GL_RGB,
    m_width,
    m_height,
    0,
    GL_RGB,
    GL_UNSIGNED_BYTE,
    &(*m_textureBuf)[0]);

  glBindTexture(GL_TEXTURE_2D, m_texture[1]);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexImage2D(
    GL_TEXTURE_2D,
    0,
    GL_RED,
    m_width,
    m_height,
    0,
    GL_RED,
    GL_UNSIGNED_SHORT,
    &(*m_rawBuf)[0]);

  glBindTexture(GL_TEXTURE_2D, 0);
}

void CameraView::update() {
  m_glAreaRef.queue_render();
}

bool CameraView::onRender(const Glib::RefPtr<Gdk::GLContext>& context) {
  if (m_rawBuf->empty() || rawBytes == nullptr) {
    glClear(GL_COLOR_BUFFER_BIT);
    return true;
  }

  convertPreviewFrame();

  glClear(GL_COLOR_BUFFER_BIT);
  bool raw = CameraConfig::get().raw;

  glUseProgram(m_program[raw ? 1 : 0]);
  reshape(600, 600);

  /* Bind vertices */
  glBindBuffer(GL_ARRAY_BUFFER, m_verts);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

  /* Bind UVs */
  glBindBuffer(GL_ARRAY_BUFFER, m_uv);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

  /* Bind the 2D texture (our camera frame) */
  if (raw) {
    glBindTexture(GL_TEXTURE_2D, m_texture[1]);
    glTexSubImage2D(
      GL_TEXTURE_2D,
      0,
      0,
      0,
      m_width,
      m_height,
      GL_RED,
      GL_UNSIGNED_SHORT,
      &(*m_rawBuf)[0]);
  } else {
    glBindTexture(GL_TEXTURE_2D, m_texture[0]);
    m_isp.runPipe(&(*m_rawBuf)[0], &(*m_textureBuf)[0], false);
    glTexSubImage2D(
      GL_TEXTURE_2D,
      0,
      0,
      0,
      m_width,
      m_height,
      GL_RGB,
      GL_UNSIGNED_BYTE,
      &(*m_textureBuf)[0]);
  }

  auto rotating = false;
  if (currentlyShowing > -1 && cameraRotations->size() > currentlyShowing) {
    rotating = true;
    glPushMatrix();
    glRotatef(cameraRotations->at(currentlyShowing), 0.0f, .0f, 1.0f);
  }

  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  if (rotating) {
    glPopMatrix();
  }
  glFlush();

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glBindTexture(GL_TEXTURE_2D, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glUseProgram(0);

  return true;
}

void CameraView::updatePreviewFrame(void* bytes) {
  rawBytes = bytes;
}

void CameraView::convertPreviewFrame() {
  auto& bufPtr = *getRawBuffer();
  auto buf = &bufPtr[0];
  auto raw = (uint8_t*)rawBytes;
  uint64_t vaddrRaw = (uint64_t)raw;
  uint64_t vaddrBuf = (uint64_t)buf;
  uint32_t x, y, p;

  const unsigned int bpp = CameraConfig::get().bits;

  p = 0;
  for (y = 0; y < m_height; ++y) {
    for (x = 0; x < m_width; ++x, vaddrBuf += 2) {
#ifdef __AVX2__
      if ((vaddrRaw & 31) == 0 && bpp == 8) {
        break;
      }
#endif
      if (bpp == 12) {
        uint16_t lo = raw[p];
        uint16_t hi = raw[p + 1];
        uint16_t unswizzled, rep;

        if (x & 1) {
          unswizzled = hi << 4 | lo >> 4;
          p += 2;
          vaddrRaw += 2;
        } else {
          unswizzled = lo << 4 | hi & 0xF;
          p += 1;
          ++vaddrRaw;
        }

        rep = unswizzled << 4 | unswizzled >> 8;
        buf[2 * (y * m_width + x)] = rep & 0xFF;
        buf[2 * (y * m_width + x) + 1] = (rep >> 8) & 0xFF;
      } else {
        buf[p * 2]     = raw[p];
        buf[p * 2 + 1] = raw[p];
        ++p;
        ++vaddrRaw;
      }
    }
  }

#ifdef __AVX2__
  assert((vaddrBuf & 31) == 0);
  assert((vaddrRaw & 31) == 0);

  if (bpp == 8) {
    for (; (p + 16) < m_width * m_height; p += 16) {
      __m128i pix  = _mm_load_si128((__m128i*)vaddrRaw);
      __m256i cvt = _mm256_cvtepi8_epi16(pix);
      __m256i hcvt = _mm256_slli_epi16(cvt, 8);

      __m256i rep = _mm256_add_epi16(cvt, hcvt);
      _mm256_stream_si256((__m256i*)vaddrBuf, rep);

      vaddrRaw += 16;
      vaddrBuf += 32;
    }
    for (; p < m_width * m_height; ++p) {
      buf[p * 2] = raw[p];
      buf[p * 2] = raw[p];
    }
  }
#endif
}
