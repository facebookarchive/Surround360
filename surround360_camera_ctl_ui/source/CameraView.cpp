/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
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
#include <algorithm>
#include <malloc.h>
#include <x86intrin.h>

using namespace surround360;
using namespace std;

enum { colorProgram = 0, rawProgram = 1, histogramProgram = 2 };

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
    "vignetteRollOffH" : [[1.1, 1.1, 1.1],
                          [1.0, 1.0, 1.0],
                          [1.0, 1.0, 1.0],
                          [1.0, 1.0, 1.0],
                          [1.1, 1.1, 1.1]],
    "vignetteRollOffV" : [[1.1, 1.1, 1.1],
                          [1.0, 1.0, 1.0],
                          [1.0, 1.0, 1.0],
                          [1.0, 1.0, 1.0],
                          [1.1, 1.1, 1.1]],
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
    m_isp(defaultIsp, true, 8) {
  m_normalized.resize(256);
  m_histogram.resize(256);
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

  glGenBuffers(1, &m_histogramGeometry);

  glGenBuffers(1, &m_uv);
  glBindBuffer(GL_ARRAY_BUFFER, m_uv);
  glBufferData(GL_ARRAY_BUFFER, sizeof(uv_data), uv_data, GL_STATIC_DRAW);

  glBindBuffer(GL_ARRAY_BUFFER, 0);

  m_program[colorProgram]     = loadShaders(shaders::vshader, shaders::colorshader);
  m_program[rawProgram]       = loadShaders(shaders::vshader, shaders::rawshader);
  m_program[histogramProgram] = loadShaders(shaders::histogramVertexShader,
                                            shaders::histogramPixelShader);

  m_isp.loadImage(&(*m_rawBuf)[0], m_width, m_height);
  m_isp.getImage(&(*m_textureBuf)[0], false);

  initTextures();
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

void CameraView::reshape(GLuint progid) {
  glMatrixMode(GL_PROJECTION);
  bool raw = CameraConfig::get().raw;
  auto location = glGetUniformLocation(progid, "transform");

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
  if (m_rawBuf->empty() || rawBytes.empty()) {
    glClear(GL_COLOR_BUFFER_BIT);
    return true;
  }

  convertPreviewFrame();

  glDisable(GL_BLEND);
  glClear(GL_COLOR_BUFFER_BIT);
  const bool raw = CameraConfig::get().raw;

  glUseProgram(m_program[raw ? rawProgram : colorProgram]);
  reshape(m_program[raw ? rawProgram : colorProgram]);

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

  // compute histogram geometry
  const float barWidth = 0.0039f;
  vector<float> histogramGeometry;

  for (auto k = 0; k < m_normalized.size(); ++k) {
    // for each bar, we use 2 triangles
    // first triangle
    histogramGeometry.push_back(-1.f + k * barWidth);
    histogramGeometry.push_back(-.8f);
    histogramGeometry.push_back(0.0f);

    histogramGeometry.push_back(-1.f + k * barWidth);
    histogramGeometry.push_back(-.8f + 0.25f/20.0f * m_normalized[k]);
    histogramGeometry.push_back(0.0f);

    histogramGeometry.push_back(-1.f + (k + 1) * barWidth);
    histogramGeometry.push_back(-.8f);
    histogramGeometry.push_back(0.0f);

    // second triangle
    histogramGeometry.push_back(-1.f + k * barWidth);
    histogramGeometry.push_back(-.8f + 0.25f/20.0f * m_normalized[k]);
    histogramGeometry.push_back(0.0f);

    histogramGeometry.push_back(-1.f + (k + 1) * barWidth);
    histogramGeometry.push_back(-.8f);
    histogramGeometry.push_back(0.0f);

    histogramGeometry.push_back(-1.f + (k + 1) * barWidth);
    histogramGeometry.push_back(-.8f + 0.25f/20.0f * m_normalized[k]);
    histogramGeometry.push_back(0.0f);
  }

  // switch to histogram drawing
  glUseProgram(m_program[histogramProgram]);
  glBindTexture(GL_TEXTURE_2D, 0);

  reshape(m_program[histogramProgram]);

  // disable blending
  glDisable(GL_BLEND);

  glBindBuffer(GL_ARRAY_BUFFER, m_histogramGeometry);
  glBufferData(GL_ARRAY_BUFFER,
               histogramGeometry.size() * sizeof(float),
               &histogramGeometry[0], GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
  glDisableVertexAttribArray(1);

  glDrawArrays(GL_TRIANGLES, 0, histogramGeometry.size());
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glBindTexture(GL_TEXTURE_2D, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glUseProgram(0);

  return true;
}

void CameraView::updatePreviewFrame(const void* bytes, const size_t size, const uint32_t bpp) {
  rawBytes.resize(size);
  memcpy(rawBytes.data(), bytes, size);
  bitsPerPixelForPreview = bpp;
}

void CameraView::convertPreviewFrame() {
  auto& bufPtr = *getRawBuffer();
  auto buf = &bufPtr[0];
  auto raw = (uint8_t*)rawBytes.data();
  uint64_t vaddrRaw = (uint64_t)raw;
  uint64_t vaddrBuf = (uint64_t)buf;
  uint32_t x, y, p;

  fill(m_histogram.begin(), m_histogram.end(), 0);

  auto bpp = bitsPerPixelForPreview;
  
  p = 0;
  for (y = 0; y < m_height; ++y) {
    for (x = 0; x < m_width; ++x, vaddrBuf += 2) {
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

        ++m_histogram[(unswizzled >> 4) & 0xFF];
        buf[2 * (y * m_width + x)] = rep & 0xFF;
        buf[2 * (y * m_width + x) + 1] = (rep >> 8) & 0xFF;
      } else {
        ++m_histogram[raw[p] & 0xFF];
        buf[p * 2]     = raw[p];
        buf[p * 2 + 1] = raw[p];
        ++p;
        ++vaddrRaw;
      }
    }
  }

  for (auto k = 0; k < m_normalized.size(); ++k) {
    m_normalized[k] = std::log(float(m_histogram[k])) / std::log(2.0f);
  }
}
