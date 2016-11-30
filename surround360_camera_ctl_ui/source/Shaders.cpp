/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include <Shaders.hpp>

const std::string surround360::shaders::colorshader{
R"(#version 330 core
in vec2 UV;
uniform sampler2D sampler;
out vec3 color;
void main() {
  color.rgb = texture(sampler, UV).rgb;
}
)"};

const std::string surround360::shaders::rawshader{
R"(#version 330 core
in vec2 UV;
uniform sampler2D sampler;
out vec3 color;
const float epsilon = 0.001f;

void main() {
  float val = texture(sampler, UV).r;
  vec3 grayscale = vec3(val, val, val);

  if (abs(val - 0.97f) <= epsilon) {
    color.rgb = vec3(1.0f, 0.0f, 0.0f);
  } else if (abs(val - 0.16f) <= epsilon) {
    color.rgb = vec3(0.0f, 0.0f, 1.0f);
  } else {
    color.rgb = grayscale;
  }
}
)"};

const std::string surround360::shaders::vshader{
R"(#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec2 vertexUV;
out vec2 UV;
uniform mat4 transform;
void main() {
  gl_Position = transform * vec4(position, 1.0f);
  UV = vertexUV;
})"};

const std::string surround360::shaders::histogramVertexShader{
R"(
#version 330 core
layout(location = 0) in vec3 position;
uniform mat4 transform;

void main() {
  gl_Position = transform * vec4(position, 1.0f);
}
)"};

const std::string surround360::shaders::histogramPixelShader{
R"(
#version 330 core
out vec4 color;
void main() {
  color = vec4(1.0f, 1.0f, 1.0f, 1.0f);
})"};
