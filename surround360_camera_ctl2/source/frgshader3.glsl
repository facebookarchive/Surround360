#version 330 core
in vec2 UV;

uniform sampler2D sampler;
out vec3 color;

const float epsilon = 0.001f;
void main() {
  float val = texture(sampler, UV).r;
  if (abs(val - 0.97f) <= epsilon) {
    color.rgb = vec3(1.0f, 0.0f, 0.0f);
  } else if (abs(val - 0.16f) <= epsilon) {
    color.rgb = vec3(0.0f, 0.0f, 1.0f);
  } else {
    color.rgb = vec3(val, val, val);
  }
}
