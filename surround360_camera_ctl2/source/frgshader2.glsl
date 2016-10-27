#version 330 core
in vec2 UV;

uniform sampler2D sampler;
out vec3 color;

void main() {
     color.rgb = texture(sampler, UV).rgb;
}
