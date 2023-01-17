#version 400

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

out vec4 ex_worldPosition;
out vec3 ex_worldNormal;

void main() {
    vec4 pos4 = vec4(position, 1.0);
    gl_Position = (projectionMatrix * viewMatrix) * pos4;
    ex_worldPosition = pos4;
    ex_worldNormal = normal;
}