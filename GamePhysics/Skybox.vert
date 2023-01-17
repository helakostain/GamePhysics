#version 400

layout (location = 0) in vec3 position;

out vec3 texCoords;

uniform mat4 projectionMatrix;
uniform mat4 viewMatrix;

void main() {
    texCoords = position;
    mat4 view = mat4(mat3(viewMatrix)); //move skybox on camera position
    gl_Position = projectionMatrix * view * vec4(position, 1.0);
}