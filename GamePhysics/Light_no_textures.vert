#version 400
uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;
out vec4 ex_worldPosition;
out vec3 ex_worldNormal;

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;

void main () {
    vec4 pos4 = vec4(position, 1.0);
    gl_Position = (projectionMatrix * viewMatrix * modelMatrix) * pos4;
    ex_worldPosition = modelMatrix * pos4;
    ex_worldNormal = normalize(transpose(inverse(mat3(modelMatrix))) * normal);
}