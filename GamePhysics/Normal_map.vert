#version 400

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

layout(location=0) in vec3 position;
layout(location=1) in vec2 in_uv;
layout(location=2) in vec3 normal;
layout(location=3) in vec3 in_tangent;

out VS_OUT {
    vec3 FragPos;
    vec2 TexCoords;
    vec3 NormalPos;
    mat3 TBN;
} vs_out;

void main () {

    gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(position, 1.0);

    vs_out.FragPos = vec3(modelMatrix * vec4(position, 1.0)); 

    mat3 normalMatrix = transpose(inverse(mat3(modelMatrix)));     

    vec3 T = normalize(vec3(normalMatrix * normalize(in_tangent)));
    vec3 bitangent = cross(normalize(normal) , normalize(in_tangent));
    vec3 B = normalize(vec3(normalMatrix * normalize(bitangent)));
    vec3 N = normalize(vec3(normalMatrix * normalize(normal)));

    mat3 TBN = mat3(T, B, N);    

    vs_out.NormalPos = normalMatrix * normal;
    vs_out.TexCoords = in_uv;
    vs_out.TBN = TBN;
}