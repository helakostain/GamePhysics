#pragma once
#include <glm/mat4x4.hpp>
#include <GL/glew.h>
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>

#include "Texture.hpp"
#include "Shader.hpp"

struct Vertex {
    glm::vec3 position;
    glm::vec2 textureCoord;
    glm::vec3 normal;
    glm::vec3 tangents;
};

struct Material {
    glm::vec3 diffuse{ 0.f };
    glm::vec3 specular{ 0.f };
    glm::vec3 ambient{ 0.f };
    std::string diffuseMap;
    std::string specularMap;
    std::string heightMap;
    float shininess = 0.f;
};

class Mesh
{
    GLuint vbo = 0;
    GLuint vao = 0;
    GLuint ebo = 0;

    
    std::shared_ptr<Texture> texture;

    void init();
public:
    Mesh(std::vector<Vertex> vertices, std::vector<uint32_t> indices, const Material& material);
    Mesh(Mesh&& mesh) noexcept;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    const Material& material;

    void bindAndDraw(unsigned int id, Shader* shader) const;
};