#pragma once
#include <memory>
#include "Shader.hpp"
#include "Texture.hpp"
#include "ShaderInstances.hpp"

class Skybox {
    std::vector<float> points;
    std::shared_ptr<Texture> cubeMap;
    Shader* shader;
    GLuint VBO = 0;
    GLuint VAO = 0;

public:
    Skybox(std::shared_ptr<Texture> cubeMap);
    void Draw();
    void Init();
    void Bind();
};