#include "Mesh.hpp"

void Mesh::bindAndDraw(const unsigned int id, Shader* shader) const 
{
    if (texture) {
        texture->bind(shader);
    }

    shader->passUniformLocation("material.diffuse", material.diffuse);
    shader->passUniformLocation("material.specular", material.specular);
    shader->passUniformLocation("material.ambient", material.ambient);
    shader->passUniformLocation("material.shininess", material.shininess);

    glStencilFunc(GL_ALWAYS, id, 0xFF);
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Mesh::init() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(decltype(vertices)::value_type), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(decltype(indices)::value_type), indices.data(), GL_STATIC_DRAW);

    /* Vertex positions */
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(decltype(vertices)::value_type), nullptr);

    /* Texture coordinates */
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(decltype(vertices)::value_type),
        (void*)sizeof(Vertex::position));

    /* Vertex normals */
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(decltype(vertices)::value_type), (void*)(sizeof(Vertex::position) + sizeof(Vertex::textureCoord)));

    /* Tangent coordinates */
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(decltype(vertices)::value_type), (void*)(sizeof(Vertex::position) + sizeof(Vertex::textureCoord) + sizeof(Vertex::normal)));

    glBindVertexArray(0);
}

Mesh::Mesh(std::vector<Vertex> vertices, std::vector<uint32_t> indices, const Material& material) :
    vertices(std::move(vertices)), indices(std::move(indices)), material(material)
{
    for (int i = 0; i < vertices.size(); i++)
    {
        gVertices.push_back(physx::PxVec3(vertices[i].position.x, vertices[i].position.y, vertices[i].position.z));
    }
    init();
}

Mesh::Mesh(Mesh&& mesh) noexcept :
    vertices(std::move(mesh.vertices)), indices(std::move(mesh.indices)),
    vbo(mesh.vbo), vao(mesh.vao), ebo(mesh.ebo), material(mesh.material) 
{
    for (int i = 0; i < vertices.size(); i++)
    {
        gVertices.push_back(physx::PxVec3(vertices[i].position.x, vertices[i].position.y, vertices[i].position.z));
    }
}