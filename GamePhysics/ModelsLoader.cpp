#include "ModelsLoader.hpp"
#include "Shader.hpp"

std::unordered_map<std::string, Models*> ModelsLoader::models;

Models* ModelsLoader::get(const std::string& key) 
{
    auto iter = models.find(key);

    //if (iter != models.end()) 
    //{
    //    return iter->second;
    //}
    return loadModel(key);
}

Mesh ModelsLoader::processMesh(aiMesh& mesh, const aiScene& scene, const Models& model) 
{
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    processVertices(mesh, vertices);
    processIndices(mesh, indices);
    const Material& mat = model.materials[mesh.mMaterialIndex];

    return Mesh(std::move(vertices), std::move(indices), mat);
}

void ModelsLoader::processNode(aiNode& node, const aiScene& scene, Models* model) 
{
    for (uint32_t i = 0; i < node.mNumMeshes; ++i) 
    {
        aiMesh* mesh = scene.mMeshes[node.mMeshes[i]];
        model->addMesh(processMesh(*mesh, scene, *model));
    }
    for (uint32_t i = 0; i < node.mNumChildren; ++i) 
    {
        processNode(*(node.mChildren[i]), scene, model);
    }
}

Models* ModelsLoader::loadModel(const std::string& key) 
{
    auto path = formatName(key);

    Assimp::Importer importer;
    const aiScene* aiscene = importer.ReadFile(path, importOpts);

    if (!aiscene) 
    {
        throw std::runtime_error("Error loading model '" + key + "'");
    }

    const aiScene& scene = *aiscene;

    if (scene.mFlags bitand AI_SCENE_FLAGS_INCOMPLETE or not scene.mRootNode) 
    {
        throw std::runtime_error("Error loading model '" + key + "'");
    }

    auto model = new Models();
    model->directory = path.substr(0, path.find_last_of('/'));
    processMaterials(scene, model->materials);
    processNode(*scene.mRootNode, scene, model);

    models.emplace(key, model);

    return model;
}

std::string ModelsLoader::formatName(const std::string& model) 
{
    return "Models/" + model + ".obj";
}

void ModelsLoader::processMaterials(const aiScene& scene, std::vector<Material>& materials) 
{
    for (uint32_t i = 0; i < scene.mNumMaterials; ++i) 
    {
        materials.emplace_back();
        aiMaterial& mat = *scene.mMaterials[i];
        loadMaterial(mat, materials.back());
    }

}

Material ModelsLoader::loadMaterial(aiMaterial& mat, Material& material) 
{
    aiColor3D color{ 0.f, 0.f, 0.f };

    mat.Get(AI_MATKEY_COLOR_DIFFUSE, color);
    material.diffuse = toVector(color);

    mat.Get(AI_MATKEY_COLOR_AMBIENT, color);
    material.ambient = toVector(color);

    mat.Get(AI_MATKEY_COLOR_SPECULAR, color);
    material.specular = toVector(color);

    mat.Get(AI_MATKEY_SHININESS, material.shininess);

    aiString path;

    if (mat.GetTexture(aiTextureType_DIFFUSE, 0, &path) == AI_SUCCESS) 
    {
        material.diffuseMap = path.C_Str();
    }
    if (mat.GetTexture(aiTextureType_SPECULAR, 0, &path) == AI_SUCCESS) 
    {
        material.specularMap = path.C_Str();
    }
    if (mat.GetTexture(aiTextureType_HEIGHT, 0, &path) == AI_SUCCESS) 
    {
        material.heightMap = path.C_Str();
    }

    return material;
}

glm::vec3 ModelsLoader::toVector(const aiColor3D& color) 
{
    return glm::vec3{ color.r, color.g, color.b };
}

void ModelsLoader::processVertices(aiMesh& mesh, std::vector<Vertex>& vertices) 
{
    for (uint32_t i = 0; i < mesh.mNumVertices; ++i) 
    {
        vertices.emplace_back();
        vertices.back().position = extractVector(mesh.mVertices[i]);
        if (mesh.mTextureCoords[0]) 
        {
            auto vec = extractVector(mesh.mTextureCoords[0][i]);
            vertices.back().textureCoord = glm::vec2{ vec.x, vec.y };
        }
        else 
        {
            vertices.back().textureCoord = glm::vec2{ 0.f, 0.f };
        }
        vertices.back().normal = extractVector(mesh.mNormals[i]);
        vertices.back().tangents = extractVector(mesh.mTangents[i]);
    }
}

void ModelsLoader::processIndices(aiMesh& mesh, std::vector<uint32_t>& indices) 
{
    for (uint32_t i = 0; i < mesh.mNumFaces; ++i) 
    {
        aiFace face = mesh.mFaces[i];
        for (uint32_t j = 0; j < face.mNumIndices; ++j) 
        {
            indices.emplace_back(face.mIndices[j]);
        }
    }
}

glm::vec3 ModelsLoader::extractVector(aiVector3D& vec) 
{
    return glm::vec3{ vec.x, vec.y, vec.z };
}