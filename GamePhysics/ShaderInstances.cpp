#include "ShaderInstances.hpp"

ShaderInstances* ShaderInstances::sInstance = nullptr;

ShaderInstances& ShaderInstances::instance()
{
    if (not sInstance)
    {
        sInstance = new ShaderInstances();
    }
    return *sInstance;
}

ShaderInstances::ShaderInstances() :
    _constant("Default.vert", "Default.frag"),
    _phong("Light.vert", "Phong.frag"),
    _skybox("Skybox.vert", "Skybox.frag"),
    _terrain("Terrain.vert", "Terrain.frag"),
    _phong_no_textures("Light_no_textures.vert", "Phong_no_textures.frag"),
    _phong_norm("Normal_map.vert", "Phong_norm.frag")
{}

Shader& ShaderInstances::constant()
{
    return instance()._constant;
}

Shader& ShaderInstances::phong()
{
    return instance()._phong;
}

Shader& ShaderInstances::skybox()
{
    return instance()._skybox;
}

Shader& ShaderInstances::terrain()
{
    return instance()._terrain;
}

Shader& ShaderInstances::phong_no_textures()
{
    return instance()._phong_no_textures;
}

Shader& ShaderInstances::phong_norm()
{
    return instance()._phong_norm;
}