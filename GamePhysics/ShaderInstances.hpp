#pragma once
#include "Shader.hpp"
class ShaderInstances {
private:
	Shader _constant;
	Shader _phong;
	Shader _terrain;
	Shader _skybox;
	Shader _phong_no_textures;
	Shader _phong_norm;

	static ShaderInstances* sInstance;
	static ShaderInstances& instance();

	ShaderInstances();
public:
	static Shader& constant();
	static Shader& phong();
	static Shader& skybox();
	static Shader& terrain();
	static Shader& phong_no_textures();
	static Shader& phong_norm();
};