#include "Shader.hpp"

void Shader::VertexShader()
{
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertex_shader, NULL);
	glCompileShader(vertexShader);
}

void Shader::FragmentShader()
{
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragment_shader, NULL);
	glCompileShader(fragmentShader);
}

void Shader::ShaderProgram()
{
	shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, fragmentShader);
	glAttachShader(shaderProgram, vertexShader);
	glLinkProgram(shaderProgram);
}

void Shader::LinkCheck()
{
	GLint status;
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &status);
	if (status == GL_FALSE)
	{
		GLint infoLogLength;
		glGetProgramiv(shaderProgram, GL_INFO_LOG_LENGTH, &infoLogLength);
		GLchar* strInfoLog = new GLchar[infoLogLength + 1];
		glGetProgramInfoLog(shaderProgram, infoLogLength, NULL, strInfoLog);
		fprintf(stderr, "Linker failure: %s\n", strInfoLog);
		delete[] strInfoLog;
	}
}

void Shader::ShaderRun() //create and compile shaders
{
	VertexShader();
	FragmentShader();
	ShaderProgram();

	LinkCheck();
}

void Shader::passUniformLocation(const std::string& var, const glm::mat4& matrix) const
{
	passUniformLocation(var.c_str(), matrix);
}

void Shader::passUniformLocation(const char* var, const glm::mat4& matrix) const
{
	const auto model = getUniformLocation(var);
	if (model >= 0) 
	{
		glProgramUniformMatrix4fv(shaderProgram, model, 1, GL_FALSE, glm::value_ptr(matrix));
	}
}

void Shader::passUniformLocation(const std::string& var, const glm::vec3& vector) const
{
	passUniformLocation(var.c_str(), vector);
}

void Shader::passUniformLocation(const char* var, const glm::vec3& vector) const
{
	const auto location = getUniformLocation(var);
	if (location >= 0) 
	{
		glProgramUniform3f(shaderProgram, location, vector.x, vector.y, vector.z);
	}
}

void Shader::passUniformLocation(const std::string& var, int32_t value) const
{
	passUniformLocation(var.c_str(), value);
}

void Shader::passUniformLocation(const std::string& var, const float value) const 
{
	passUniformLocation(var.c_str(), value);
}

void Shader::passUniformLocation(const std::string& var, const glm::mat3& matrix) const
{
	passUniformLocation(var.c_str(), matrix);
}

void Shader::passUniformLocation(const char* var, const glm::mat3& matrix) const
{
	const auto location = getUniformLocation(var);
	if (location >= 0) 
	{
		glProgramUniformMatrix3fv(shaderProgram, location, 1, GL_FALSE, glm::value_ptr(matrix));
	}
}

GLint Shader::getUniformLocation(const std::string& var) const
{
	return getUniformLocation(var.c_str());
}

GLint Shader::getUniformLocation(const char* var) const
{
	auto location = glGetUniformLocation(shaderProgram, var);
	if (location < 0) 
	{
		// std::cout << "Uniform variable '" << var << "' not found." << std::endl;
	}
	return location;
}

void Shader::applyLight(ColoredLight& light)
{
	colorChanged(light.getColor(), light.lightIndex, light.type());
	typeChanged(light.type(), light.lightIndex);
}

void Shader::applyLight(PositionedLight& light)
{
	passUniformLocation("lights[" + std::to_string(light.lightIndex) + "].position", light.getPosition());
	passUniformLocation("lights[" + std::to_string(light.lightIndex) + "].lightType", int32_t(light.type()));
}

void Shader::applyLight(DirectionalLight& light)
{
	passUniformLocation("lights[" + std::to_string(light.lightIndex) + "].direction", light.getDirection());
	passUniformLocation("lights[" + std::to_string(light.lightIndex) + "].lightType", int32_t(light.type()));
}

void Shader::applyLight(Spotlight& light)
{
	passUniformLocation("lights[" + std::to_string(light.lightIndex) + "].position", light.getPosition());
	passUniformLocation("lights[" + std::to_string(light.lightIndex) + "].direction", light.getDirection());
	passUniformLocation("lights[" + std::to_string(light.lightIndex) + "].cutoff", glm::cos(glm::radians(light.getCutoff())));
	passUniformLocation("lights[" + std::to_string(light.lightIndex) + "].lightType", int32_t(light.type()));
}

void Shader::typeChanged(gl::Light type, size_t lightIndex)
{
	passUniformLocation("lights[" + std::to_string(lightIndex) + "].lightType", int32_t(type));
}

Shader::Shader(const char* vertex_path, const char* fragment_path) : ShaderLoader(vertex_path, fragment_path, &this->shaderProgram)
{
	this->vertex_shader = vertex_path;
	this->fragment_shader = fragment_path;
}

void Shader::UpdateMatrix()
{
}

void Shader::setMatrix(glm::mat4 modelMatrix)
{
	GLuint MatrixID = glGetUniformLocation(this->shaderProgram, "modelMatrix");
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &modelMatrix[0][0]);
}

void Shader::Init()
{
	glUseProgram(shaderProgram);
}

void Shader::shaderUseProgram()
{
	glUseProgram(shaderProgram);
}

void Shader::updateView(const glm::mat4& view)
{
	passUniformLocation("viewMatrix", view);
}

void Shader::updateProjection(const glm::mat4& projection)
{
	passUniformLocation("projectionMatrix", projection);
}

void Shader::updatePosition(const glm::vec3& position)
{
	passUniformLocation("cameraPosition", position);
}

void Shader::colorChanged(glm::vec3 color, size_t lightIndex, gl::Light lightType)
{
	if (lightType == gl::Light::Ambient) 
	{
		passUniformLocation("ambientColor", color);
	}
	else if (lightType == gl::Light::Point or lightType == gl::Light::Directional) 
	{
		passUniformLocation("lights[" + std::to_string(lightIndex) + "].lightColor", color);
	}
}

void Shader::positionChanged(glm::vec3 position, size_t lightIndex, gl::Light lightType)
{
	if (lightType == gl::Light::Ambient) 
	{
		/* NOTHING ambient light has no coordinates */
	}
	else if (lightType == gl::Light::Point) 
	{
		passUniformLocation("lights[" + std::to_string(lightIndex) + "].position", position);
	}
	else if (lightType == gl::Light::Directional)
	{
		passUniformLocation("lights[" + std::to_string(lightIndex) + "].direction", position);
	}
	else if (lightType == gl::Light::Spotlight) 
	{
		passUniformLocation("lights[" + std::to_string(lightIndex) + "].position", position);
	}
}

void Shader::notify(EventType eventType, void* object)
{
	if (eventType == EventType::LightChanged)
	{
		ColoredLight& light = *(ColoredLight*)object;
		applyLight(light);
		if (light.type() == gl::Light::Point)
		{
			PositionedLight& pol = *(PositionedLight*)object;
			applyLight(pol);
		}
		else if (light.type() == gl::Light::Directional)
		{
			DirectionalLight& dil = *(DirectionalLight*)object;
			applyLight(dil);
		}
		else if (light.type() == gl::Light::Spotlight)
		{
			Spotlight& spl = *(Spotlight*)object;
			applyLight(spl);
		}
	}
	else if (eventType == EventType::CameraMoved)
	{
		Camera& camera = *((Camera*)object);
		updateView(camera.view());
		updatePosition(camera.position());
		updateProjection(camera.project());
	}
}

void Shader::passUniformLocation(const char* var, int32_t value) const
{
	const auto location = getUniformLocation(var);
	if (location >= 0) 
	{
		glProgramUniform1i(shaderProgram, location, value);
	}
}

void Shader::passUniformLocation(const char* var, const float value) const 
{
	const auto location = getUniformLocation(var);
	if (location >= 0) 
	{
		glProgramUniform1f(shaderProgramID, location, value);
	}
}