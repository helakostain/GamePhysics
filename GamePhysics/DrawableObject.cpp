#include "DrawableObject.hpp"

DrawableObject::DrawableObject()
{
}

DrawableObject::DrawableObject(Models* model, const char* vertex_path, const char* fragment_path, int size)
{
    this->models = model;
    this->models->Init();
    this->shaders = new Shader(vertex_path, fragment_path);
    this->transformations = new Transformation();
    this->isObject = false;
    this->id = size;
    this->size_points = model->get_size_points();
}

DrawableObject::DrawableObject(Models* model, Shader& shader, int size)
{
    this->models = model;
    this->models->Init();
    this->shaders = &shader;
    this->transformations = new Transformation();
    this->texture = nullptr;
    this->isObject = false;
    this->id = size;
    this->size_points = model->get_size_points();
}

DrawableObject::DrawableObject(Models* model, Shader& shader, shared_ptr<MovementCalculator> movement, int size)
{
    this->models = model;
    this->models->Init();
    this->shaders = &shader;
    this->transformations = new Transformation();
    this->texture = nullptr;
    this->isObject = false;
    this->id = size;
    this->movementCalculator = movement;
    this->size_points = model->get_size_points();
}

DrawableObject::DrawableObject(Models* model, Shader& shader, shared_ptr<Texture> texture, int size)
{
    this->models = model;
    this->models->Init();
    this->shaders = &shader;
    this->transformations = new Transformation();
    this->texture = texture;
    this->isObject = false;
    this->id = size;
    this->size_points = model->get_size_points();
}

DrawableObject::DrawableObject(Models* model, Shader& shader, shared_ptr<Texture> texture, int size, bool object)
{
    this->models = model;
    this->shaders = &shader;
    this->transformations = new Transformation();
    this->texture = texture;
    this->isObject = object;
    this->id = size;
    this->size_points = model->get_size_points();
}

DrawableObject::DrawableObject(Models* model, Shader& shader, shared_ptr<Texture> texture, shared_ptr<MovementCalculator> movement, int size, bool object)
{
    this->movementCalculator = movement;
    this->models = model;
    this->shaders = &shader;
    this->transformations = new Transformation();
    this->texture = texture;
    this->isObject = object;
    this->id = size;
    this->size_points = model->get_size_points();
}

void DrawableObject::DoTransformations(const double delta)
{
    this->transformations->Update(delta);
}

void DrawableObject::sendShaderMatrix()
{
    this->shaders->setMatrix(this->transformations->matrix());
}

bool DrawableObject::SetUp()
{
    this->shaders->Init();
    this->models->Bind();
    return true;
}

void DrawableObject::Pos_scale(float a)
{
    this->transformations->scale(a);
}

void DrawableObject::setFy(Direction dir)
{
    this->transformations->applyFy(dir);
}
void DrawableObject::setFx(Direction dir)
{
    this->transformations->applyFx(dir);
}
void DrawableObject::setRot(Rotation r)
{
    this->transformations->setRotation(r);
}
void DrawableObject::setGrow(Growth g)
{
    this->transformations->setGrowth(g);
}

void DrawableObject::Pos_mov(glm::vec3 a)
{
    this->currPosition = a;
    this->transformations->translate(a);
}

void DrawableObject::Draw()
{
    this->models->Draw();
}

void DrawableObject::updateObject(const float delta)
{
    this->shaders->shaderUseProgram();
    if (!isObject)
    {
        models->Bind();
        glStencilFunc(GL_ALWAYS, id + 1, 0xFF);
    }
    transformations->Update(delta);
    if (movementCalculator != nullptr)
        this->updateMovement(delta);
    sendShaderMatrix();

    if (this->texture != nullptr)
    {
        this->texture->bind(this->shaders);
    }

    if (isObject)
    {
        models->draw(id + 1, this->shaders);
    }
    else
    {
        Draw();
    }
}

void DrawableObject::applyTexture(std::shared_ptr<Texture> texture)
{
}

void DrawableObject::rotate(float degree, glm::vec3 axis)
{
    this->transformations->rotate(degree, axis);
}

void DrawableObject::updateMovement(double delta)
{
    movementCalculator->update(delta);
    transformations->setPosition(movementCalculator->currentPosition());
    transformations->rotate(movementCalculator->currentHeading().x, glm::vec3(1, 0, 0));
    transformations->rotate(movementCalculator->currentHeading().y, glm::vec3(0, 1, 0));
    transformations->rotate(movementCalculator->currentHeading().z, glm::vec3(0, 0, 1));
}

int DrawableObject::getId()
{
    return id;
}

int DrawableObject::getSizePoints()
{
    return this->size_points;
}

Models* DrawableObject::getModel()
{
    return this->models;
}

Transformation* DrawableObject::getTransformation()
{
    return this->transformations;
}

Shader& DrawableObject::getShader()
{
    return *this->shaders;
}