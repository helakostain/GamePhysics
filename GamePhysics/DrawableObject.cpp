#include "DrawableObject.hpp"

DrawableObject::DrawableObject()
{
}

DrawableObject::DrawableObject(Models* model, const char* vertex_path, const char* fragment_path, int size)
{
    this->models = model;
    this->models->Init();
    this->getModel()->setShader(new Shader(vertex_path, fragment_path));
    this->isObject = false;
    this->id = size;
    this->size_points = model->get_size_points();
}

DrawableObject::DrawableObject(Models* model, Shader& shader, int size)
{
    this->models = model;
    this->models->Init();
    this->getModel()->setShader(&shader);
    this->texture = nullptr;
    this->isObject = false;
    this->id = size;
    this->size_points = model->get_size_points();
}

DrawableObject::DrawableObject(Models* model, Shader& shader, shared_ptr<MovementCalculator> movement, int size)
{
    this->models = model;
    this->models->Init();
    this->getModel()->setShader(&shader);
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
    this->getModel()->setShader(&shader);
    this->texture = texture;
    this->isObject = false;
    this->id = size;
    this->size_points = model->get_size_points();
}

DrawableObject::DrawableObject(Models* model, Shader& shader, shared_ptr<Texture> texture, int size, bool object, int actorType)
{
    this->models = model;
    this->getModel()->setShader(&shader);
    this->texture = texture;
    this->isObject = object;
    this->id = size;
    this->size_points = model->get_size_points();
    this->actorType = actorType;
}

DrawableObject::DrawableObject(Models* model, Shader& shader, shared_ptr<Texture> texture, shared_ptr<MovementCalculator> movement, int size, bool object)
{
    this->movementCalculator = movement;
    this->models = model;
    this->getModel()->setShader(&shader);
    this->texture = texture;
    this->isObject = object;
    this->id = size;
    this->size_points = model->get_size_points();
}
void DrawableObject::sendShaderMatrix(int i)
{
    this->getModel()->shaders[i]->setMatrix(this->getModel()->getTransformation(i)->matrix());
}

bool DrawableObject::SetUp()
{
    for (int i = 0; i < this->getModel()->meshes.size(); i++)
    {
        this->getModel()->shaders[i]->Init();
    }
    this->models->Bind();
    return true;
}

void DrawableObject::Draw()
{
    this->models->Draw();
}

void DrawableObject::updateObject(const float delta)
{
    for (int i = 0; i < this->getModel()->meshes.size(); i++)
    {
        this->getModel()->shaders[i]->shaderUseProgram();

        if (!isObject)
        {
            models->Bind();
            glStencilFunc(GL_ALWAYS, id + 1, 0xFF);
        }
        this->models->DoTransformations(delta, i);
        if (movementCalculator != nullptr)
            this->updateMovement(delta); //NEFUNKCNI rozbije physx
        sendShaderMatrix(i);

        if (this->texture != nullptr)
        {
            this->texture->bind(this->getModel()->shaders[i]);

        }

        if (isObject)
        {
            models->draw(id + 1, i);

        }
        else
        {
            Draw();
        }
    }
}

void DrawableObject::applyTexture(std::shared_ptr<Texture> texture)
{
}

void DrawableObject::updateMovement(double delta)
{
    movementCalculator->update(delta);
    this->models->setPos(movementCalculator->currentPosition());
    this->models->rotate(movementCalculator->currentHeading().x, glm::vec3(1, 0, 0));
    this->models->rotate(movementCalculator->currentHeading().y, glm::vec3(0, 1, 0));
    this->models->rotate(movementCalculator->currentHeading().z, glm::vec3(0, 0, 1));
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

int DrawableObject::getActorType()
{
    return this->actorType;
}