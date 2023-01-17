#pragma once
#include "MovementCalculator.hpp"
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <memory>
#include <functional>

class MoveLine : public MovementCurve
{
    const glm::vec3 begin;
    const glm::vec3 end;
    const glm::vec3 dir;
    glm::vec3 calcDir(const glm::vec3 begin, const glm::vec3 end);

public:
    MoveLine(glm::vec3 begin, glm::vec3 end);

    glm::vec3 position(float t) const override;
    glm::vec3 direction(float t) const override;
};