#pragma once
#include <memory>
#include <functional>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct MovementCurve {
    virtual glm::vec3 position(float t) const = 0;
    virtual glm::vec3 direction(float t) const = 0;
};

class MovementCalculator {
    std::shared_ptr<MovementCurve> curve;
    const glm::vec3 offset;
    const float duration;
    float t = 0.0;

    float currentT() const;
public:
    MovementCalculator(std::shared_ptr<MovementCurve> curve, glm::vec3 rotationOffset, float duration);

    void update(float dt);
    glm::vec3 currentPosition() const;
    glm::vec3 currentHeading() const;
};