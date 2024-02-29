#pragma once

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
#include <glm/glm.hpp>

class Player {
public:
    Player(btDynamicsWorld* dynamicsWorld, const btVector3& startPosition);
    ~Player();

    void move(const glm::vec3& direction, float speed);
    void update(float deltaTime);
    btRigidBody* getPlayerRigidBody() const;
    glm::vec3 getPlayerPosition() const;

private:
    btDynamicsWorld* dynamicsWorld;
    btCapsuleShape* capsuleShape;
    btRigidBody* playerRigidBody;
    float dampingFactor = 0.1f;
};