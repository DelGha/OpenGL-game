#include "Player.h"
#include <iostream>

using namespace std;

    Player::Player(btDynamicsWorld* dynamicsWorld, const btVector3& startPosition)
        : dynamicsWorld(dynamicsWorld) {
        // Create a capsule shape for the player
        capsuleShape = new btCapsuleShape(0.5, 2);

        // Create a rigid body for the player
        btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(1.0, nullptr, capsuleShape);
        playerRigidBody = new btRigidBody(rigidBodyCI);
        playerRigidBody->setUserPointer(this); // Set a pointer to this controller for identification in contact callbacks
        playerRigidBody->setAngularFactor(btVector3(0, 1, 0)); // Disable rotation

        // Set the initial position
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(startPosition);
        playerRigidBody->setWorldTransform(startTransform);

        // Add the player rigid body to the dynamics world
        dynamicsWorld->addRigidBody(playerRigidBody);

        playerRigidBody->setFriction(1);
        playerRigidBody->setDamping(0.8, 0.8);
    }

    Player::~Player() {
        dynamicsWorld->removeRigidBody(playerRigidBody);
        delete playerRigidBody;
        delete capsuleShape;
    }

    void Player:: move(const glm::vec3& direction, float speed) {
        // Apply a force to move the player in the desired direction
        playerRigidBody->activate(true);
        btVector3 btDirection(direction.x, direction.y, direction.z);
        btDirection.normalize();

        float currentSpeed = playerRigidBody->getLinearVelocity().length();
        float maxSpeed = 100;

        if(currentSpeed < maxSpeed)
            playerRigidBody->applyCentralForce(btDirection * speed);
        else {
            btVector3 clampedVelocity = playerRigidBody->getLinearVelocity().normalized() * maxSpeed;
            playerRigidBody->setLinearVelocity(clampedVelocity);
        }
    }

    void Player:: update(float deltaTime) {
        // Perform any updates, if necessary
        // For example, handle player input, apply damping forces, etc.
        playerRigidBody->applyCentralForce(-playerRigidBody->getLinearVelocity() * dampingFactor);
    }

    glm::vec3 Player::getPlayerPosition() const {
        btTransform trans;
        trans = playerRigidBody->getWorldTransform();
        btVector3 pos = trans.getOrigin();
        return glm::vec3(pos.x(), pos.y(), pos.z());
    }

    // Getter for the player's rigid body
    btRigidBody* Player:: getPlayerRigidBody() const {
        return playerRigidBody;
    }
