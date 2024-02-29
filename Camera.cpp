#include "Camera.hpp"
#include <iostream>

using namespace std;

namespace gps {

    //Camera constructor
    Camera::Camera(glm::vec3 cameraPosition, glm::vec3 cameraTarget, glm::vec3 cameraUp, glm::vec3 cameraFrontDirection, glm::vec3 cameraRightDirection, glm::vec3 cameraAlwaysFrontDirection) {
        this->cameraPosition = cameraPosition;
        this->cameraTarget = cameraTarget;
        this->cameraUpDirection = cameraUp;
        this->cameraFrontDirection = cameraFrontDirection;
        this->cameraRightDirection = cameraRightDirection;
        this->cameraAlwaysFrontDirection = cameraAlwaysFrontDirection;
        //std::cout << "nu mere";
        
    }

    //return the view matrix, using the glm::lookAt() function
    glm::mat4 Camera::getViewMatrix() {

        //glm::mat4 view = glm::lookAt(cameraPosition, cameraTarget, cameraUpDirection);
        glm::mat4 view = glm::lookAt(cameraPosition, cameraPosition+cameraFrontDirection, cameraUpDirection);

        return view;
    }

    glm::vec3 Camera::getCameraTarget()
    {
        return cameraTarget;
    }

    glm::vec3 Camera::getCameraPosition()
    {
        return cameraPosition;
    }

    void Camera::setCameraPosition(glm::vec3 cameraPosition)
    {
        this->cameraPosition = cameraPosition;
    }

    glm::vec3 Camera::getCameraFrontDirection()
    {
        return cameraFrontDirection;
    }

    glm::vec3 Camera::getCameraRightDirection()
    {
        return cameraRightDirection;
    }

    glm::vec3 Camera::getCameraAlwaysFrontDirection()
    {
        return cameraAlwaysFrontDirection;
    }

    //update the camera internal parameters following a camera move event
    void Camera::move(MOVE_DIRECTION direction, float speed) {

        glm::mat4 translate = glm::mat4(1.0f);
        glm::vec3 cameraRight = glm::normalize(glm::cross(cameraFrontDirection, cameraUpDirection));
        glm::vec3 horizontalFront = glm::normalize(glm::vec3(cameraFrontDirection.x, 0.0f, cameraFrontDirection.z));
        
        switch (direction)
        {
        case gps::MOVE_FORWARD:
            //translate = glm::translate(translate, glm::vec3(0, 0, -1) * speed);
            cameraPosition += horizontalFront * speed;
            break;
        case gps::MOVE_BACKWARD:
            //translate = glm::translate(translate, glm::vec3(0, 0, 1) * speed);
            cameraPosition -= horizontalFront * speed;
            break;
        case gps::MOVE_RIGHT:
            //translate = glm::translate(translate, glm::vec3(1, 0, 0) * speed);
            cameraPosition += cameraRight * speed;
            break;
        case gps::MOVE_LEFT:
            //translate = glm::translate(translate, glm::vec3(-1, 0, 0) * speed);
            cameraPosition -= cameraRight * speed;
            break;
        default:
            break;
        }

        cameraPosition = glm::vec3(translate * glm::vec4(cameraPosition, 1.0f));

    }

    //update the camera internal parameters following a camera rotate event
    //yaw - camera rotation around the y axis
    //pitch - camera rotation around the x axis
    void Camera::rotate(float pitch, float yaw) {

        glm::vec3 front;

        front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        front.y = sin(glm::radians(pitch));
        front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));

        cameraFrontDirection = glm::normalize(front);
        cameraRightDirection = glm::normalize(glm::cross(cameraFrontDirection, cameraUpDirection));

        cameraAlwaysFrontDirection.x = cos(glm::radians(yaw));
        cameraAlwaysFrontDirection.y = 0.0f; // No change in the y-axis for forward direction
        cameraAlwaysFrontDirection.z = sin(glm::radians(yaw));
        cameraAlwaysFrontDirection = glm::normalize(cameraAlwaysFrontDirection);

    }
}
