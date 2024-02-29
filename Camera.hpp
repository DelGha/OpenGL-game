#ifndef Camera_hpp
#define Camera_hpp

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

namespace gps {
    
    enum MOVE_DIRECTION {MOVE_FORWARD, MOVE_BACKWARD, MOVE_RIGHT, MOVE_LEFT};
    
    class Camera {

    public:
        //Camera constructor
        Camera(glm::vec3 cameraPosition, glm::vec3 cameraTarget, glm::vec3 cameraUp);
        //Camera constructor with front direction implemented
        Camera(glm::vec3 cameraPosition, glm::vec3 cameraTarget, glm::vec3 cameraUp, glm::vec3 cameraFrontDirection);
        //Camera constructor with front direction and right(left) direction implemented
        Camera(glm::vec3 cameraPosition, glm::vec3 cameraTarget, glm::vec3 cameraUp, glm::vec3 cameraFrontDirection, glm::vec3 cameraRightDirection);
        Camera(glm::vec3 cameraPosition, glm::vec3 cameraTarget, glm::vec3 cameraUp, glm::vec3 cameraFrontDirection, glm::vec3 cameraRightDirection, glm::vec3 cameraAlwaysFrontDirection);
        //return the view matrix, using the glm::lookAt() function
        glm::mat4 getViewMatrix();
        //update the camera internal parameters following a camera move event
        void move(MOVE_DIRECTION direction, float speed);
        //update the camera internal parameters following a camera rotate event
        //yaw - camera rotation around the y axis
        //pitch - camera rotation around the x axis
        void rotate(float pitch, float yaw);
        glm::vec3 getCameraTarget();
        glm::vec3 getCameraPosition();
        glm::vec3 getCameraFrontDirection();
        void setCameraPosition(glm::vec3 cameraPosition);
        glm::vec3 getCameraRightDirection();
        glm::vec3 getCameraAlwaysFrontDirection();
    private:
        glm::vec3 cameraPosition;
        glm::vec3 cameraTarget;
        glm::vec3 cameraFrontDirection;
        glm::vec3 cameraRightDirection;
        glm::vec3 cameraUpDirection;
        glm::vec3 cameraAlwaysFrontDirection;
    };    
}

#endif /* Camera_hpp */
