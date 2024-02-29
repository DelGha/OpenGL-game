#if defined (__APPLE__)
    #define GLFW_INCLUDE_GLCOREARB
    #define GL_SILENCE_DEPRECATION
#else
    #define GLEW_STATIC
    #include <GL/glew.h>
#endif

#include <GLFW/glfw3.h>

#include <glm/glm.hpp> //core glm functionality
#include <glm/gtc/matrix_transform.hpp> //glm extension for generating common transformation matrices
#include <glm/gtc/matrix_inverse.hpp> //glm extension for computing inverse matrices
#include <glm/gtc/type_ptr.hpp> //glm extension for accessing the internal data structure of glm types

#include "Window.h"
#include "Shader.hpp"
#include "Camera.hpp"
#include "Model3D.hpp"
#include "SkyBox.hpp"
#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"

#include "Player.h"

#include <iostream>

using namespace std;

// window
gps::Window myWindow;

// shadow matrices
GLuint lightSpaceMatrixLoc;
GLuint lightSpaceMatrixLoc2;
GLuint shadowModelLoc;

//float near_plane = 35.0f, far_plane = 200.0f;
float SHADOW_WIDTH = 8192.0f, SHADOW_HEIGHT = 8192.0f;
//shadow stuff
GLuint shadowMapFBO;
GLuint depthMapTexture;
gps::Shader depthMapShader;
bool depthPass = false;
glm::mat4 lightSpaceMatrix;

// matrices
glm::mat4 model;
glm::mat4 view;
glm::mat4 skyBoxView;   //view matrix for skybox
glm::mat4 projection;
glm::mat4 skyBoxProjection; // projection matrix for skybox
glm::mat3 normalMatrix;

glm::mat4 pedestalModelMatrix;
glm::mat4 hammerModelMatrix;
glm::mat4 generatedShipMatrix;

// light parameters
glm::vec3 lightDir;
glm::vec3 lightPos;
glm::vec3 lightColor;

// shader uniform locations
GLint modelLoc;
GLint viewLoc;
GLint skyBoxViewLoc; //addition for skybox shader
GLint projectionLoc;
GLint skyBoxProjectionLoc; // addition for skybox shader
GLint normalMatrixLoc;
GLint lightDirLoc;
GLint lightPosLoc;
GLint lightColorLoc;

// camera
// Initial values of the camera

gps::Camera myCamera(
    //glm::vec3(0.0f, 0.0f, 3.0f),   // cameraPosition
    glm::vec3(0.0f, 5.0f, 3.0f),
    glm::vec3(0.0f, 0.0f, -10.0f), // cameraTarget
    glm::vec3(0.0f, 1.0f, 0.0f),   // cameraUp
    glm::vec3(0.0f, 0.0f, -1.0f),  // cameraFrontDirection
    glm::vec3(1.0f, 0.0f, 0.0f),   // cameraRightDirection
    glm::vec3(0.0f, 0.0f, -1.0f)); // cameraAlwaysFrontDirection

GLfloat cameraSpeed = 0.05f;

GLboolean pressedKeys[1024];

// models
gps::Model3D teapot;
gps::Model3D cubeFloor;
gps::Model3D house;
gps::Model3D house2;
gps::Model3D house3;
gps::Model3D helmet;
gps::Model3D pedestal;
gps::Model3D hammer;
gps::Model3D soldier;
gps::Model3D fence1;
gps::Model3D fence2;
gps::Model3D fence3;
gps::Model3D fence4;
gps::Model3D ship;
gps::Model3D generatedShip;
float shipX = 0;
float shipY = 50;
float originalZ = 100;
float shipZ = 150;
float desiredShipZ = -150;
float deltaTime;


float pedestalY = -0.5;
float hammerY = 4;
float hammerRotate = 0;

GLfloat angle;

// shaders
gps::Shader myBasicShader;

float yaw = -90.0f;
float pitch = 0.0f;
float fov = 45.0f;

bool firstMouse = true;
float lastX = 800.0 / 2.0;
float lastY = 600.0 / 2.0;

//std::vector<std::string> faces
//{
//    "skybox/right.tga",
//    "skybox/left.tga",
//    "skybox/top.tga",
//    "skybox/bottom.tga",
//    "skybox/front.tga",
//    "skybox/back.tga"
//};

gps::SkyBox mySkyBox;
gps::Shader skyboxShader;

bool wireframeView = false;
bool flatView = false;

float animationDuration = 5.0f;  // Duration of the animation in seconds
float currentAnimationTime = 0.0f;

// Initial and final camera positions
glm::vec3 initialCameraPosition(0.0f, 5.0f, 3.0f);  // Facing downwards
float desiredX = 0.0f;
float desiredY = 0.65f;
float desiredZ = 3.0f;
glm::vec3 desiredCameraPosition(desiredX, desiredY, desiredZ); // Your desired original position

// Blinking variables
float blinkDuration = 0.5f; // Duration for each blink
float blinkTimer = 0.0f;
bool blink = false;

btDiscreteDynamicsWorld* dynamicsWorld;
btSequentialImpulseConstraintSolver* solver;
btCollisionDispatcher* dispatcher;
btDefaultCollisionConfiguration* collisionConfiguration;
btBroadphaseInterface* broadphase;
btCollisionShape* boxCollisionShape_Floor;
btCollisionShape* boxCollisionShape_Lampas;
btCollisionShape* boxCollisionShape_House;
btCollisionShape* boxCollisionShape_House2;
btCollisionShape* boxCollisionShape_House3;
btCollisionShape* boxCollisionShape_Joe;
btCollisionShape* boxCollisionShape_Pedestal;
btCollisionShape* boxCollisionShape_Ship;
btCollisionShape* boxCollisionShape_WallFront;
btCollisionShape* boxCollisionShape_WallBack;
btCollisionShape* boxCollisionShape_WallLeft;
btCollisionShape* boxCollisionShape_WallRight;

Player *player;


GLenum glCheckError_(const char *file, int line)
{
	GLenum errorCode;
	while ((errorCode = glGetError()) != GL_NO_ERROR) {
		std::string error;
		switch (errorCode) {
            case GL_INVALID_ENUM:
                error = "INVALID_ENUM";
                break;
            case GL_INVALID_VALUE:
                error = "INVALID_VALUE";
                break;
            case GL_INVALID_OPERATION:
                error = "INVALID_OPERATION";
                break;
            case GL_OUT_OF_MEMORY:
                error = "OUT_OF_MEMORY";
                break;
            case GL_INVALID_FRAMEBUFFER_OPERATION:
                error = "INVALID_FRAMEBUFFER_OPERATION";
                break;
        }
		std::cout << error << " | " << file << " (" << line << ")" << std::endl;
	}
	return errorCode;
}
#define glCheckError() glCheckError_(__FILE__, __LINE__)

void initSkybox() {
    std::vector<const GLchar*> faces;
    faces.push_back("skybox/right.tga");
    faces.push_back("skybox/left.tga");
    faces.push_back("skybox/top.tga");
    faces.push_back("skybox/bottom.tga");
    faces.push_back("skybox/back.tga");
    faces.push_back("skybox/front.tga");

    mySkyBox.Load(faces);
}

void windowResizeCallback(GLFWwindow* window, int width, int height) {
	fprintf(stdout, "Window resized! New width: %d , and height: %d\n", width, height);
	//TODO
}

void keyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mode) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }

	if (key >= 0 && key < 1024) {
        if (action == GLFW_PRESS) {
            pressedKeys[key] = true;
        } else if (action == GLFW_RELEASE) {
            pressedKeys[key] = false;
        }
    }
}

void mouseCallback(GLFWwindow* window, double xpos, double ypos) {

    if (firstMouse) {
        lastX = (float)xpos;
        lastY = (float)ypos;
        firstMouse = false;
    }

    float xoffset = (float)xpos - lastX;
    float yoffset = lastY - (float)ypos;

    lastX = (float)xpos;
    lastY = (float)ypos;

    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    if (pitch > 89.0f) {
        pitch = 89.0f;
    }
    if (pitch < -89.0f) {
        pitch = -89.0f;
    }

    myCamera.rotate(pitch, yaw);

    view = myCamera.getViewMatrix();
    myBasicShader.useShaderProgram();
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    // compute normal matrix for teapot
    normalMatrix = glm::mat3(glm::inverseTranspose(view * model));

}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    if (fov >= 1.0f && fov <= 45.0f) {
        fov -= (float)yoffset;
    }
    if (fov <= 1.0f) {
        fov = 1.0f;
    }

    if (fov >= 45.0f) {
        fov = 45.0f;
    }
}

void processMovement() {

    glm::vec3 combinedDirection = glm::vec3(0.0f);

	if (pressedKeys[GLFW_KEY_W]) {
		//myCamera.move(gps::MOVE_FORWARD, cameraSpeed);
        //player->move(glm::normalize(myCamera.getCameraAlwaysFrontDirection() + myCamera.getCameraRightDirection()), 15.0f);
        combinedDirection += myCamera.getCameraAlwaysFrontDirection();
		//update view matrix
        view = myCamera.getViewMatrix();
        myBasicShader.useShaderProgram();
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        // compute normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view*model));

	}

	if (pressedKeys[GLFW_KEY_S]) {
		//myCamera.move(gps::MOVE_BACKWARD, cameraSpeed);
        //player->move(-(myCamera.getCameraAlwaysFrontDirection()), 15.0f);
        combinedDirection -= myCamera.getCameraAlwaysFrontDirection();
        //update view matrix
        view = myCamera.getViewMatrix();
        myBasicShader.useShaderProgram();
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        // compute normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view*model));
	}

	if (pressedKeys[GLFW_KEY_A]) {
		//myCamera.move(gps::MOVE_LEFT, cameraSpeed);
        //player->move(-(myCamera.getCameraRightDirection()), 15.0f);
        combinedDirection -= myCamera.getCameraRightDirection();
        //update view matrix
        view = myCamera.getViewMatrix();
        myBasicShader.useShaderProgram();
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        // compute normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view*model));
	}

	if (pressedKeys[GLFW_KEY_D]) {
		//myCamera.move(gps::MOVE_RIGHT, cameraSpeed);
        //player->move((myCamera.getCameraRightDirection()), 15.0f);
        combinedDirection += myCamera.getCameraRightDirection();
        //update view matrix
        view = myCamera.getViewMatrix();
        myBasicShader.useShaderProgram();
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        // compute normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view*model));
	}

    if (pressedKeys[GLFW_KEY_Q]) {
        angle -= 1.0f;
        // update model matrix for teapot
        model = glm::rotate(glm::mat4(1.0f), glm::radians(angle), glm::vec3(0, 1, 0));
        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view*model));
    }

    if (pressedKeys[GLFW_KEY_E]) {
        angle += 1.0f;
        // update model matrix for teapot
        model = glm::rotate(glm::mat4(1.0f), glm::radians(angle), glm::vec3(0, 1, 0));
        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view*model));
    }

    if (pressedKeys[GLFW_KEY_T] && !wireframeView) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        wireframeView = true;
    }

    if (pressedKeys[GLFW_KEY_G] && wireframeView) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        wireframeView = false;   
    }

    if (pressedKeys[GLFW_KEY_Y]) {
        //std::cout << "am intrat in Y";
        //glShadeModel(GL_FLAT);
        flatView = false; // Toggle the flatView boolean
        myBasicShader.useShaderProgram();
        glUniform1i(glGetUniformLocation(myBasicShader.shaderProgram, "flatShading"), flatView);
    }

    if (pressedKeys[GLFW_KEY_H]) {
        //std::cout << "am intrat in U";
        //glShadeModel(GL_SMOOTH);
        flatView = true; // Toggle the flatView boolean
        myBasicShader.useShaderProgram();
        glUniform1i(glGetUniformLocation(myBasicShader.shaderProgram, "flatShading"), flatView);
    }

    // Normalize the combined direction vector (optional)
    if (glm::length(combinedDirection) > 0.0f) {
        combinedDirection = glm::normalize(combinedDirection);
    }

    // Apply the movement only if there's a valid direction
    if (glm::length(combinedDirection) > 0.0f) {
        player->move(combinedDirection, deltaTime * 1500.0f);
    }
}

void initOpenGLWindow() {
    myWindow.Create(1024, 768, "OpenGL Project Core");
}

void setWindowCallbacks() {
    glfwSetInputMode(myWindow.getWindow(), GLFW_CURSOR, GLFW_CURSOR_DISABLED); // call to disable the mouse cursor inside the application
	glfwSetWindowSizeCallback(myWindow.getWindow(), windowResizeCallback);
    glfwSetKeyCallback(myWindow.getWindow(), keyboardCallback);
    glfwSetCursorPosCallback(myWindow.getWindow(), mouseCallback);
    glfwSetScrollCallback(myWindow.getWindow(), scrollCallback);
}

void initOpenGLState() {
	glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
	glViewport(0, 0, myWindow.getWindowDimensions().width, myWindow.getWindowDimensions().height);
    glEnable(GL_FRAMEBUFFER_SRGB);
	glEnable(GL_DEPTH_TEST); // enable depth-testing
	glDepthFunc(GL_LESS); // depth-testing interprets a smaller value as "closer"
	glEnable(GL_CULL_FACE); // cull face
	glCullFace(GL_BACK); // cull back face
	glFrontFace(GL_CCW); // GL_CCW for counter clock-wise
}

void initModels() {
    teapot.LoadModel("models/teapot/teapot20segUT.obj");
    cubeFloor.LoadModel("models/ground/ground.obj");
    house.LoadModel("models/house/House_Lil.obj");
    house2.LoadModel("models/house/House_Lil.obj");
    house3.LoadModel("models/house/House_Lil.obj");
    helmet.LoadModel("models/helmet/KasrkinHelmet.obj");
    pedestal.LoadModel("models/pedestal/pedestal2.obj");
    hammer.LoadModel("models/hammer/Warhammer.obj");
    soldier.LoadModel("models/realSoldier/soldier.obj");
    fence1.LoadModel("models/fence/fence.obj");
    fence2.LoadModel("models/fence/fence.obj");
    fence3.LoadModel("models/fence/fence.obj");
    fence4.LoadModel("models/fence/fence.obj");
    ship.LoadModel("models/ship/Valkyrie.obj");
    generatedShip.LoadModel("models/ship/Valkyrie.obj");
}

void initShaders() {
	myBasicShader.loadShader(
        "shaders/basic.vert",
        "shaders/basic.frag");

    skyboxShader.loadShader(
        "shaders/skyboxShader.vert", 
        "shaders/skyboxShader.frag");

    depthMapShader.loadShader(
        "shaders/depthMapShader.vert", 
        "shaders/depthMapShader.frag");
    depthMapShader.useShaderProgram();
}

void initShadowMap()
{
    //generate FBO ID
    glGenFramebuffers(1, &shadowMapFBO);

    //create depth texture for FBO
    glGenTextures(1, &depthMapTexture);
    glBindTexture(GL_TEXTURE_2D, depthMapTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
        SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    //attach texture to FBO
    glBindFramebuffer(GL_FRAMEBUFFER, shadowMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMapTexture, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void initCollisionBoxes() {

    broadphase = new btDbvtBroadphase();

    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    solver = new btSequentialImpulseConstraintSolver;

    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, -9.81, 0)); // suspect

    boxCollisionShape_Floor= new btBoxShape(btVector3(100.0f, 0.1f, 100.0f));

    btTransform transform;
    transform.setIdentity();

    transform.setOrigin(btVector3(0.0f, -1.0f, 0.0f));

    btDefaultMotionState* motionstate = new btDefaultMotionState(transform);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate,
        boxCollisionShape_Floor,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);

    dynamicsWorld->addRigidBody(rigidBody);

    // --

    glm::vec3 lampasBoundingBoxSize = teapot.calculateOverallBoundingBoxSize();

    boxCollisionShape_Lampas = new btBoxShape(btVector3(
        lampasBoundingBoxSize.x * 0.5, // Width
        lampasBoundingBoxSize.y * 0.5, // Height
        lampasBoundingBoxSize.z * 0.5  // Depth
    ));

    btTransform transform2;
    transform2.setIdentity();

    transform2.setOrigin(btVector3(-19.0f, -0.5f, 20.0f));

    btDefaultMotionState* motionstate2 = new btDefaultMotionState(transform2);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI2(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate2,
        boxCollisionShape_Lampas,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody2 = new btRigidBody(rigidBodyCI2);

    dynamicsWorld->addRigidBody(rigidBody2);

    // --

    glm::vec3 house1BoundingBoxSize = house.calculateOverallBoundingBoxSize();

    boxCollisionShape_House = new btBoxShape(btVector3(
        house1BoundingBoxSize.x * 0.5, // Width
        house1BoundingBoxSize.y * 0.5, // Height
        house1BoundingBoxSize.z * 0.5  // Depth
    ));

    btTransform transform3;
    transform3.setIdentity();

    transform3.setOrigin(btVector3(0.0f, -1.0f, -20.0f));
    transform3.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians(-30.0f)));

    btDefaultMotionState* motionstate3 = new btDefaultMotionState(transform3);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI3(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate3,
        boxCollisionShape_House,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody3 = new btRigidBody(rigidBodyCI3);

    dynamicsWorld->addRigidBody(rigidBody3);

    // --
    glm::vec3 house2BoundingBoxSize = house2.calculateOverallBoundingBoxSize();

    boxCollisionShape_House2 = new btBoxShape(btVector3(
        house2BoundingBoxSize.x * 0.5, // Width
        house2BoundingBoxSize.y * 0.5, // Height
        house2BoundingBoxSize.z * 0.5  // Depth
    ));

    btTransform transform4;
    transform4.setIdentity();

    transform4.setOrigin(btVector3(7.0f, -1.0f, -17.0f));
    transform4.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians(-20.0f)));

    btDefaultMotionState* motionstate4 = new btDefaultMotionState(transform4);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI4(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate4,
        boxCollisionShape_House2,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody4 = new btRigidBody(rigidBodyCI4);

    dynamicsWorld->addRigidBody(rigidBody4);

    //--
    glm::vec3 house3BoundingBoxSize = house3.calculateOverallBoundingBoxSize();

    boxCollisionShape_House3 = new btBoxShape(btVector3(
        house3BoundingBoxSize.x * 0.5, // Width
        house3BoundingBoxSize.y * 0.5, // Height
        house3BoundingBoxSize.z * 0.5  // Depth
    ));

    btTransform transform5;
    transform5.setIdentity();

    transform5.setOrigin(btVector3(-17.0f, -1.0f, 17.0f));
    transform5.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians(-220.0f)));

    btDefaultMotionState* motionstate5 = new btDefaultMotionState(transform5);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI5(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate5,
        boxCollisionShape_House3,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody5 = new btRigidBody(rigidBodyCI5);

    dynamicsWorld->addRigidBody(rigidBody5);

    //--

    glm::vec3 JoeBoundingBoxSize = soldier.calculateOverallBoundingBoxSize();

    boxCollisionShape_Joe = new btBoxShape(btVector3(
        JoeBoundingBoxSize.x * 0.5, // Width
        JoeBoundingBoxSize.y * 0.5, // Height
        JoeBoundingBoxSize.z * 0.5  // Depth
    ));

    btTransform transform6;
    transform6.setIdentity();

    transform6.setOrigin(btVector3(12.0f, -1.0f, -2.0f));
    transform6.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians((140.0f))));

    btDefaultMotionState* motionstate6 = new btDefaultMotionState(transform6);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI6(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate6,
        boxCollisionShape_Joe,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody6 = new btRigidBody(rigidBodyCI6);

    dynamicsWorld->addRigidBody(rigidBody6);

    //--
    glm::vec3 PedestalBoundingBoxSize = pedestal.calculateOverallBoundingBoxSize();

    boxCollisionShape_Pedestal = new btBoxShape(btVector3(
        PedestalBoundingBoxSize.x / 150 * 0.5, // Width
        PedestalBoundingBoxSize.y / 150 * 0.5, // Height
        PedestalBoundingBoxSize.z / 150 * 0.5  // Depth
    ));

    btTransform transform7;
    transform7.setIdentity();

    transform7.setOrigin(btVector3(15.0f, pedestalY, 0.0f));

    btDefaultMotionState* motionstate7 = new btDefaultMotionState(transform7);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI7(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate7,
        boxCollisionShape_Pedestal,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody7 = new btRigidBody(rigidBodyCI7);

    dynamicsWorld->addRigidBody(rigidBody7);
    //--
    glm::vec3 TaxiBoundingBoxSize = ship.calculateOverallBoundingBoxSize();

    boxCollisionShape_Ship = new btBoxShape(btVector3(
        TaxiBoundingBoxSize.x / 100 * 0.5, // Width
        TaxiBoundingBoxSize.y / 100 * 0.5, // Height
        TaxiBoundingBoxSize.z / 100 * 0.5  // Depth
    ));

    btTransform transform8;
    transform8.setIdentity();

    transform8.setOrigin(btVector3(-13.0f, -1.0f, -13.0f));
    transform8.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians((60.0f))));

    btDefaultMotionState* motionstate8 = new btDefaultMotionState(transform8);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI8(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate8,
        boxCollisionShape_Ship,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody8 = new btRigidBody(rigidBodyCI8);

    dynamicsWorld->addRigidBody(rigidBody8);
    //--
    glm::vec3 LeftWallBoundingBoxSize = fence1.calculateOverallBoundingBoxSize();

    boxCollisionShape_WallLeft = new btBoxShape(btVector3(
        LeftWallBoundingBoxSize.x * 0.5, // Width
        LeftWallBoundingBoxSize.y * 0.5, // Height
        LeftWallBoundingBoxSize.z / 2.2 * 0.5  // Depth
    ));

    btTransform transform9;
    transform9.setIdentity();

    transform9.setOrigin(btVector3(-30.0f, -2.0f, 0.0f));
    transform9.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians((-90.0f))));

    btDefaultMotionState* motionstate9 = new btDefaultMotionState(transform9);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI9(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate9,
        boxCollisionShape_WallLeft,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody9 = new btRigidBody(rigidBodyCI9);

    dynamicsWorld->addRigidBody(rigidBody9);
    //--
    glm::vec3 FrontWallBoundingBoxSize = fence2.calculateOverallBoundingBoxSize();

    boxCollisionShape_WallFront = new btBoxShape(btVector3(
        FrontWallBoundingBoxSize.x * 0.5, // Width
        FrontWallBoundingBoxSize.y * 0.5, // Height
        FrontWallBoundingBoxSize.z / 2.2 * 0.5  // Depth
    ));

    btTransform transform10;
    transform10.setIdentity();

    transform10.setOrigin(btVector3(0.0f, -2.0f, -30.0f));
    //transform9.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians((-90.0f))));

    btDefaultMotionState* motionstate10 = new btDefaultMotionState(transform10);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI10(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate10,
        boxCollisionShape_WallFront,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody10 = new btRigidBody(rigidBodyCI10);

    dynamicsWorld->addRigidBody(rigidBody10);
    //--
    glm::vec3 RightWallBoundingBoxSize = fence3.calculateOverallBoundingBoxSize();

    boxCollisionShape_WallRight = new btBoxShape(btVector3(
        RightWallBoundingBoxSize.x * 0.5, // Width
        RightWallBoundingBoxSize.y * 0.5, // Height
        RightWallBoundingBoxSize.z / 2.2 * 0.5  // Depth
    ));

    btTransform transform11;
    transform11.setIdentity();

    transform11.setOrigin(btVector3(30.0f, -2.0f, 0.0f));
    transform11.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians((-90.0f))));

    btDefaultMotionState* motionstate11 = new btDefaultMotionState(transform11);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI11(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate11,
        boxCollisionShape_WallRight,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody11 = new btRigidBody(rigidBodyCI11);

    dynamicsWorld->addRigidBody(rigidBody11);
    //--
    glm::vec3 BackWallBoundingBoxSize = fence4.calculateOverallBoundingBoxSize();

    boxCollisionShape_WallBack = new btBoxShape(btVector3(
        BackWallBoundingBoxSize.x * 0.5, // Width
        BackWallBoundingBoxSize.y * 0.5, // Height
        BackWallBoundingBoxSize.z / 2.2 * 0.5  // Depth
    ));

    btTransform transform12;
    transform12.setIdentity();

    transform12.setOrigin(btVector3(0.0f, -2.0f, 30.0f));
    //transform9.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians((-90.0f))));

    btDefaultMotionState* motionstate12 = new btDefaultMotionState(transform12);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI12(
        0,                  // mass, in kg. 0 -> Static object, will never move.
        motionstate12,
        boxCollisionShape_WallBack,  // collision shape of body
        btVector3(0, 0, 0)    // local inertia
    );

    btRigidBody* rigidBody12 = new btRigidBody(rigidBodyCI12);

    dynamicsWorld->addRigidBody(rigidBody12);
    //--
    btVector3 startPosition(desiredX, desiredY, desiredZ);
    player = new Player(dynamicsWorld, startPosition);
    
}

glm::mat4 computeLightSpaceTrMatrix() {
    glm::mat4 lightView = glm::lookAt(glm::vec3(0, 15, -40), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    //glm::mat4 lightView = auxiliaryCamera.getViewMatrix();
    const GLfloat near_plane = 0.1f, far_plane = 200.0f;
    glm::mat4 lightProjection = glm::ortho(-100.0f, 100.0f, -100.0f, 100.0f, near_plane, far_plane);
    glm::mat4 lightSpaceTrMatrix = lightProjection * lightView;


    return lightSpaceTrMatrix;
}

void initUniforms() {
	myBasicShader.useShaderProgram();
    
    model = glm::mat4(1.0f);
    modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

	// get view matrix for current camera
	view = myCamera.getViewMatrix();
	viewLoc = glGetUniformLocation(myBasicShader.shaderProgram, "view");
	// send view matrix to shader
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

    normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
    normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");
    glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));

    // create projection matrix
    projection = glm::perspective(glm::radians(fov),
        (float)myWindow.getWindowDimensions().width / (float)myWindow.getWindowDimensions().height,
        0.1f, 100.0f);
    projectionLoc = glGetUniformLocation(myBasicShader.shaderProgram, "projection");
    // send projection matrix to shader
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));

	//set the light direction (direction towards the light) (Directional Light)
	lightDir = glm::vec3(0.0f, 15.0f, -1.0f);
	lightDirLoc = glGetUniformLocation(myBasicShader.shaderProgram, "lightDir");
	// send light dir to shader
	glUniform3fv(lightDirLoc, 1, glm::value_ptr(lightDir));
    glCheckError();

    //set the position of the light (Point Light)
    lightPos = glm::vec3(15.0f, 7.0f, 0.0f);
    lightPosLoc = glGetUniformLocation(myBasicShader.shaderProgram, "lightPos");
    // send light pos to shader
    glUniform3fv(lightPosLoc, 1, glm::value_ptr(lightPos));

	//set light color
	lightColor = glm::vec3(1.0f, 1.0f, 1.0f); //white light
	lightColorLoc = glGetUniformLocation(myBasicShader.shaderProgram, "lightColor");
	// send light color to shader
	glUniform3fv(lightColorLoc, 1, glm::value_ptr(lightColor));

    lightSpaceMatrixLoc2 = glGetUniformLocation(myBasicShader.shaderProgram, "fragPosLightSpace");
    glCheckError();
    depthMapShader.useShaderProgram();
    glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    lightSpaceMatrixLoc = glGetUniformLocation(depthMapShader.shaderProgram, "fragPosLightSpace");
    glCheckError();
    shadowModelLoc = glGetUniformLocation(depthMapShader.shaderProgram, "model");
    glCheckError();

    skyboxShader.useShaderProgram();
    glCheckError();

    //Skybox view matrix
    skyBoxView = myCamera.getViewMatrix();
    glCheckError();
    skyBoxViewLoc = glGetUniformLocation(skyboxShader.shaderProgram, "skyBoxView");
    glCheckError();
    glUniformMatrix4fv(skyBoxViewLoc, 1, GL_FALSE, glm::value_ptr(skyBoxView));
    glCheckError();

    //Skybox projection matrix
    skyBoxProjection = glm::perspective(glm::radians(fov),
        (float)myWindow.getWindowDimensions().width / (float)myWindow.getWindowDimensions().height,
        0.1f, 100.0f);
    glCheckError();
    skyBoxProjectionLoc = glGetUniformLocation(skyboxShader.shaderProgram, "skyBoxProjection");
    glCheckError();
    glUniformMatrix4fv(skyBoxProjectionLoc, 1, GL_FALSE, glm::value_ptr(skyBoxProjection));
    glCheckError();

    initSkybox();

}

void renderTeapot(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();
        glCheckError();
        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(-19.0f, -0.5f, 20.0f));
        glCheckError();
        // update model matrix for teapot
        model = glm::rotate(model, glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f));
        glCheckError();
        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");
        glCheckError();

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        glCheckError();
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");
        glCheckError();

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        //glCheckError();

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));
        glCheckError();

        // draw teapot
        teapot.Draw(shader);
        glCheckError();
    }
    else {
        // select active shader program
        //std::cout << "aici am intrat bine";
        depthMapShader.useShaderProgram();
        glCheckError();

        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(-19.0f, -0.5f, 20.0f));
        model = glm::rotate(model, glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f));
        glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        // draw teapot
        teapot.Draw(depthMapShader);
        glCheckError();
    }
}

void renderSkyBox(gps::Shader shader) {
    // select active shader program
    shader.useShaderProgram();

    // Get the camera's view matrix without translation
    glm::mat4 viewWithoutTranslation = glm::mat4(glm::mat3(myCamera.getViewMatrix()));
    glUniformMatrix4fv(skyBoxViewLoc, 1, GL_FALSE, glm::value_ptr(viewWithoutTranslation));

    // Use the camera's projection matrix
    glUniformMatrix4fv(skyBoxProjectionLoc, 1, GL_FALSE, glm::value_ptr(skyBoxProjection));

    //draw skybox
    mySkyBox.Draw(skyboxShader, viewWithoutTranslation, skyBoxProjection);
}

void renderCubeFloor(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        shader.useShaderProgram();

        model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -1.0f, 0.0f));
        model = glm::scale(model, glm::vec3(3.0f));
        glUniformMatrix4fv(glGetUniformLocation(shader.shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));

        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));

        cubeFloor.Draw(shader);
    }
    else {
        depthMapShader.useShaderProgram();

        model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -1.0f, 0.0f));
        model = glm::scale(model, glm::vec3(3.0f));
        glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));

        cubeFloor.Draw(shader);
    }
}

void renderHouse(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();

        model = glm::mat4(1.0f); // Initialize as identity matrix
        model = glm::translate(model, glm::vec3(0.0f, -1.0f, -20.0f)); // Translate first
        model = glm::rotate(model, glm::radians(-30.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate

        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));

        house.Draw(shader);
    }
    else {
        depthMapShader.useShaderProgram();


        model = glm::mat4(1.0f); // Initialize as identity matrix
        model = glm::translate(model, glm::vec3(0.0f, -1.0f, -20.0f)); // Translate first
        model = glm::rotate(model, glm::radians(-30.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate

        modelLoc = glGetUniformLocation(depthMapShader.shaderProgram, "model");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        house.Draw(shader);
        glCheckError();

    }
}

void renderHouse2(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();

        model = glm::mat4(1.0f); // Initialize as identity matrix
        model = glm::translate(model, glm::vec3(7.0f, -1.0f, -17.0f)); // Translate first
        model = glm::rotate(model, glm::radians(-20.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate

        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));

        house2.Draw(shader);
    }
    else {
        depthMapShader.useShaderProgram();


        model = glm::mat4(1.0f); // Initialize as identity matrix
        model = glm::translate(model, glm::vec3(7.0f, -1.0f, -17.0f)); // Translate first
        model = glm::rotate(model, glm::radians(-20.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate

        modelLoc = glGetUniformLocation(depthMapShader.shaderProgram, "model");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        house2.Draw(shader);
        glCheckError();

    }
}

void renderHouse3(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();

        model = glm::mat4(1.0f); // Initialize as identity matrix
        model = glm::translate(model, glm::vec3(-17.0f, -1.0f, 17.0f)); // Translate first
        model = glm::rotate(model, glm::radians(-220.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate

        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));

        house3.Draw(shader);
    }
    else {
        depthMapShader.useShaderProgram();


        model = glm::mat4(1.0f); // Initialize as identity matrix
        model = glm::translate(model, glm::vec3(-17.0f, -1.0f, 17.0f)); // Translate first
        model = glm::rotate(model, glm::radians(-220.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate

        modelLoc = glGetUniformLocation(depthMapShader.shaderProgram, "model");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        house3.Draw(shader);
        glCheckError();

    }
}

void renderSoldier(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();

        model = glm::mat4(1.0f); // Initialize as identity matrix
        model = glm::translate(model, glm::vec3(12.0f, -1.0f, -2.0f)); // Translate first
        model = glm::rotate(model, glm::radians(140.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        //model = glm::scale(model, glm::vec3(1 / 2.0f, 1 / 2.0f, 1 / 60.0f));

        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));

        soldier.Draw(shader);
    }
    else {
        depthMapShader.useShaderProgram();


        model = glm::mat4(1.0f); // Initialize as identity matrix
        model = glm::translate(model, glm::vec3(12.0f, -1.0f, -2.0f)); // Translate first
        model = glm::rotate(model, glm::radians(140.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        //model = glm::scale(model, glm::vec3(1 / 60.0f, 1 / 60.0f, 1 / 60.0f));

        modelLoc = glGetUniformLocation(depthMapShader.shaderProgram, "model");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        soldier.Draw(shader);
        glCheckError();

    }
}

void renderHelmet(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();
        glCheckError();
        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(4.8f, -1.0f, -15.3f));
        model = glm::rotate(model, glm::radians(-70.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 70.0f, 1 / 70.0f, 1 / 70.0f));
        glCheckError();
        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");
        glCheckError();

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        glCheckError();
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");
        glCheckError();

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        //glCheckError();

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));
        glCheckError();

        // draw teapot
        helmet.Draw(shader);
        glCheckError();
    }
    else {
        // select active shader program
        //std::cout << "aici am intrat bine";
        depthMapShader.useShaderProgram();
        glCheckError();

        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(4.8f, -1.0f, -15.3f));
        model = glm::rotate(model, glm::radians(-70.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 70.0f, 1 / 70.0f, 1 / 70.0f));
        glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        // draw teapot
        helmet.Draw(depthMapShader);
        glCheckError();
    }
}

void renderPedestal(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();

        pedestalModelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(15.0f, pedestalY, 0.0f));
        pedestalModelMatrix = glm::scale(pedestalModelMatrix, glm::vec3(1 / 150.0f, 1 / 150.0f, 1 / 150.0f));
        // Translate first
        //model = glm::rotate(model, glm::radians(-30.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = pedestalModelMatrix;
        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));

        pedestal.Draw(shader);
    }
    else {
        depthMapShader.useShaderProgram();


        pedestalModelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(15.0f, pedestalY, 0.0f));
        pedestalModelMatrix = glm::scale(pedestalModelMatrix, glm::vec3(1 / 150.0f, 1 / 150.0f, 1 / 150.0f));
        // Translate first
        //model = glm::rotate(model, glm::radians(-30.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = pedestalModelMatrix;
        modelLoc = glGetUniformLocation(depthMapShader.shaderProgram, "model");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        pedestal.Draw(shader);
        glCheckError();

    }
}

void renderHammer(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();

        hammerModelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(15.0f, hammerY, 0.0f));
        hammerModelMatrix = glm::rotate(hammerModelMatrix, hammerRotate, glm::vec3(0.0f, 1.0f, 0.0f));
        hammerModelMatrix = glm::scale(hammerModelMatrix, glm::vec3(1 / 270.0f, 1 / 270.0f, 1 / 270.0f));
        // Translate first
        //model = glm::rotate(model, glm::radians(-30.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = hammerModelMatrix;
        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));

        hammer.Draw(shader);
    }
    else {
        depthMapShader.useShaderProgram();


        hammerModelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(15.0f, hammerY, 0.0f));
        hammerModelMatrix = glm::rotate(hammerModelMatrix, hammerRotate, glm::vec3(0.0f, 1.0f, 0.0f));
        hammerModelMatrix = glm::scale(hammerModelMatrix, glm::vec3(1 / 270.0f, 1 / 270.0f, 1 / 270.0f));
        model = hammerModelMatrix;
        modelLoc = glGetUniformLocation(depthMapShader.shaderProgram, "model");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        hammer.Draw(shader);
        glCheckError();

    }
}

void renderFenceFront(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();
        glCheckError();
        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(-30.0f, -2.0f, 0.0f));
        model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 1.3f, 1 / 1.3f, 1 / 1.3f));
        glCheckError();
        // update model matrix for teapot

        glCheckError();
        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");
        glCheckError();

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        glCheckError();
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");
        glCheckError();

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        //glCheckError();

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));
        glCheckError();

        // draw teapot
        fence1.Draw(shader);
        glCheckError();
    }
    else {
        // select active shader program
        //std::cout << "aici am intrat bine";
        depthMapShader.useShaderProgram();
        glCheckError();

        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(-30.0f, -2.0f, 0.0f));
        model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 1.3f, 1 / 1.3f, 1 / 1.3f));
        glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        // draw teapot
        fence1.Draw(depthMapShader);
        glCheckError();
    }
}
void renderFenceLeft(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();
        glCheckError();
        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -2.0f, -30.0f));
        //model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 1.3f, 1 / 1.3f, 1 / 1.3f));
        glCheckError();
        // update model matrix for teapot

        glCheckError();
        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");
        glCheckError();

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        glCheckError();
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");
        glCheckError();

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        //glCheckError();

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));
        glCheckError();

        // draw teapot
        fence2.Draw(shader);
        glCheckError();
    }
    else {
        // select active shader program
        //std::cout << "aici am intrat bine";
        depthMapShader.useShaderProgram();
        glCheckError();

        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -2.0f, -30.0f));
        //model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 1.3f, 1 / 1.3f, 1 / 1.3f));
        glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        // draw teapot
        fence2.Draw(depthMapShader);
        glCheckError();
    }
}
void renderFenceRight(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();
        glCheckError();
        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(30.0f, -2.0f, 0.0f));
        model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 1.3f, 1 / 1.3f, 1 / 1.3f));
        glCheckError();
        // update model matrix for teapot

        glCheckError();
        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");
        glCheckError();

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        glCheckError();
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");
        glCheckError();

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        //glCheckError();

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));
        glCheckError();

        // draw teapot
        fence4.Draw(shader);
        glCheckError();
    }
    else {
        // select active shader program
        //std::cout << "aici am intrat bine";
        depthMapShader.useShaderProgram();
        glCheckError();

        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(30.0f, -2.0f, 0.0f));
        model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 1.3f, 1 / 1.3f, 1 / 1.3f));
        glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        // draw teapot
        fence4.Draw(depthMapShader);
        glCheckError();
    }
}
void renderFenceBack(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();
        glCheckError();
        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -2.0f, 30.0f));
        //model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 1.3f, 1 / 1.3f, 1 / 1.3f));
        glCheckError();
        // update model matrix for teapot

        glCheckError();
        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");
        glCheckError();

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        glCheckError();
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");
        glCheckError();

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        //glCheckError();

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));
        glCheckError();

        // draw teapot
        fence4.Draw(shader);
        glCheckError();
    }
    else {
        // select active shader program
        //std::cout << "aici am intrat bine";
        depthMapShader.useShaderProgram();
        glCheckError();

        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -2.0f, 30.0f));
        //model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 1.3f, 1 / 1.3f, 1 / 1.3f));
        glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        // draw teapot
        fence4.Draw(depthMapShader);
        glCheckError();
    }
}
void renderShip(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();
        glCheckError();
        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(-12.0f, -1.0f, -12.0f));
        model = glm::rotate(model, glm::radians(60.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 100.0f, 1 / 100.0f, 1 / 100.0f));
        glCheckError();
        // update model matrix for teapot

        glCheckError();
        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");
        glCheckError();

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        glCheckError();
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");
        glCheckError();

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        //glCheckError();

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));
        glCheckError();

        // draw teapot
        ship.Draw(shader);
        glCheckError();
    }
    else {
        // select active shader program
        //std::cout << "aici am intrat bine";
        depthMapShader.useShaderProgram();
        glCheckError();

        // create model matrix for teapot
        model = glm::translate(glm::mat4(1.0f), glm::vec3(-12.0f, -1.0f, -12.0f));
        model = glm::rotate(model, glm::radians(60.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        model = glm::scale(model, glm::vec3(1 / 100.0f, 1 / 100.0f, 1 / 100.0f));
        glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        // draw teapot
        ship.Draw(depthMapShader);
        glCheckError();
    }
}
void renderGeneratedShip(gps::Shader shader, bool depthPass) {
    if (!depthPass) {
        // select active shader program
        shader.useShaderProgram();
        glCheckError();
        // create model matrix for teapot
        generatedShipMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(shipX, shipY, shipZ));
        generatedShipMatrix = glm::rotate(generatedShipMatrix, glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        generatedShipMatrix = glm::scale(generatedShipMatrix, glm::vec3(1 / 100.0f, 1 / 100.0f, 1 / 100.0f));
        glCheckError();
        // update model matrix for teapot
        model = generatedShipMatrix;
        glCheckError();
        modelLoc = glGetUniformLocation(myBasicShader.shaderProgram, "model");
        glCheckError();

        // update normal matrix for teapot
        normalMatrix = glm::mat3(glm::inverseTranspose(view * model));
        glCheckError();
        normalMatrixLoc = glGetUniformLocation(myBasicShader.shaderProgram, "normalMatrix");
        glCheckError();

        //send the view matrix to the shader
        //glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        //glCheckError();

        //send teapot model matrix data to shader
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        //send teapot normal matrix data to shader
        glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, glm::value_ptr(normalMatrix));
        glCheckError();

        // draw teapot
        ship.Draw(shader);
        glCheckError();
    }
    else {
        // select active shader program
        //std::cout << "aici am intrat bine";
        depthMapShader.useShaderProgram();
        glCheckError();

        // create model matrix for teapot
        generatedShipMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(shipX, shipY, shipZ));
        generatedShipMatrix = glm::rotate(generatedShipMatrix, glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // Then rotate
        generatedShipMatrix = glm::scale(generatedShipMatrix, glm::vec3(1 / 100.0f, 1 / 100.0f, 1 / 100.0f));
        model = generatedShipMatrix;
        glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glCheckError();

        // draw teapot
        ship.Draw(depthMapShader);
        glCheckError();
    }
}

void renderDepthMap() {

    lightSpaceMatrix = computeLightSpaceTrMatrix();
    // Use the depth shader for rendering depth from light's perspective
    depthMapShader.useShaderProgram();
    glUniformMatrix4fv(glGetUniformLocation(depthMapShader.shaderProgram, "lightSpaceTrMatrix"),
        1,
        GL_FALSE,
        glm::value_ptr(computeLightSpaceTrMatrix()));

    glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowMapFBO);
    glClear(GL_DEPTH_BUFFER_BIT);

    depthPass = true;

    // Render scene from light's perspective
    renderTeapot(depthMapShader, depthPass);
    glCheckError();
    renderHouse(depthMapShader, depthPass);
    renderHouse2(depthMapShader, depthPass);
    renderHouse3(depthMapShader, depthPass);
    renderSoldier(depthMapShader, depthPass);
    glCheckError();
    renderHelmet(depthMapShader, depthPass);
    glCheckError();
    renderPedestal(depthMapShader, depthPass);
    glCheckError();
    renderHammer(depthMapShader, depthPass);
    renderFenceLeft(depthMapShader, depthPass);
    renderFenceFront(depthMapShader, depthPass);
    renderFenceRight(depthMapShader, depthPass);
    renderFenceBack(depthMapShader, depthPass);
    renderShip(depthMapShader, depthPass);
    renderGeneratedShip(depthMapShader, depthPass);
    renderCubeFloor(depthMapShader, depthPass);
    glCheckError();

    depthPass = false;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glCheckError();
    //glViewport(0, 0, myWindow.getWindowDimensions().width, myWindow.getWindowDimensions().height);
    glCheckError();
}

void renderScene() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glCheckError();

    renderDepthMap();
    glCheckError();

    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glCheckError();

    myBasicShader.useShaderProgram();

    //// create projection matrix
    projection = glm::perspective(glm::radians(fov),
        (float)myWindow.getWindowDimensions().width / (float)myWindow.getWindowDimensions().height,
        0.1f, 100.0f);
    glCheckError();

    glCheckError();
    // send projection matrix to shader
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));
    glCheckError();

    view = myCamera.getViewMatrix();
    glUniformMatrix4fv(glGetUniformLocation(myBasicShader.shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));

    //glUniformMatrix4fv(lightSpaceMatrixLoc2, 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));
    glUniformMatrix4fv(glGetUniformLocation(myBasicShader.shaderProgram, "lightSpaceTrMatrix"), 1, GL_FALSE, glm::value_ptr(computeLightSpaceTrMatrix()));


    glViewport(0, 0, (float)myWindow.getWindowDimensions().width, (float)myWindow.getWindowDimensions().height);

    myBasicShader.useShaderProgram();

    /*skyboxShader.useShaderProgram();
    glUniformMatrix4fv(skyBoxProjectionLoc, 1, GL_FALSE, glm::value_ptr(projection));*/

    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, depthMapTexture);
    glUniform1i(glGetUniformLocation(myBasicShader.shaderProgram, "shadowMap"), 3);

    // render the teapot
    renderTeapot(myBasicShader, depthPass);
    glCheckError();

    //render the desert house
    renderHouse(myBasicShader, depthPass);
    renderHouse2(myBasicShader, depthPass);
    renderHouse3(myBasicShader, depthPass);
    renderSoldier(myBasicShader, depthPass);
    glCheckError();

    renderHelmet(myBasicShader, depthPass);
    glCheckError();

    renderPedestal(myBasicShader, depthPass);
    glCheckError();

    renderHammer(myBasicShader, depthPass);

    renderFenceLeft(myBasicShader, depthPass);
    renderFenceFront(myBasicShader, depthPass);
    renderFenceRight(myBasicShader, depthPass);
    renderFenceBack(myBasicShader, depthPass);
    renderShip(myBasicShader, depthPass);
    renderGeneratedShip(myBasicShader, depthPass);

    // render the cub floor
    renderCubeFloor(myBasicShader, depthPass);
    glCheckError();

    // render the skybox
    renderSkyBox(skyboxShader);
    glCheckError();
}

void animateObjects(float currentTime) {
    // Animation parameters
    float pedestalAmplitude = 0.5f;
    float pedestalSpeed = 1.0f;
    float hammerRotationSpeed = 30.0f;

    // Update pedestal's Y position
    pedestalY = pedestalAmplitude * sin(currentTime * pedestalSpeed);
    pedestalModelMatrix = glm::translate(pedestalModelMatrix, glm::vec3(0.0f, pedestalY, 0.0f));

    // Update hammer's Y position and rotation around the Y axis
    hammerY = pedestalY + 5.0f;  // Adjust the height above the pedestal
    float hammerRotation = glm::radians(currentTime * hammerRotationSpeed);
    hammerRotate = hammerRotation;
    hammerModelMatrix = glm::translate(hammerModelMatrix, glm::vec3(0.0f, hammerY, 0.0f));
    hammerModelMatrix = glm::rotate(hammerModelMatrix, hammerRotate, glm::vec3(0.0f, 1.0f, 0.0f));

}

void generateShip(float currentTime) {
    float shipSpeed = 40.0f;
    //float tempShipZ = sin(currentTime * shipSpeed);
    if (shipZ <= desiredShipZ) {
        // If so, generate a new random x-coordinate
        //std::cout << "mere inainte si shipZ: " << shipZ << endl;
        shipX = (-30 + rand() % 61);
        shipZ = originalZ;
        //std::cout << "mere dupa si shipZ: " << shipZ << endl;
        //generatedShipMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(shipX, 0.0f, originalZ));
    }
    else {
        // Otherwise, keep the previous x-coordinate
        shipZ = shipZ - (currentTime * shipSpeed);
        generatedShipMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(shipX, 0.0f, shipZ));
    }
}

void cleanup() {
    myWindow.Delete();
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    //cleanup code for your own data
}

int main(int argc, const char * argv[]) {

    try {
        initOpenGLWindow();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    srand(time(NULL));

    initOpenGLState();
	initModels();
	initShaders();
	initUniforms();
    initCollisionBoxes();
    initShadowMap();
    setWindowCallbacks();

    double lastFrameTime = glfwGetTime();
    float animationDuration = 5.0f;
    float currentAnimationTime = 0.0f;
    float currentTime_animation_hammer = 0.0f;
	// application loop
	while (!glfwWindowShouldClose(myWindow.getWindow())) {
        double currentFrameTime = glfwGetTime();
        deltaTime = static_cast<float>(currentFrameTime - lastFrameTime);
        lastFrameTime = currentFrameTime;

        currentTime_animation_hammer += deltaTime;

        while (currentAnimationTime < animationDuration) {
            // Calculate delta time
            double currentFrameTime = glfwGetTime();
            float deltaTime = static_cast<float>(currentFrameTime - lastFrameTime);
            lastFrameTime = currentFrameTime;
            glCheckError();

            // Increment animation time
            currentAnimationTime += deltaTime;
            glCheckError();

            // Calculate interpolation factor
            float t = glm::clamp(currentAnimationTime / animationDuration, 0.0f, 1.0f);
            glCheckError();

            // Interpolate camera position gradually from initial to desired position
            glm::vec3 animatedCameraPosition = initialCameraPosition + t * (desiredCameraPosition - initialCameraPosition);
            glCheckError();

            // Recreate the camera with the animated position
            myCamera = gps::Camera(
                animatedCameraPosition,
                glm::vec3(0.0f, 0.0f, -10.0f), // Assuming these values remain constant
                glm::vec3(0.0f, 1.0f, 0.0f),
                glm::vec3(0.0f, 0.0f, -1.0f),
                glm::vec3(1.0f, 0.0f, 0.0f),
                glm::vec3(0.0f, 0.0f, -1.0f)
            );
            glCheckError();

            // Update the view matrix using the new camera object
            view = myCamera.getViewMatrix();
            //processMovement();
            renderScene(); // bai aici
            glfwPollEvents();
            glfwSwapBuffers(myWindow.getWindow());
        }
        dynamicsWorld->stepSimulation(deltaTime);
        animateObjects(currentTime_animation_hammer);
        generateShip(deltaTime);
        // Render the scene during the animation
        processMovement();
        glm::vec3 playerPosition = player->getPlayerPosition();
        myCamera.setCameraPosition(playerPosition);
        renderScene();

        glfwPollEvents();
        glfwSwapBuffers(myWindow.getWindow());

        glCheckError();
	}

	cleanup();

    return EXIT_SUCCESS;
}
