#if WIN32
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#include <engine/engine.h>
#include <input/input.h>
#include <graphics/graphics3d.h>

#include <SFML/Graphics/Texture.hpp>

#include <glm/glm.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/type_ptr.hpp>

const float pixelPerUnit = 100.0f;
//#define ORTHOGRAPHIC
#define CAMERA_CONTROLS

class HelloCameraDrawingProgram : public sfge::DrawingProgram
{
public:
	using sfge::DrawingProgram::DrawingProgram;
	~HelloCameraDrawingProgram() override;
    void OnEngineInit() override;
    void OnDraw() override;
	void Destroy() override;
private:
	void ProcessInput();
	sfge::Shader shaderProgram;
	unsigned int VBO, VAO;
	sf::Texture sfTextureWall;
	unsigned int textureWall;
	float vertices[5*36] = 
	{
		-0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
		 0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
		 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
		 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
		-0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

		-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
		 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
		-0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

		-0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
		-0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
		-0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

		 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
		 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
		 0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
		 0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
		 0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

		-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
		 0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
		 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
		 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

		-0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
		 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
		-0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
		-0.5f,  0.5f, -0.5f,  0.0f, 1.0f
	};

	glm::vec3 cubePositions[10] = {
	  glm::vec3(0.0f,  0.0f,  0.0f),
	  glm::vec3(2.0f,  5.0f, -15.0f),
	  glm::vec3(-1.5f, -2.2f, -2.5f),
	  glm::vec3(-3.8f, -2.0f, -12.3f),
	  glm::vec3(2.4f, -0.4f, -3.5f),
	  glm::vec3(-1.7f,  3.0f, -7.5f),
	  glm::vec3(1.3f, -2.0f, -2.5f),
	  glm::vec3(1.5f,  2.0f, -2.5f),
	  glm::vec3(1.5f,  0.2f, -1.5f),
	  glm::vec3(-1.3f,  1.0f, -1.5f)
	};

	float cubeInitialAngle[10] =
	{
		M_PI / 2.0f,
		M_PI / 4.0f,
		-M_PI / 4.0f,
		-M_PI / 2.0f,
		M_PI * 3.0f / 2.0f,
		M_PI * 3.0f/ 4.0f,
		M_PI * 3.0f / 4.0f,
		M_PI / 3.0f,
		M_PI / 6.0f,
		-M_PI/ 6.0f
	};
#ifdef CAMERA_CONTROLS
	glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
	glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
	glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

	float yaw = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
	float pitch = 0.0f;
	float fov = 45.0f;
	float fovScroolSpeed = 100.0f;
	float lastX = 800.0f / 2.0;
	float lastY = 600.0 / 2.0;
#endif
};

HelloCameraDrawingProgram::~HelloCameraDrawingProgram()
{
}

void HelloCameraDrawingProgram::OnEngineInit()
{
	programName = "HelloCamera";
	shaders.push_back(&shaderProgram);
    shaderProgram.Init("data/shaders/05_hello_camera/camera.vert", "data/shaders/05_hello_camera/camera.frag");

#ifdef CAMERA_CONTROLS
	auto* config = m_Engine.GetConfig();
	lastX = config->screenResolution.x/2.0f;
	lastY = config->screenResolution.y/2.0f;
#endif
	sfTextureWall.loadFromFile("data/sprites/wall.jpg");
	textureWall = sfTextureWall.getNativeHandle();
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// texture coord attribute
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0);

}

void HelloCameraDrawingProgram::OnDraw()
{
	ProcessInput();

	auto* config = m_Engine.GetConfig();

	glEnable(GL_DEPTH_TEST);
	
	glm::mat4 view = glm::mat4(1.0f);
#ifdef CAMERA_CONTROLS
	view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
#else
	float radius = 10.0f;
	float camX = sin(engine->GetTimeSinceInit()) * radius;
	float camZ = cos(engine->GetTimeSinceInit()) * radius;
	view = glm::lookAt(glm::vec3(camX, 0.0, camZ), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 1.0, 0.0));
#endif
#ifdef ORTHOGRAPHIC
	glm::mat4 projection = glm::ortho(
		-(float)config.screenSizeX / pixelPerUnit /2.0f, 
		(float)config.screenSizeX/pixelPerUnit/2.0f, 
		-(float)config.screenSizeY / pixelPerUnit / 2.0f,
		(float)config.screenSizeY / pixelPerUnit/2.0f, 
		0.1f, 100.0f);
#else
	glm::mat4 projection = glm::perspective(glm::radians(fov), (float)config->screenResolution.x / config->screenResolution.y, 0.1f, 100.0f);
#endif

	shaderProgram.Bind();
	const int viewLoc = glGetUniformLocation(shaderProgram.GetProgram(), "view");
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

	const int projectionLoc = glGetUniformLocation(shaderProgram.GetProgram(), "projection");
	glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));

	glBindTexture(GL_TEXTURE_2D, textureWall);
	glBindVertexArray(VAO);
	for(int i = 0; i < 10;i++)
	{
		glm::mat4 model = glm::mat4(1.0f);
		model = glm::translate(model, cubePositions[i]);
		model = glm::rotate(model, (float)m_Engine.GetTimeSinceInit() * glm::radians(50.0f) + cubeInitialAngle[i], glm::vec3(0.5f, 1.0f, 0.0f));
		
		const int modelLoc = glGetUniformLocation(shaderProgram.GetProgram(), "model");
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
		
		glDrawArrays(GL_TRIANGLES, 0, 36);
	}
	glBindVertexArray(0);
}

void HelloCameraDrawingProgram::Destroy()
{
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
}

void HelloCameraDrawingProgram::ProcessInput()
{
	auto* inputManager = m_Engine.GetInputManager();
	auto& keyboardManager = inputManager->GetKeyboardManager();
	
	float dt = m_Engine.GetDeltaTime();
	float cameraSpeed = 1.0f;
	if (keyboardManager.IsKeyHeld(sf::Keyboard::W))
	{
		cameraPos += cameraSpeed * cameraFront * dt;
	}
	if (keyboardManager.IsKeyHeld(sf::Keyboard::S))
	{
		cameraPos -= cameraSpeed * cameraFront * dt;
	}
	if (keyboardManager.IsKeyHeld(sf::Keyboard::A))
	{
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed * dt;
	}
	if (keyboardManager.IsKeyHeld(sf::Keyboard::D))
	{
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed* dt;
	}
	auto& mouseManager = inputManager->GetMouseManager();
	auto mousePos = mouseManager.GetPosition();

	float xoffset = mousePos.x - lastX;
	float yoffset = lastY - mousePos.y; // reversed since y-coordinates go from bottom to top
	lastX = mousePos.x;
	lastY = mousePos.y;

	float sensitivity = 0.1f; // change this value to your liking
	xoffset *= sensitivity;
	yoffset *= sensitivity;

	yaw += xoffset;
	pitch += yoffset;

	// make sure that when pitch is out of bounds, screen doesn't get flipped
	if (pitch > 89.0f)
		pitch = 89.0f;
	if (pitch < -89.0f)
		pitch = -89.0f;

	glm::vec3 front(0.0f,0.0f,0.0f);
	front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
	front.y = sin(glm::radians(pitch));
	front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
	cameraFront = glm::normalize(front);

	
	if (fov >= 1.0f && fov <= 45.0f)
		fov -= mouseManager.GetWheelDelta() * m_Engine.GetDeltaTime() * fovScroolSpeed;
	if (fov <= 1.0f)
		fov = 1.0f;
	if (fov >= 45.0f)
		fov = 45.0f;
}


int main(int argc, char** argv)
{
	sfge::Engine engine;

	auto* graphics3dManager = engine.GetGraphics3dManager();
	HelloCameraDrawingProgram helloCamera(engine);
	graphics3dManager->AddDrawingProgam(&helloCamera);

	{
		auto config = std::make_unique<sfge::Configuration>();
		config->screenResolution.x = 1024;
		config->screenResolution.y = 1024;
		config->windowName = "Hello Camera";

		engine.Init(std::move(config));
	}

    engine.Start();

    return EXIT_SUCCESS;
}
