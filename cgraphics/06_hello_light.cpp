#include <engine/engine.h>
#include <graphics/graphics3d.h>
#include <input/input.h>

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include "graphics/camera.h"
#include <glm/gtc/type_ptr.hpp>


class HelloLightDrawingProgram : public sfge::DrawingProgram
{
public:
	using sfge::DrawingProgram::DrawingProgram;
	void OnEngineInit() override;
	void OnDraw() override;
	void Destroy() override;
	void ProcessInput();

private:
	sfge::Shader objShaderProgram;
	sfge::Shader lampShaderProgram;

	float vertices[6*36] = {
		-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
		 0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
		-0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,

		-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
		 0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
		-0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
		-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,

		-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
		-0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
		-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
		-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
		-0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
		-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,

		 0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
		 0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
		 0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
		 0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
		 0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,

		-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
		 0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
		 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
		 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,

		-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
		-0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
		-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f
	};

	unsigned int VBO = 0, cubeVAO = 0;
	unsigned int lightVAO;

	glm::vec3 lightPos = { 2.0f, 0.0f, 2.0f };

	sfge::Camera camera = sfge::Camera(glm::vec3(0.0f, 0.0f, 3.0f));
	float lastX = 0;
	float lastY = 0;
};

void HelloLightDrawingProgram::OnEngineInit()
{
	programName = "HelloLight";

	auto* config = m_Engine.GetConfig();
	lastX = config->screenResolution.x / 2.0f;
	lastY = config->screenResolution.y / 2.0f;

	objShaderProgram.Init(
		"data/shaders/06_hello_light/light.vert",
		"data/shaders/06_hello_light/light.frag");
	lampShaderProgram.Init(
		"data/shaders/06_hello_light/lamp.vert",
		"data/shaders/06_hello_light/lamp.frag");
	shaders.push_back(&objShaderProgram);
	shaders.push_back(&lampShaderProgram);

	// first, configure the cube's VAO (and VBO)

	glGenVertexArrays(1, &cubeVAO);
	glGenBuffers(1, &VBO);

	glBindVertexArray(cubeVAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glGenVertexArrays(1, &lightVAO);
	glBindVertexArray(lightVAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// note that we update the lamp's position attribute's stride to reflect the updated buffer data
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);
}

void HelloLightDrawingProgram::OnDraw()
{
	ProcessInput();

	auto* config = m_Engine.GetConfig();

	lightPos = glm::vec3(2.0f*sin(2.0f*M_PI / 3.0f*m_Engine.GetTimeSinceInit()), lightPos.y, 2.0f*cos(2.0f*M_PI / 3.0f*m_Engine.GetTimeSinceInit()));

	glEnable(GL_DEPTH_TEST);

	objShaderProgram.Bind();
	glUniform1f(glGetUniformLocation(objShaderProgram.GetProgram(), "ambientStrength"), 0.2f);
	glUniform1f(glGetUniformLocation(objShaderProgram.GetProgram(), "specularStrength"), 0.5f);
	glUniform3f(glGetUniformLocation(objShaderProgram.GetProgram(), "objectColor"), 1.0f, 0.5f, 0.31f);
	glUniform3f(glGetUniformLocation(objShaderProgram.GetProgram(), "lightColor"), 1.0f, 1.0f, 1.0f);
	glUniform1i(glGetUniformLocation(objShaderProgram.GetProgram(), "specularPow"), 256);
	glUniform3fv(glGetUniformLocation(objShaderProgram.GetProgram(), "lightPos"), 1, &lightPos[0]);
	glUniform3fv(glGetUniformLocation(objShaderProgram.GetProgram(), "viewPos"), 1, &camera.Position[0]);
	glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)config->screenResolution.x / (float)config->screenResolution.y, 0.1f, 100.0f);
	glm::mat4 view = camera.GetViewMatrix();
	glUniformMatrix4fv(glGetUniformLocation(objShaderProgram.GetProgram(), "projection"), 1, GL_FALSE, &projection[0][0]);
	glUniformMatrix4fv(glGetUniformLocation(objShaderProgram.GetProgram(), "view"), 1, GL_FALSE, glm::value_ptr(view));

	// world transformation
	glm::mat4 model = glm::mat4(1.0f);
	glUniformMatrix4fv(glGetUniformLocation(objShaderProgram.GetProgram(), "model"), 1, GL_FALSE, glm::value_ptr(model));

	// render the cube
	glBindVertexArray(cubeVAO);
	glDrawArrays(GL_TRIANGLES, 0, 36);
	glBindVertexArray(0);

	// also draw the lamp object
	lampShaderProgram.Bind();
	glUniformMatrix4fv(glGetUniformLocation(lampShaderProgram.GetProgram(), "projection"), 1, GL_FALSE, glm::value_ptr(projection));
	glUniformMatrix4fv(glGetUniformLocation(lampShaderProgram.GetProgram(), "view"), 1, GL_FALSE, glm::value_ptr(view));

	model = glm::mat4(1.0f);
	model = glm::translate(model, lightPos);
	model = glm::scale(model, glm::vec3(0.2f)); // a smaller cube
	glUniformMatrix4fv(glGetUniformLocation(lampShaderProgram.GetProgram(), "model"), 1, GL_FALSE, glm::value_ptr(model));

	glBindVertexArray(lightVAO);
	glDrawArrays(GL_TRIANGLES, 0, 36);
	glBindVertexArray(0);

}

void HelloLightDrawingProgram::Destroy()
{
}



void HelloLightDrawingProgram::ProcessInput()
{
	auto* inputManager = m_Engine.GetInputManager();
	auto& keyboardManager = inputManager->GetKeyboardManager();
	float dt = m_Engine.GetDeltaTime();
	float cameraSpeed = 1.0f;
	if (keyboardManager.IsKeyHeld(sf::Keyboard::W))
	{
		camera.ProcessKeyboard(sfge::FORWARD, m_Engine.GetDeltaTime());
	}
	if (keyboardManager.IsKeyHeld(sf::Keyboard::S))
	{
		camera.ProcessKeyboard(sfge::BACKWARD, m_Engine.GetDeltaTime());
	}
	if (keyboardManager.IsKeyHeld(sf::Keyboard::A))
	{
		camera.ProcessKeyboard(sfge::LEFT, m_Engine.GetDeltaTime());
	}
	if (keyboardManager.IsKeyHeld(sf::Keyboard::D))
	{
		camera.ProcessKeyboard(sfge::RIGHT, m_Engine.GetDeltaTime());
	}

	auto& mouseManager = inputManager->GetMouseManager();
	auto mousePos = mouseManager.GetPosition();

	float xoffset = mousePos.x - lastX;
	float yoffset = lastY - mousePos.y; // reversed since y-coordinates go from bottom to top
	lastX = mousePos.x;
	lastY = mousePos.y;

	camera.ProcessMouseMovement(xoffset, yoffset);
	
	camera.ProcessMouseScroll(mouseManager.GetWheelDelta());


}


int main(int argc, char** argv)
{
	sfge::Engine engine;

	HelloLightDrawingProgram helloLight(engine);
	auto* graphics3dManager = engine.GetGraphics3dManager();
	graphics3dManager->AddDrawingProgam(&helloLight);

	{
		auto config = std::make_unique<sfge::Configuration>();
		config->screenResolution.x = 1024;
		config->screenResolution.y = 1024;
		config->windowName = "Hello Light";

		engine.Init(std::move(config));
	}

	engine.Start();

	return EXIT_SUCCESS;
}