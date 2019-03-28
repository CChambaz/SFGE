/*
MIT License

Copyright (c) 2017 SAE Institute Switzerland AG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef SFGE_GRAPHICS3D_H
#define SFGE_GRAPHICS3D_H

#include <vector>
#include <string>

#include <GL/glew.h>
#include <SFML/OpenGL.hpp>

#include <engine/system.h>
#include <glm/detail/type_mat4x3.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat2x2.hpp>

namespace sfge
{
class DrawingProgram;

class Shader
{
public:
	void Init(std::string vertexShaderPath, std::string fragmentShaderPath);
	void Bind();
	int GetProgram();

	void SetBool(const std::string& attributeName, bool value) const;
	void SetInt(const std::string& attributeName, int value) const;
	void SetFloat(const std::string& attributeName, float value) const;
	void SetVec2(const std::string & name, const glm::vec2 & value) const;
	void SetVec2(const std::string& name, float x, float y) const;
	void SetVec3(const std::string & name, const glm::vec3 & value) const;
	void SetVec3(const std::string& name, float x, float y, float z) const;
	void SetVec4(const std::string & name, const glm::vec4 & value) const;
	void SetVec4(const std::string& name, float x, float y, float z, float w);
	void SetMat2(const std::string & name, const glm::mat2 & mat) const;
	void SetMat3(const std::string & name, const glm::mat3 & mat) const;
	void SetMat4(const std::string & name, const glm::mat4 & mat) const;
private:
	int shaderProgram = 0;
};

class Graphics3dManager : public System
{
public:
	using System::System;
	void AddDrawingProgam(DrawingProgram* drawingProgram);
	void OnEngineInit() override;
	void OnDraw() override;
	void Destroy() override;
	std::vector<DrawingProgram*>& GetDrawingPrograms();
private:
	std::vector<DrawingProgram*> m_DrawingPrograms;
};

class DrawingProgram : public System
{
public:
	using System::System;

	void OnEditorDraw() override;

	const std::string& GetProgramName();
	const std::vector<Shader*>& GetShaders();
protected:
	std::vector<Shader*> shaders;
	std::string programName;
};
}
#endif