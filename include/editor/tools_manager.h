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

#ifndef SFGE_TOOLS_MANAGER_H
#define SFGE_TOOLS_MANAGER_H

#include <string>
#include <array>

#include <engine/system.h>
#include <SFML/Window/Keyboard.hpp>
namespace sfge
{
class PySystemManager;
class PySystem;
using InstanceId = unsigned;

namespace editor
{
class ToolsManager : public System
{
public:
	using System::System;
    void OnEngineInit() override;
    void OnUpdate(float dt) override;
    void OnDraw() override;
private:
	std::vector<InstanceId> m_ToolSystems;
	std::vector<std::string> m_ToolSystemsNames;
	std::array<bool, 256> m_WhichToolsIsActive; //because std::vector<bool> is shit
	PySystemManager* m_PySystemManager = nullptr;
};
}
}
#endif