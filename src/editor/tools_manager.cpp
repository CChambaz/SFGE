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
#include <vector>

#include <imgui.h>

#include <editor/tools_manager.h>
#include <input/input.h>
#include <python/python_engine.h>
#include <python/pysystem.h>
#include <utility/json_utility.h>

namespace sfge::editor
{
void ToolsManager::OnEngineInit()
{
	m_PySystemManager = &m_Engine.GetPythonEngine()->GetPySystemManager();

    auto toolConfigJsonPtr = LoadJson("data/tools_config.json");
	if(toolConfigJsonPtr != nullptr)
	{
		auto& toolConfigJson = *toolConfigJsonPtr.get();
		if(CheckJsonExists(toolConfigJson, "tools") && CheckJsonParameter(toolConfigJson, "tools", json::value_t::array))
		{
			int i = 0;
			for(auto& tool : toolConfigJson["tools"])
			{
				const InstanceId pySystemInstance = m_PySystemManager->LoadCppExtensionSystem(tool);
				PySystem* pySystem = m_PySystemManager->GetPySystemFromInstanceId(pySystemInstance);
				if(pySystem != nullptr)
				{
					m_ToolSystemsNames.push_back(tool);
					m_ToolSystems.push_back(pySystemInstance);
					m_WhichToolsIsActive[i] = false;
					i++;
				}
			}
		}
	}

}

void ToolsManager::OnUpdate(float dt)
{
	for(auto i = 0u; i < m_ToolSystems.size();i++)
	{
		if(m_ToolSystems[i] != INVALID_INSTANCE && m_WhichToolsIsActive[i])
		{
			auto* pySystem = m_PySystemManager->GetPySystemFromInstanceId(m_ToolSystems[i]);
			pySystem->OnUpdate(dt);
		}
	}
}

void ToolsManager::OnDraw()
{
	ImGui::Begin("Tool Window");
	for(auto i = 0u; i < m_ToolSystemsNames.size();i++)
	{
		auto& toolName = m_ToolSystemsNames[i];
		ImGui::Checkbox(toolName.c_str(), &m_WhichToolsIsActive[i]);
	}
	if (ImGui::Button("Create New Tool..."))
	{
		ImGui::OpenPopup("NewTool");
	}
	if (ImGui::BeginPopup("NewTool"))
	{
		static char buf1[64] = ""; ImGui::InputText("default", buf1, 64);
		if(ImGui::Button("Create"))
		{
			std::ostringstream oss;
			oss << "from scripts.tools.tool_generator import generate_new_tool\ngenerate_new_tool(\""<<buf1<<"\")";
			m_Engine.GetPythonEngine()->ExecutePythonCommand(oss.str());
		}

		ImGui::EndPopup();
	}
	ImGui::End();

	for(auto i = 0u; i < m_ToolSystems.size();i++)
	{
		if(m_ToolSystems[i] != INVALID_INSTANCE && m_WhichToolsIsActive[i])
		{
			auto* pySystem = m_PySystemManager->GetPySystemFromInstanceId(m_ToolSystems[i]);
			pySystem->OnDraw();
		}
	}

}
}