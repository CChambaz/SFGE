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

#include <sstream>

#include <input/input.h>
#include <utility/log.h>
#include <SFML/Window.hpp>
#include <imgui-SFML.h>
#include <imgui.h>

#include <graphics/graphics2d.h>

namespace sfge
{


KeyboardManager& InputManager::GetKeyboardManager()
{
	return m_KeyboardManager;
}

MouseManager& InputManager::GetMouseManager()
{
	return m_MouseManager;
}

void InputManager::OnEngineInit()
{
}

void InputManager::OnUpdate(float dt)
{
	m_KeyboardManager.OnUpdate(dt);
}

void InputManager::Destroy()
{
}

void InputManager::OnBeforeSceneLoad()
{
}

void InputManager::OnAfterSceneLoad()
{
}

void KeyboardManager::OnUpdate(float dt)
{
	(void) dt;
	for (int i = 0; i < sf::Keyboard::KeyCount; i++)
	{
		keyPressedStatusArray[i].previousKeyPressed = keyPressedStatusArray[i].keyPressed;
		keyPressedStatusArray[i].keyPressed = sf::Keyboard::isKeyPressed(static_cast<sf::Keyboard::Key>(i));
		if(IsKeyDown(static_cast<sf::Keyboard::Key>(i)))
		{
			std::ostringstream oss;
			oss << "[Input] Pressing key: "<<i;
			Log::GetInstance()->Msg(oss.str());
		}
	}
}

bool KeyboardManager::IsKeyHeld(sf::Keyboard::Key key) const
{	
	return keyPressedStatusArray[(int) key].keyPressed;
}

bool KeyboardManager::IsKeyDown(sf::Keyboard::Key key) const
{
	return !keyPressedStatusArray[(int)key].previousKeyPressed && keyPressedStatusArray[(int)key].keyPressed;
}
bool KeyboardManager::IsKeyUp(sf::Keyboard::Key key) const
{
	return !keyPressedStatusArray[(int)key].keyPressed && keyPressedStatusArray[(int)key].previousKeyPressed;
}

sf::Vector2i MouseManager::GetLocalPosition(sf::Window& window) const
{
	return sf::Mouse::getPosition(window);
}

float MouseManager::GetWheelDelta()
{
	return 0.0f;
}

sf::Vector2i MouseManager::GetPosition() const
{
	auto* windowPtr = m_Engine.GetGraphics2dManager()->GetWindow();
	return sf::Mouse::getPosition(*windowPtr);
}
}
