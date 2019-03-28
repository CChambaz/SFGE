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

#ifndef SFGE_INPUT_H
#define SFGE_INPUT_H

#include <array>

#include <engine/system.h>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/System/Time.hpp>

namespace sfge
{

struct KeyPressedStatus { bool previousKeyPressed; bool keyPressed; };

class KeyboardManager : public System
{
public:
	using System::System;
	void OnUpdate(float dt) override;
	bool IsKeyHeld(sf::Keyboard::Key key) const;
	bool IsKeyDown(sf::Keyboard::Key key) const;
	bool IsKeyUp(sf::Keyboard::Key key) const;

private:
	std::array<KeyPressedStatus, sf::Keyboard::Key::KeyCount> keyPressedStatusArray= {};
};

class MouseManager : public System
{
public:
	using System::System;
	sf::Vector2i GetPosition() const;
	sf::Vector2i GetLocalPosition(sf::Window& window) const;
	float GetWheelDelta();
};
/**
* \brief Handles Input like the Keyboard, the Joystick or the Mouse
*/
class InputManager : public System
{
public:
	using System::System;
	/**
	 * \brief Initialize the Input Manager
	 */
	void OnEngineInit() override;
	/**
	 * \brief Update called each frame to report input status
	 * \param dt Delta time since last frame
	 */
	void OnUpdate(float dt) override;
	void Destroy() override;

	void OnBeforeSceneLoad() override;
	void OnAfterSceneLoad() override;

	KeyboardManager& GetKeyboardManager();
	MouseManager& GetMouseManager();

protected:
	KeyboardManager m_KeyboardManager{m_Engine};
	MouseManager m_MouseManager{m_Engine};
};




}
#endif
