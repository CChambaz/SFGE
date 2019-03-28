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

#ifndef SFGE_PHYSICS_H
#define SFGE_PHYSICS_H

#include <Box2D/Box2D.h>
#include <SFML/System/Time.hpp>

#include <engine/system.h>
#include <physics/collider2d.h>
#include <physics/body2d.h>

namespace sfge
{
	

float pixel2meter(float pixel);
float pixel2meter(int pixel);
b2Vec2 pixel2meter(sf::Vector2f pixel);
b2Vec2 pixel2meter(Vec2f pixel);
b2Vec2 pixel2meter(sf::Vector2i pixel);

float meter2pixel(float meter);
Vec2f meter2pixel(b2Vec2 meter);

class ContactListener : public b2ContactListener
{
public:
	ContactListener(Engine& engine);
	void BeginContact(b2Contact* contact) override;

	void EndContact(b2Contact* contact) override;
protected:
	Engine & m_Engine;
};

class RaycastCallback : public b2RayCastCallback
{
public:
	float32 fraction;
	b2Fixture* touchedFixture = nullptr;
	float32 ReportFixture(	b2Fixture* fixture, const b2Vec2& point,
									  const b2Vec2& normal, float32 fraction) override;


};
/**
 * \brief The Physics Manager use Box2D to simulate 2D physics
 */
class Physics2dManager : public System
{
public:
	using System::System;
	~Physics2dManager() = default;
	/**
	 * \brief Initialize the Physics Manager, but do not create a b2World
	 */
	void OnEngineInit() override;

	/**
	* \brief Get The World
	*/
	std::weak_ptr<b2World> GetWorld() const;
	/**
	 * \brief Called each frame to update the b2World if not in editor mode
	 * @param dt Delta time since last frame
	 */
	void OnUpdate(float dt) override;
	void OnFixedUpdate() override;
	/**
	* \brief Called at the end of the program to Destroy a b2World, if it sill exists
	*/
	void Destroy() override;

	void OnBeforeSceneLoad() override;
	void OnAfterSceneLoad() override;


	Body2dManager* GetBodyManager();
	ColliderManager* GetColliderManager();

	float Raycast(Vec2f startPoint, Vec2f direction, float rayLength);

	const static float pixelPerMeter;
private:
	friend class Body2d;
	std::shared_ptr<b2World> m_World = nullptr;

	std::unique_ptr<ContactListener> m_ContactListener = nullptr;
	Body2dManager m_BodyManager{m_Engine};
	ColliderManager m_ColliderManager{m_Engine};

};



}
#endif
