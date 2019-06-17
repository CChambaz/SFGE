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
#include <graphics/graphics2d.h>
#include <engine/engine.h>
#include <utility/log.h>
#include <engine/config.h>

//Dependencies includes
#include <SFML/Graphics/RenderWindow.hpp>
#include <imgui-SFML.h>
#include <imgui.h>


namespace sfge
{

void Graphics2dManager::OnEngineInit()
{
	if (const auto configPtr = m_Engine.GetConfig())
	{
		m_Windowless = configPtr->windowLess;
		if (!m_Windowless)
		{
			sf::ContextSettings settings;
			settings.depthBits = 24;
			settings.stencilBits = 8;
			settings.antialiasingLevel = 4;
			settings.majorVersion = 4;
			settings.minorVersion = 6;
			m_Window = std::make_unique<sf::RenderWindow>(
				sf::VideoMode(configPtr->screenResolution.x, configPtr->screenResolution.y),
				configPtr->windowName, sf::Style::Default, settings);
			if (configPtr->maxFramerate)
			{
				m_Window->setFramerateLimit(configPtr->maxFramerate);
				CheckVersion();
			}

		}
	}
	else
	{
		Log::GetInstance()->Error("[Error] Config is null from Graphics Manager");
		
	}
	m_TextureManager.OnEngineInit();
	m_ShapeManager.OnEngineInit();
	m_SpriteManager.OnEngineInit();

}

void Graphics2dManager::OnUpdate(float dt)
{
	if (!m_Windowless)
	{
		rmt_ScopedCPUSample(Graphics2dUpdate,0)
		m_Window->clear();

		m_SpriteManager.OnUpdate(dt);
		m_ShapeManager.OnUpdate(dt);

		
	}
}

void Graphics2dManager::OnDraw()
{
	rmt_ScopedCPUSample(Graphics2dDraw,0);
	if(!m_Windowless)
	{
		m_SpriteManager.DrawSprites(*m_Window);
		m_ShapeManager.DrawShapes(*m_Window);
		p2DebugDraw();
	}
}

void Graphics2dManager::Display()
{

	rmt_ScopedCPUSample(Graphics2dDisplay,0)
	if (!m_Windowless)
	{
		m_Window->display();
	}
}

void Graphics2dManager::DrawLine(Vec2f from, Vec2f to, sf::Color color)
{
	sf::Vertex vertices[2] =
	{
	    sf::Vertex(from, color),
	    sf::Vertex(to, color)
	};

	m_Window->draw(vertices, 2, sf::Lines);
}

sf::RenderWindow* Graphics2dManager::GetWindow()
{
	return m_Window.get();
}

SpriteManager* Graphics2dManager::GetSpriteManager()
{
	return &m_SpriteManager;
}

TextureManager* Graphics2dManager::GetTextureManager()
{
	return &m_TextureManager;
}

ShapeManager* Graphics2dManager::GetShapeManager()
{
	return &m_ShapeManager;
}

void Graphics2dManager::CheckVersion() const
{
	sf::ContextSettings settings = m_Window->getSettings();
	std::stringstream log_message;
	log_message << "OpenGL version: " << settings.majorVersion << "." << settings.minorVersion << std::endl;

	Log::GetInstance()->Msg(log_message.str());
}


void Graphics2dManager::Destroy()
{
	OnBeforeSceneLoad();
	OnAfterSceneLoad();

	m_Window = nullptr;
}

void Graphics2dManager::OnBeforeSceneLoad()
{
	m_TextureManager.OnBeforeSceneLoad();
	m_SpriteManager.OnBeforeSceneLoad();
}

void Graphics2dManager::OnAfterSceneLoad()
{

	m_TextureManager.OnAfterSceneLoad();
	m_SpriteManager.OnAfterSceneLoad();
}

void Graphics2dManager::DrawVector(Vec2f drawingVector, Vec2f originPos, sf::Color color)
{

    const Vec2f destination = originPos+drawingVector*debugVectorPixelResolution;
    //Draw length line
    DrawLine(originPos, destination, color);
    const Vec2f dir = drawingVector.Normalized();
    const float length = (drawingVector*debugVectorPixelResolution).GetMagnitude();
    DrawLine(destination, destination+dir.Rotate(135.0f)*length/5,color);
    DrawLine(destination, destination+dir.Rotate(-135.0f)*length/5,color);

}

void Graphics2dManager::p2DebugDraw()
{
	if (m_World == nullptr)
	{
		m_World = m_Engine.GetPhysicsManager()->GetWorldSharedPointer();
		m_WorldBodies = m_World->GetBodies();
		m_WorldQuadTree = m_World->GetQuad();
	}

	if (m_World->debugBodies)
	{
		for each (p2Body body in *m_WorldBodies)
		{
			p2Vec2 currentPosition = (body.GetPosition()) * 100;
			p2Vec2 currentForce = body.GetLinearVelocity();
			Vec2f currentAABBMinPosition = meter2pixel(body.GetMinPosition());
			Vec2f currentAABBMaxPosition = meter2pixel(body.GetMaxPosition());

			float maxX = currentAABBMaxPosition.x;
			float maxY = currentAABBMaxPosition.y;

			float minX = currentAABBMinPosition.x;
			float minY = currentAABBMinPosition.y;

			// Draw AABB
			DrawLine(Vec2f(minX, minY), Vec2f(maxX, minY), sf::Color::Red);	// bottom
			DrawLine(Vec2f(minX, minY), Vec2f(minX, maxY), sf::Color::Red);	// left
			DrawLine(Vec2f(maxX, minY), Vec2f(maxX, maxY), sf::Color::Red);	// right
			DrawLine(Vec2f(minX, maxY), Vec2f(maxX, maxY), sf::Color::Red);	// top
			DrawLine(Vec2f(minX, minY), Vec2f(maxX, maxY), sf::Color::Yellow);	// Diagonal

			// Draw force
			DrawVector(Vec2f(currentForce.x, currentForce.y), Vec2f(currentPosition.x, currentPosition.y), sf::Color::Blue);
		}
	}
	if(m_World->debugQuadTree)
	{
		std::vector<p2AABB> quadsAABB;
		quadsAABB = m_WorldQuadTree->RetrieveAABB(quadsAABB);

		for each (p2AABB aabb in quadsAABB)
		{
			p2Vec2 aabbCenter = aabb.GetCenter();

			/*DrawLine(Vec2f(aabbCenter.x, aabb.m_BottomLeft.y), Vec2f(aabbCenter.x, aabb.m_TopRight.y), sf::Color::Yellow);
			DrawLine(Vec2f(aabb.m_BottomLeft.x, aabbCenter.y), Vec2f(aabb.m_TopRight.x, aabbCenter.y), sf::Color::Yellow);
			*/
			float minY = aabb.m_BottomLeft.y;
			float minX = aabb.m_BottomLeft.x;

			float maxY = aabb.m_TopRight.y;
			float maxX = aabb.m_TopRight.x;

			DrawLine(Vec2f(minX, minY), Vec2f(maxX, minY), sf::Color::Green);	// bottom
			DrawLine(Vec2f(minX, minY), Vec2f(minX, maxY), sf::Color::Green);	// left
			DrawLine(Vec2f(maxX, minY), Vec2f(maxX, maxY), sf::Color::Green);	// right
			DrawLine(Vec2f(minX, maxY), Vec2f(maxX, maxY), sf::Color::Green);	// top
		}
	}
}

}
