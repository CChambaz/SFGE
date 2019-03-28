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

#include <graphics/graphics2d.h>
#include <graphics/sprite2d.h>
#include <graphics/texture.h>
#include <utility/file_utility.h>

#include <utility/log.h>
#include <engine/engine.h>
#include <engine/config.h>
#include <engine/transform2d.h>

#include <imgui.h>
#include <imgui-SFML.h>

namespace sfge
{

Sprite::Sprite() : Offsetable(sf::Vector2f())
{
}

Sprite::Sprite(Transform2d* transform, sf::Vector2f offset) : Offsetable(offset)
{
}
void Sprite::Draw(sf::RenderWindow& window)
{
	window.draw(sprite);
}
void Sprite::SetTexture(sf::Texture* newTexture)
{
	sprite.setTexture(*newTexture);

	sprite.setOrigin(sf::Vector2f(sprite.getLocalBounds().width, sprite.getLocalBounds().height) / 2.0f);
}


void Sprite::Init()
{
}

void Sprite::Update()
{
	sprite.setPosition(transform.Position + m_Offset);
	sprite.setScale(transform.Scale);
	sprite.setRotation(transform.EulerAngle);

}


void editor::SpriteInfo::DrawOnInspector()
{
	ImGui::Separator();
	ImGui::Text("Sprite");
	ImGui::LabelText("Texture Path", "%s", texturePath.c_str());
	ImGui::InputInt("Texture Id", (int*)&textureId);
	/*if(sprite)
	{
		float offset[2] =
		{
			sprite->GetOffset().x,
			sprite->GetOffset().y
		};
		ImGui::InputFloat2("Offset", offset);
	}*/
}


void SpriteManager::OnEngineInit()
{
	SingleComponentManager::OnEngineInit();
	m_GraphicsManager = m_Engine.GetGraphics2dManager();
	m_Transform2dManager = m_Engine.GetTransform2dManager();

}

Sprite* SpriteManager::AddComponent(Entity entity)
{
	auto& sprite = GetComponentRef(entity);
	auto& spriteInfo = GetComponentInfo(entity);

	//sprite.SetTransform(m_Transform2dManager->GetComponentPtr(entity));
	//spriteInfo.sprite = &sprite;

	m_EntityManager->AddComponentType(entity, ComponentType::SPRITE2D);
	return &sprite;
}


void SpriteManager::OnUpdate(float dt)
{
	(void) dt;

	rmt_ScopedCPUSample(SpriteUpdate, 0);
	auto* transformManager = m_Engine.GetTransform2dManager();
	for(auto i = 0u; i < m_Components.size();i++)
	{
		if(m_EntityManager->HasComponent(i+1, ComponentType::SPRITE2D))
		{
			if(m_EntityManager->HasComponent(i+1, ComponentType::TRANSFORM2D))
			{
				m_Components[i].transform = transformManager->GetComponentRef(i + 1);
			}
			m_Components[i].Update();
		}
	}
}


void SpriteManager::DrawSprites(sf::RenderWindow &window)
{

	rmt_ScopedCPUSample(SpriteDraw,0)
	for (auto i = 0u; i < m_Components.size();i++)
	{
		if(m_EntityManager->HasComponent(i + 1, ComponentType::SPRITE2D))
			m_Components[i].Draw(window);
	}
	
}

void SpriteManager::OnBeforeSceneLoad()
{
}

void SpriteManager::OnAfterSceneLoad()
{
	
}

void SpriteManager::CreateComponent(json& componentJson, Entity entity)
{
	auto & newSprite = m_Components[entity - 1];
	auto & newSpriteInfo = m_ComponentsInfo[entity - 1];
	if (CheckJsonParameter(componentJson, "path", json::value_t::string))
	{
		std::string path = componentJson["path"].get<std::string>();
		newSpriteInfo.texturePath = path;
		sf::Texture* texture = nullptr;
		if (FileExists(path))
		{
			auto* textureManager = m_GraphicsManager->GetTextureManager();
			const TextureId textureId = textureManager->LoadTexture(path);
			if (textureId != INVALID_TEXTURE)
			{
				/*{
					std::ostringstream oss;
					oss << "Loading Sprite with Texture at: " << path << " with texture id: " << textureId;
					sfge::Log::GetInstance()->Msg(oss.str());
				}*/
				texture = textureManager->GetTexture(textureId);
				newSprite.SetTexture(texture);
				//newSprite.SetTransform(m_Transform2dManager->GetComponentPtr(entity));
				newSpriteInfo.textureId = textureId;
			}
			else
			{
				std::ostringstream oss;
				oss << "Texture file " << path << " cannot be loaded";
				Log::GetInstance()->Error(oss.str());
			}
		}
		else
		{
			std::ostringstream oss;
			oss << "Texture file " << path << " does not exist";
			Log::GetInstance()->Error(oss.str());
		}
	}
	else
	{
		Log::GetInstance()->Error("[Error] No Path for Sprite");
	}
	if (CheckJsonParameter(componentJson, "layer", json::value_t::number_integer))
	{
		newSprite.SetLayer(componentJson["layer"]);
	}

}

void SpriteManager::DestroyComponent(Entity entity)
{
	(void) entity;
}

void SpriteManager::OnResize(size_t new_size)
{
	m_Components.resize(new_size);
	m_ComponentsInfo.resize(new_size);
}
}
