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


//STL
#include <sstream>
#include <list>
#include <set>
#include <memory>

#include <graphics/texture.h>
#include <utility/log.h>
#include <engine/config.h>
#include <engine/engine.h>
#include <utility/file_utility.h>



namespace sfge
{

static std::set<std::string> imgExtensionSet
{
	".png",
	".jpg",
	".jpeg",
	".bmp",
	".tga",
	".gif",
	".psd",
	".hdr",
	".pic"
};


void TextureManager::OnEngineInit()
{
	System::OnEngineInit();
	if(const auto config = m_Engine.GetConfig())
	{
		if(config->devMode)
		{
			LoadTextures(config->dataDirname);
		}
	}
}

void TextureManager::LoadTextures(std::string dataDirname)
{
	std::function<void(std::string)> LoadAllTextures;
	LoadAllTextures = [&LoadAllTextures, this](std::string entry)
	{
		if (IsRegularFile(entry) && HasValidExtension(entry))
		{
			const TextureId newTextureId = LoadTexture(entry);
			if (newTextureId != INVALID_TEXTURE)
			{
				std::ostringstream oss;
				oss << "Loading texture: " << entry << "\n";
				Log::GetInstance()->Msg(oss.str());
			}
		}

		if (IsDirectory(entry))
		{
			IterateDirectory(entry, LoadAllTextures);
		}
	};
	IterateDirectory(dataDirname, LoadAllTextures);
}

TextureId TextureManager::LoadTexture(std::string filename)
{
	if (!HasValidExtension (filename))
	{
		std::ostringstream oss;
		oss << "[ERROR] Texture path: " << filename << " has invalid extension";
		Log::GetInstance()->Error(oss.str());
		return INVALID_TEXTURE;
	}

	auto textureId = INVALID_TEXTURE;
	for (TextureId checkedId = 1U; checkedId <= m_IncrementId; checkedId++)
	{
		if (filename == m_TexturePaths[checkedId-1])
		{
			textureId = checkedId;
		}
	}
	//Was or still is loaded
	if (textureId != INVALID_TEXTURE)
	{
		
		//Check if the texture was destroyed
		if (m_Textures[textureId - 1].getNativeHandle () != 0U)
		{
			m_TextureIdsRefCounts[textureId-1]++;
			return textureId;
		}
		else
		{
			auto& texture = m_Textures[textureId-1];
			if (!texture.loadFromFile(filename))
			{
				std::ostringstream oss;
				oss << "[ERROR] Could not load texture file: " << filename;
				Log::GetInstance()->Error(oss.str());
				return INVALID_TEXTURE;
			}
			m_TextureIdsRefCounts[textureId-1] = 1U;
			return m_IncrementId;
		}
	}
	//Texture was never loaded
	if (FileExists(filename))
	{
		textureId = m_IncrementId+1;
		auto& texture = m_Textures[textureId-1] ;
		if (!texture.loadFromFile(filename))
		{
			std::ostringstream oss;
			oss << "[ERROR] Could not load texture file: " << filename;
			Log::GetInstance()->Error(oss.str());
			return INVALID_TEXTURE;
		}

		m_TexturePaths[textureId-1] = filename;
		m_TextureIdsRefCounts[textureId-1] = 1U;

		m_IncrementId++;
		return textureId;
	}
	else
	{
		std::ostringstream oss;
		oss << "[ERROR] Could not load texture file: " << filename;
		Log::GetInstance()->Error(oss.str());
	}
	return INVALID_TEXTURE;
}

sf::Texture* TextureManager::GetTexture(TextureId textureId)
{
	return &m_Textures[textureId-1];
}

bool TextureManager::HasValidExtension(std::string filename)
{
	const std::string::size_type filenameExtensionIndex = filename.find_last_of('.');
	if (filenameExtensionIndex >= filename.size())
	{
		return false;
	}

	//Check extension first
	const std::string extension = filename.substr(filenameExtensionIndex);
	if(imgExtensionSet.find(extension) == imgExtensionSet.end())
	{
		return false;
	}
	return true;
}

void TextureManager::OnBeforeSceneLoad()
{
	for (auto& textureIdsRefCount : m_TextureIdsRefCounts)
	{
		textureIdsRefCount = 0U;
	}
}

void TextureManager::OnAfterSceneLoad()
{
	std::list<TextureId> unusedTextureIds;
	for (auto i = 0U; i < m_TextureIdsRefCounts.size(); i++)
	{
		if(m_Textures[i].getNativeHandle () != 0U && m_TextureIdsRefCounts[i] == 0U )
		{
			unusedTextureIds.push_back(i+1);
		}
	}
	for (auto unusedTextureId : unusedTextureIds)
	{
		m_Textures[unusedTextureId-1] = sf::Texture();
	}
}

}
