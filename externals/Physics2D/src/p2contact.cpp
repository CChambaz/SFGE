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

#include <p2contact.h>

p2Contact::p2Contact(p2Collider* col1, p2Collider* col2)
{
	m_ColliderA = col1;
	m_ColliderB = col2;
}

p2Collider * p2Contact::GetColliderA()
{
	return m_ColliderA;
}

p2Collider * p2Contact::GetColliderB()
{
	return m_ColliderB;
}

void p2ContactManager::CreateContact(p2Collider* colliderA, p2Collider* colliderB)
{
	// Try to get the contact composed with the given colliders
	p2Contact* checkContact = GetContact(colliderA, colliderB);

	// Check if a contact has been found
	if (checkContact != nullptr)
		return;

	// Create the contact
	m_Contacts.emplace_back(colliderA, colliderB);
	m_ContactIndex++;
}

p2Contact* p2ContactManager::GetContact(p2Collider* colliderA, p2Collider* colliderB)
{
	for (int i = 0; i < m_ContactIndex; i++)
	{
		// Get the colliders of the current contact
		p2Collider* colA = m_Contacts[i].GetColliderA();
		p2Collider* colB = m_Contacts[i].GetColliderB();

		// Check if the contact's colliders are the same as the function parameters
		if((colA == colliderA && colB == colliderB) || (colA == colliderB && colB == colliderA))
		{
			return &m_Contacts[i];
		}
	}

	// Contact does not exist
	return nullptr;
}

void p2ContactManager::ApplyContacts(p2ContactListener* contactListener)
{
	for(int i = 0; i < m_ContactIndex; i++)
	{
		contactListener->BeginContact(&m_Contacts[i]);
	}
}

void p2ContactManager::EndContacts(p2ContactListener* contactListener)
{
	for (int i = 0; i < m_ContactIndex; i++)
	{
		contactListener->EndContact(&m_Contacts[i]);
	}
}

void p2ContactManager::DestroyContacts()
{
	m_Contacts.clear();
	m_ContactIndex = 0;
}


