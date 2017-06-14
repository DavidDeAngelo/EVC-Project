/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BASIC_EXAMPLE_H
#define BASIC_EXAMPLE_H
#include "../Evolution/TimeWarpBaseMultiBody.h"


struct MultiBodyTest : public TimeWarpBaseMultiBody
{
	MultiBodyTest(struct GUIHelperInterface* helper)
		:TimeWarpBaseMultiBody(helper)
	{
	}
	btMultiBody* m_multiBody;
	virtual ~MultiBodyTest();
	btVector3 MultiBodyTest::anglesToVector(float a, float b);
	virtual void initPhysics();
	float friction = 1.;
	virtual void renderScene();
	void castRays();
	btMultiBody* createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld *pWorld, int numLinks, const btVector3 &basePosition, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents, bool spherical, bool floating);
	void addColliders_testMultiDof(btMultiBody *pMultiBody, btMultiBodyDynamicsWorld *pWorld, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents);
	void addBoxes_testMultiDof(btBoxShape* colShapeIn);
	btMultiBody* createMultiRagDoll(btMultiBodyDynamicsWorld *pWorld);
	void stepSimulation(float deltaTime);
	void resetCamera()
	{
		float dist = 4;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3] = { 0,0,0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
};


class CommonExampleInterface*    BasicExampleCreateFunc(struct CommonExampleOptions& options);


#endif //BASIC_DEMO_PHYSICS_SETUP_H
