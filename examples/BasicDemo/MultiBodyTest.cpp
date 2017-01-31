

#include "MultiBodyTest.h"
#include <iostream>
#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5



//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)


#define START_POS_X -5
//#define START_POS_Y 12
#define START_POS_Y 2
#define START_POS_Z -3


#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "../Evolution/TimeWarpBaseMultiBody.h"
MultiBodyTest::~MultiBodyTest() {
	m_guiHelper->removeAllGraphicsInstances();
	m_guiHelper->removeAllUserDebugItems();

}

void MultiBodyTest::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));


	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));

	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(.1, .1, .1));


		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);


		for (int k = ARRAY_SIZE_Y; k < ARRAY_SIZE_Y; k++)
		{
			for (int i = 0; i < ARRAY_SIZE_X; i++)
			{
				for (int j = 0; j < ARRAY_SIZE_Z; j++)
				{
					startTransform.setOrigin(btVector3(
						btScalar(0.2*i),
						btScalar(2 + .2*k),
						btScalar(0.2*j)));


					createRigidBody(mass, startTransform, colShape);


				}
			}
		}

		btTransform baseWorldTrans;
		baseWorldTrans.setIdentity();
		baseWorldTrans.setOrigin(btVector3(1, 2, 3));
		//m_multiBody = createInvertedPendulumMultiBody1(m_dynamicsWorld, m_guiHelper, baseWorldTrans, true);


		//	addBoxes_testMultiDof();
	}

	{
		btVector3 linkHalfExtents(0.05, 0.37, 0.1);
		btVector3 baseHalfExtents(0.05, 0.37, 0.1);
		bool g_floatingBase = true;
		bool canSleep = true;
		bool selfCollide = true;
		bool gyro = true;
		bool damping = true;
		int numLinks = 5;
		bool spherical = true;


		btMultiBody* mbC = createFeatherstoneMultiBody_testMultiDof(m_dynamicsWorld, numLinks, btVector3(0, 2.5f, 0), linkHalfExtents, linkHalfExtents, true, false);

		g_floatingBase = !g_floatingBase;
		mbC->setCanSleep(canSleep);
		mbC->setHasSelfCollision(selfCollide);
		mbC->setUseGyroTerm(gyro);
		//
		if (!damping)
		{
			mbC->setLinearDamping(0.f);
			mbC->setAngularDamping(0.f);
		}
		else
		{
			mbC->setLinearDamping(0.1f);
			mbC->setAngularDamping(0.9f);
		}
		//

		//m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
		//////////////////////////////////////////////
		if (numLinks > 0)
		{
			btScalar q0 = 270.f * SIMD_PI / 180.f;
			if (!spherical)
			{
				mbC->setJointPosMultiDof(0, &q0);
			}
			else
			{
				btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
				quat0.normalize();
				mbC->setJointPosMultiDof(0, quat0);
			}
		}
		///

		addColliders_testMultiDof(mbC, m_dynamicsWorld, linkHalfExtents, linkHalfExtents);
		mbC->setHasSelfCollision(false);
		//addBoxes_testMultiDof();
	}


	m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);


}


void MultiBodyTest::renderScene()
{
	TimeWarpBaseMultiBody::renderScene();

}

btMultiBody* MultiBodyTest::createMultiRagDoll(btMultiBodyDynamicsWorld *pWorld) {

	btVector3 baseDimensions(.3, 0.3, 0.3);
	btVector3 basePos(0.f, .5f, 0.f);
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;

	if (baseMass)
	{
		btCollisionShape *pTempBox = new btBoxShape(btVector3(baseDimensions[0], baseDimensions[1], baseDimensions[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}
	btMultiBody *pMultiBody = new btMultiBody(5, 1.f, baseInertiaDiag, true, true);
	pMultiBody->setBasePos(basePos);

	btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
	pMultiBody->setWorldToBaseRot(baseOriQuat);

	return pMultiBody;
}
btMultiBody* MultiBodyTest::createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld *pWorld, int numLinks, const btVector3 &basePosition, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents, bool spherical, bool floating)
{
	//init the base	
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;

	if (baseMass)
	{
		btCollisionShape *pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	bool canSleep = false;

	btMultiBody *pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);

	numLinks = 5;
	btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
	pMultiBody->setBasePos(basePosition);
	pMultiBody->setWorldToBaseRot(baseOriQuat);
	btVector3 vel(0, 0, 0);
	//	pMultiBody->setBaseVel(vel);

	//init the links	
	btVector3 hingeJointAxis(1, 0, 0);
	float linkMass = 1.f;
	btVector3 linkInertiaDiag(1.f, 0.f, 0.f);

	btCollisionShape *pTempBox = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));
	pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
	delete pTempBox;

	//y-axis assumed up
	btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] * 2.f, 0);						//par body's COM to cur body's COM offset	
	btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);							//cur body's COM to cur body's PIV offset
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;	//par body's COM to cur body's PIV offset
	parentComToCurrentPivot = btVector3(-linkHalfExtents[0] * 2.f, 0, 0);
	//////
	btScalar q0 = 0.f * SIMD_PI / 180.f;
	std::cout << q0;
	btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
	quat0.normalize();
	/////

	//pMultiBody->setupSpherical(0, linkMass, linkInertiaDiag, -1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	//pMultiBody->setupSpherical(1, linkMass, linkInertiaDiag, 0, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	/*
	for (int i = 2; i < numLinks-2; ++i)
	{
		if (!spherical)
			pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag,  i- 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, true);
		else
			//pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
			pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag,  i-1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	}
*/

	pMultiBody->setupSpherical(4, linkMass, linkInertiaDiag, 2, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	pMultiBody->setupSpherical(3, linkMass, linkInertiaDiag, 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	pMultiBody->setupSpherical(2, linkMass, linkInertiaDiag, 3, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	pMultiBody->setupSpherical(1, linkMass, linkInertiaDiag, 0, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	pMultiBody->setupSpherical(0, linkMass, linkInertiaDiag, -1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);



	pMultiBody->finalizeMultiDof();

	///
	pWorld->addMultiBody(pMultiBody);
	///
	return pMultiBody;
}

void MultiBodyTest::addColliders_testMultiDof(btMultiBody *pMultiBody, btMultiBodyDynamicsWorld *pWorld, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents)
{

	btAlignedObjectArray<btQuaternion> world_to_local;
	world_to_local.resize(pMultiBody->getNumLinks() + 1);

	btAlignedObjectArray<btVector3> local_origin;
	local_origin.resize(pMultiBody->getNumLinks() + 1);
	world_to_local[0] = pMultiBody->getWorldToBaseRot();
	local_origin[0] = pMultiBody->getBasePos();

	{

		//	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
		btScalar quat[4] = { -world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w() };


		if (1)
		{
			btCollisionShape* box = new btBoxShape(baseHalfExtents);
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
			col->setCollisionShape(box);
			//col->setCollisionFlags(col->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(local_origin[0]);
			tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
			col->setWorldTransform(tr);

			pWorld->addCollisionObject(col, 2, 1 + 2);



			col->setFriction(friction);
			pMultiBody->setBaseCollider(col);

		}
	}


	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
	{
		const int parent = pMultiBody->getParent(i);
		world_to_local[i + 1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
		local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), pMultiBody->getRVector(i)));
	}


	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
	{

		btVector3 posr = local_origin[i + 1];
		//	float pos[4]={posr.x(),posr.y(),posr.z(),1};

		btScalar quat[4] = { -world_to_local[i + 1].x(),-world_to_local[i + 1].y(),-world_to_local[i + 1].z(),world_to_local[i + 1].w() };
		btCollisionShape* box = new btBoxShape(linkHalfExtents);
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

		col->setCollisionShape(box);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
		col->setWorldTransform(tr);
		col->setFriction(friction);
		pWorld->addCollisionObject(col, 2, 1 + 2);


		pMultiBody->getLink(i).m_collider = col;
	}
}

void MultiBodyTest::addBoxes_testMultiDof()
{
	//create a few dynamic rigidbodies
	// Re-using the same collision is better for memory usage and performance

	btBoxShape* colShape = new btBoxShape(btVector3(1, 1, 1));
	//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
	m_collisionShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar	mass(1.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	float start_x = START_POS_X - ARRAY_SIZE_X / 2;
	float start_y = START_POS_Y;
	float start_z = START_POS_Z - ARRAY_SIZE_Z / 2;

	for (int k = 0; k < ARRAY_SIZE_Y; k++)
	{
		for (int i = 0; i < ARRAY_SIZE_X; i++)
		{
			for (int j = 0; j < ARRAY_SIZE_Z; j++)
			{
				startTransform.setOrigin(btVector3(
					btScalar(3.0*i + start_x),
					btScalar(3.0*k + start_y),
					btScalar(3.0*j + start_z)));


				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
				btRigidBody* body = new btRigidBody(rbInfo);

				m_dynamicsWorld->addRigidBody(body);//,1,1+2);
			}
		}
	}
}





CommonExampleInterface*    BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new MultiBodyTest(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)



