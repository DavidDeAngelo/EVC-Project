#include <math.h>  

#include "MultiBodyTest.h"
#include <iostream>
#include "btBulletDynamicsCommon.h"

#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "LinearMath/btAlignedObjectArray.h"


#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_X 2
#define ARRAY_SIZE_Z 1



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

struct	LastRayResultCallback : public btCollisionWorld::RayResultCallback
{
	LastRayResultCallback(const btVector3&	rayFromWorld, const btVector3&	rayToWorld)
		:m_rayFromWorld(rayFromWorld),
		m_rayToWorld(rayToWorld)
	{
	}

	btAlignedObjectArray<const btCollisionObject*>		m_collisionObjects;

	btVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
	btVector3	m_rayToWorld;

	btScalar	m_lastHitFraction=0.;
	btVector3	m_lastHitNormalWorld;

	btAlignedObjectArray<btVector3>	m_hitNormalWorld;
	btAlignedObjectArray<btVector3>	m_hitPointWorld;
	btAlignedObjectArray<btScalar> m_hitFractions;

	virtual	btScalar	addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
	{
		m_collisionObject = rayResult.m_collisionObject;
		m_collisionObjects.push_back(rayResult.m_collisionObject);
		btVector3 hitNormalWorld;
		if (normalInWorldSpace)
		{
			hitNormalWorld = rayResult.m_hitNormalLocal;
		}
		else
		{
			///need to transform normal into worldspace
			hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
		}
		m_hitNormalWorld.push_back(hitNormalWorld);
		btVector3 hitPointWorld;
		hitPointWorld.setInterpolate3(m_rayFromWorld, m_rayToWorld, rayResult.m_hitFraction);
		m_hitPointWorld.push_back(hitPointWorld);
		m_hitFractions.push_back(rayResult.m_hitFraction);
		if (rayResult.m_hitFraction > m_lastHitFraction) {
			m_lastHitFraction = rayResult.m_hitFraction;
			m_lastHitNormalWorld = hitNormalWorld;
		}


		return m_closestHitFraction;
	}
};



MultiBodyTest::~MultiBodyTest() {
	m_guiHelper->removeAllGraphicsInstances();
	m_guiHelper->removeAllUserDebugItems();

}

btVector3 MultiBodyTest::anglesToVector(float a, float b) {
	return btVector3(cos(a)*cos(b), sin(a)*cos(b), sin(b));
	//creates vector based on alpha beta angle describe here https://stackoverflow.com/questions/30011741/3d-vector-defined-by-2-angles
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

		btBoxShape* colShape = createBoxShape(btVector3(.5f, .5f, .5f));


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


		addBoxes_testMultiDof(colShape);
	}
	

	#define CUBE_HALF_EXTENTS 1.f
	btCollisionShape* shape = new btBoxShape(btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));
	m_collisionShapes.push_back(shape);
	btHingeConstraint* spHingeDynAB = NULL;
	/*
	{

		// create a Hinge joint between two dynamic bodies
		// create two rigid bodies
		// static bodyA (parent) on top:
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(btScalar(-20.), btScalar(-2.), btScalar(0.)));
		btRigidBody* pBodyA = createRigidBody(1.0f, tr, shape);
		pBodyA->setActivationState(DISABLE_DEACTIVATION);
		// dynamic bodyB:
		tr.setIdentity();
		tr.setOrigin(btVector3(btScalar(-30.), btScalar(-2.), btScalar(0.)));
		btRigidBody* pBodyB = createRigidBody(10.0, tr, shape);
		pBodyB->setActivationState(DISABLE_DEACTIVATION);
		// add some data to build constraint frames
		btVector3 axisA(0.f, 1.f, 0.f);
		btVector3 axisB(0.f, 1.f, 0.f);
		btVector3 pivotA(-5.f, 0.f, 0.f);
		btVector3 pivotB(5.f, 0.f, 0.f);
		spHingeDynAB = new btHingeConstraint(*pBodyA, *pBodyB, pivotA, pivotB, axisA, axisB);
		spHingeDynAB->setLimit(-SIMD_HALF_PI * 0.5f, SIMD_HALF_PI * 0.5f);
		// add constraint to world
		m_dynamicsWorld->addConstraint(spHingeDynAB, true);
		// draw constraint frames and limits for debugging
		spHingeDynAB->setDbgDrawSize(btScalar(5.f));
	}
	*/

	/*
	{
		btVector3 linkHalfExtents(0.05, 0.37, 0.1);
		btVector3 baseHalfExtents(0.05, 0.37, 0.1);
		bool g_floatingBase = true;
		bool canSleep = true;
		bool selfCollide = true;
		bool gyro = true;
		bool damping = true;
		int numLinks = 1; //FUCK
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

	*/



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
	btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1]+ 1.f, 0);							//cur body's COM to cur body's PIV offset
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

	//pMultiBody->setupSpherical(4, linkMass, linkInertiaDiag, 2, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	//pMultiBody->setupSpherical(3, linkMass, linkInertiaDiag, 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	//pMultiBody->setupSpherical(2, linkMass, linkInertiaDiag, 3, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	//pMultiBody->setupSpherical(1, linkMass, linkInertiaDiag, 0, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
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

void MultiBodyTest::addBoxes_testMultiDof(btBoxShape* colShapeIn)
{
	//create a few dynamic rigidbodies
	// Re-using the same collision is better for memory usage and performance

	//btBoxShape* colShape = new btBoxShape(btVector3(1, 1, 1));
	//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
	btBoxShape* colShape = new btBoxShape(*colShapeIn);
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

void MultiBodyTest::castRays() {
	static float up = 0.f;
	static float dir = 1.f;

	//a  [0, 2π] and b  [−π/2, π/2]

	static float alpha=0.f;
	static float beta = 0.f;

	//alpha += .1f;
	//beta += .1f;
	float x = cos(alpha)*cos(beta);
	float z = sin(alpha)*cos(beta);
	float y = sin(beta);
	btVector3 angle_ray = btVector3(x, y, z);

	btVector3 A_normal;
	btVector3 A_up;
	btVector3 A_cross;

	btVector3 B_normal;
	btVector3 B_up;
	btVector3 B_cross;

	//add some simple animation

	up += 0.01*dir;
	if (btFabs(up)>2)
	{
		dir *= -1.f;
	}

	btTransform tr = m_dynamicsWorld->getCollisionObjectArray()[2]->getWorldTransform();

	static float angle = 0.f;
	angle += 0.01f;
	tr.setRotation(btQuaternion(btVector3(0, 1, 0), angle));
	m_dynamicsWorld->getCollisionObjectArray()[2]->setWorldTransform(tr);

	///step the simulation
	if (m_dynamicsWorld)
	{

		m_dynamicsWorld->updateAabbs();
		m_dynamicsWorld->computeOverlappingPairs();

		btVector3 red(1, 0, 0);
		btVector3 blue(0, 0, 1);

		//ray to object 2 hits
		//*******************************************
		btTransform tr_2 = m_dynamicsWorld->getCollisionObjectArray()[2]->getWorldTransform();
		btVector3 to(tr_2.getOrigin());
		//btVector3 from(-30, 1.2, 0);
		btVector3 from = angle_ray * 10;

		m_dynamicsWorld->getDebugDrawer()->drawLine(from, to, btVector4(0, 0, 0, 1)); // draw ray
		m_dynamicsWorld->getDebugDrawer()->drawLine(from, to, btVector4(0, 0, 1, 1));

		LastRayResultCallback	allResults(from, to);
		allResults.m_flags |= btTriangleRaycastCallback::kF_FilterBackfaces;

		m_dynamicsWorld->rayTest(from, to, allResults);


		btVector3 p = from.lerp(to, allResults.m_lastHitFraction);
		m_dynamicsWorld->getDebugDrawer()->drawSphere(p, 0.1, red);
		m_dynamicsWorld->getDebugDrawer()->drawLine(p, p + allResults.m_lastHitNormalWorld, red);

		A_up = btVector3(0, 0, 1).cross(btVector3(allResults.m_lastHitNormalWorld));
		btVector3 A_up_rounded = btVector3(floorf(abs(A_up.getX()) * 10) / 10, floorf(abs(A_up.getY()) * 10) / 10, floorf(abs(A_up.getZ()) * 10) / 10);
		if (A_up_rounded == btVector3(0, 0, 0))
			A_up = btVector3(0, 1, 0).cross(allResults.m_lastHitNormalWorld);
		btVector3 A_cross = A_up.cross(allResults.m_lastHitNormalWorld);
		if (A_up_rounded == btVector3(0, 0, 0))
			A_cross = A_cross*(-1.f);
		m_dynamicsWorld->getDebugDrawer()->drawLine(p, p + A_up * 10, red);
		m_dynamicsWorld->getDebugDrawer()->drawLine(p, p + A_cross * 10, red);

		A_normal = allResults.m_lastHitNormalWorld;
		//*******************************************



	}
}



/*
void MultiBodyTest::castRays()
{
	m_dynamicsWorld->getCollisionObjectArray()[1]->setUserIndex(1);
	m_dynamicsWorld->getCollisionObjectArray()[2]->setUserIndex(2);
	static float up = 0.f;
	static float dir = 1.f;

	btVector3 A_normal;
	btVector3 A_up;
	btVector3 A_cross;
		
	btVector3 B_normal;
	btVector3 B_up;
	btVector3 B_cross;
	//add some simple animation
	//if (!m_idle)
	{
		up += 0.01*dir;

		if (btFabs(up)>2)
		{
			dir *= -1.f;
		}

		btTransform tr = m_dynamicsWorld->getCollisionObjectArray()[2]->getWorldTransform();
		
		static float angle = 0.f;
		angle += 0.01f;
		tr.setRotation(btQuaternion(btVector3(0, 1, 0), angle));
		m_dynamicsWorld->getCollisionObjectArray()[2]->setWorldTransform(tr);
	}


	///step the simulation
	if (m_dynamicsWorld)
	{

		m_dynamicsWorld->updateAabbs();
		m_dynamicsWorld->computeOverlappingPairs();

		btVector3 red(1, 0, 0);
		btVector3 blue(0, 0, 1);

		//ray to object 2 hits
		{
			btTransform tr_2 = m_dynamicsWorld->getCollisionObjectArray()[2]->getWorldTransform();
			btVector3 to(tr_2.getOrigin());
			btVector3 from(-30, 1.2, 0);
			//btVector3 from(30, 1.2, 0 + up);

			m_dynamicsWorld->getDebugDrawer()->drawLine(from, to, btVector4(0, 0, 0, 1));
			m_dynamicsWorld->getDebugDrawer()->drawLine(from, to, btVector4(0, 0, 1, 1));

			//btCollisionWorld::ClosestRayResultCallback	closestResults(from, to);
			LastRayResultCallback	allResults(from, to);
			//allResults.m_flags |= btTriangleRaycastCallback::kF_FilterBackfaces;

			m_dynamicsWorld->rayTest(from, to, allResults);


			btVector3 p = from.lerp(to, allResults.m_lastHitFraction);
			m_dynamicsWorld->getDebugDrawer()->drawSphere(p, 0.1, red);
			m_dynamicsWorld->getDebugDrawer()->drawLine(p, p + allResults.m_lastHitNormalWorld, red);
			
			btVector3 up = btVector3(0, 0, 1).cross(btVector3(allResults.m_lastHitNormalWorld));
			btVector3 up_rounded = btVector3(floorf(abs(up.getX()) * 10) / 10, floorf(abs(up.getY()) * 10) / 10, floorf(abs(up.getZ()) * 10) / 10);
			if (up_rounded == btVector3(0, 0, 0))
				up = btVector3(0, 1, 0).cross(allResults.m_lastHitNormalWorld);
			btVector3 cross = up.cross(allResults.m_lastHitNormalWorld);
			if (up_rounded == btVector3(0, 0, 0))
				cross = cross*(-1.f);
			m_dynamicsWorld->getDebugDrawer()->drawLine(p, p + up * 10, red);
			m_dynamicsWorld->getDebugDrawer()->drawLine(p, p + cross * 10, red);

			B_normal = allResults.m_lastHitNormalWorld;
			B_up = up;
			B_cross = cross;
		}

		///ray test to object 1
		{
			btTransform tr = m_dynamicsWorld->getCollisionObjectArray()[1]->getWorldTransform();
			//tr.setOrigin(tr.getOrigin() + btVector3(0, 0, 0));
			//m_dynamicsWorld->getCollisionObjectArray()[1]->setWorldTransform(tr);

			//btTransform tr_2 = m_dynamicsWorld->getCollisionObjectArray()[2]->getWorldTransform();
			//static float angle = 0.f;
			//angle += 0.01f;
			//tr_2.setRotation(btQuaternion(btVector3(0, 1, 0), angle));
			//m_dynamicsWorld->getCollisionObjectArray()[2]->setWorldTransform(tr_2);
			
			
			btVector3 to(tr.getOrigin());
			//btVector3 from(-30, 1.2, 0);
			btVector3 from(30, 1.2, 0 + up);
			m_dynamicsWorld->getDebugDrawer()->drawLine(from, to, btVector4(0, 0, 1, 1));

			//btCollisionWorld::ClosestRayResultCallback	closestResults(from, to);
			LastRayResultCallback	closestResults(from, to);
			closestResults.m_flags |= btTriangleRaycastCallback::kF_FilterBackfaces;


			m_dynamicsWorld->rayTest(from, to, closestResults);

			if (closestResults.hasHit())
			{
			//std::cout << "Ray Hit";
				btVector3 p = from.lerp(to, closestResults.m_lastHitFraction);
				m_dynamicsWorld->getDebugDrawer()->drawSphere(p, 0.1, red);
				m_dynamicsWorld->getDebugDrawer()->drawLine(p, p + closestResults.m_lastHitNormalWorld, red);
				btVector3 u = btVector3(0, 0, 1).cross(closestResults.m_lastHitNormalWorld);
				btVector3 u_rounded = btVector3(floorf(abs(u.getX()) * 10) / 10, floorf(abs(u.getY()) * 10) / 10, floorf(abs(u.getZ()) * 10) / 10);
				if(u_rounded == btVector3(0, 0, 0))
					u = btVector3(0, 1, 0).cross(closestResults.m_lastHitNormalWorld);
				//std::cout << "(" << floorf(crossed.getX() * 10) / 10 << "|" << floorf(abs(crossed.getY()) * 10) / 10 << "|" << floorf(crossed.getZ() * 10) / 10 << ")";

				//crossed = btVector3(0, 1, 0).cross(closestResults.m_hitNormalWorld);
				btVector3 v = u.cross(closestResults.m_lastHitNormalWorld);
				if (u_rounded == btVector3(0, 0, 0))
					v=v*(-1.f);
				m_dynamicsWorld->getDebugDrawer()->drawLine(p, p + u*10, red);
				m_dynamicsWorld->getDebugDrawer()->drawLine(p, p + v*10, red);
				
				//A_normal = closestResults.m_lastHitNormalWorld;
				//A_up = u;
				//A_cross = v;
			}
		}
	}

}
*/



void MultiBodyTest::stepSimulation(float deltaTime)
{
	castRays();
	TimeWarpBaseMultiBody::stepSimulation(deltaTime);
}

CommonExampleInterface*    BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new MultiBodyTest(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)



