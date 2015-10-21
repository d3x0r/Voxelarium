/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///This file was written by Erwin Coumans

#if ! BT_MULTIBODY_POINT2POINT_H
#define BT_MULTIBODY_POINT2POINT_H

#include "btMultiBodyConstraint.h"

//#define BTMBP2PCONSTRAINT_BLOCK_ANGULAR_MOTION_TEST

class btMultiBodyPoint2Point : btMultiBodyConstraint
{
protected:

	btRigidBody	m_rigidBodyA;
	btRigidBody	m_rigidBodyB;
	btVector3		m_pivotInA;
	btVector3		m_pivotInB;


public:

	btMultiBodyPoint2Point(btMultiBody* body, int link, btRigidBody bodyB, ref btVector3 pivotInA, ref btVector3 pivotInB);
	btMultiBodyPoint2Point(btMultiBody* bodyA, int linkA, btMultiBody* bodyB, int linkB, ref btVector3 pivotInA, ref btVector3 pivotInB);

	virtual ~btMultiBodyPoint2Point();

	virtual void finalizeMultiDof();

	virtual int getIslandIdA();
	virtual int getIslandIdB();

	virtual void createConstraintRows(btMultiBodyConstraintArray& constraintRows,
		btMultiBodyJacobianData& data,
		btContactSolverInfo& infoGlobal);

	ref btVector3 getPivotInB()
	{
		return m_pivotInB;
	}

	void setPivotInB(ref btVector3 pivotInB)
	{
		m_pivotInB = pivotInB;
	}

	virtual void debugDraw(btIDebugDraw drawer);

};

#endif //BT_MULTIBODY_POINT2POINT_H
