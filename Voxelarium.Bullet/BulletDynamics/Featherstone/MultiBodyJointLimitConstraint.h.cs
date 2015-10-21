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

#if ! BT_MULTIBODY_JOINT_LIMIT_CONSTRAINT_H
#define BT_MULTIBODY_JOINT_LIMIT_CONSTRAINT_H

#include "btMultiBodyConstraint.h"
struct btSolverInfo;

class btMultiBodyJointLimitConstraint : btMultiBodyConstraint
{
protected:

	double	m_lowerBound;
	double	m_upperBound;
public:

	btMultiBodyJointLimitConstraint(btMultiBody* body, int link, double lower, double upper);
	virtual ~btMultiBodyJointLimitConstraint();

	virtual void finalizeMultiDof();

	virtual int getIslandIdA();
	virtual int getIslandIdB();

	virtual void createConstraintRows(btMultiBodyConstraintArray& constraintRows,
		btMultiBodyJacobianData& data,
		btContactSolverInfo& infoGlobal);

	virtual void debugDraw(btIDebugDraw drawer)
	{
		//todo(erwincoumans)
	}

};

#endif //BT_MULTIBODY_JOINT_LIMIT_CONSTRAINT_H

