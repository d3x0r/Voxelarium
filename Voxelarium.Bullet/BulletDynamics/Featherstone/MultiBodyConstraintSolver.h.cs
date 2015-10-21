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

#if ! BT_MULTIBODY_CONSTRAINT_SOLVER_H
#define BT_MULTIBODY_CONSTRAINT_SOLVER_H

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "btMultiBodySolverConstraint.h"

#define DIRECTLY_UPDATE_VELOCITY_DURING_SOLVER_ITERATIONS

class btMultiBody;

#include "btMultiBodyConstraint.h"



internal class btMultiBodyConstraintSolver : btSequentialImpulseConstraintSolver
{

protected:

	btMultiBodyConstraintArray			m_multiBodyNonContactConstraints;

	btMultiBodyConstraintArray			m_multiBodyNormalContactConstraints;
	btMultiBodyConstraintArray			m_multiBodyFrictionContactConstraints;

	btMultiBodyJacobianData				m_data;
	
	//temp storage for multi body constraints for a specific island/group called by 'solveGroup'
	btMultiBodyConstraint[]					m_tmpMultiBodyConstraints;
	int										m_tmpNumMultiBodyConstraints;

	void resolveSingleConstraintRowGeneric(btMultiBodySolverConstraint& c);
	void resolveSingleConstraintRowGenericMultiBody(btMultiBodySolverConstraint& c);

	void convertContacts(btPersistentManifold[] manifoldPtr,int numManifolds, btContactSolverInfo& infoGlobal);
	btMultiBodySolverConstraint&	addMultiBodyFrictionConstraint(ref btVector3 normalAxis,btPersistentManifold* manifold,int frictionIndex,btManifoldPoint cp,btCollisionObject colObj0,btCollisionObject colObj1, double relaxation, btContactSolverInfo& infoGlobal, double desiredVelocity=0, double cfmSlip=0);


	void setupMultiBodyJointLimitConstraint(btMultiBodySolverConstraint& constraintRow, 
																 double* jacA,double* jacB,
																 double penetration,double combinedFrictionCoeff, double combinedRestitutionCoeff,
																 btContactSolverInfo& infoGlobal);

	void setupMultiBodyContactConstraint(btMultiBodySolverConstraint& solverConstraint, 
																 ref btVector3 contactNormal,
																 btManifoldPoint cp, btContactSolverInfo& infoGlobal,
																 double relaxation,
																 bool isFriction, double desiredVelocity=0, double cfmSlip=0);

	void convertMultiBodyContact(btPersistentManifold* manifold,btContactSolverInfo& infoGlobal);
	virtual double solveGroupCacheFriendlySetup(btCollisionObject bodies,int numBodies,btPersistentManifold[] manifoldPtr, int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer);
//	virtual double solveGroupCacheFriendlyIterations(btCollisionObject bodies,int numBodies,btPersistentManifold[] manifoldPtr, int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer);

	virtual double solveSingleIteration(int iteration, btCollisionObject bodies ,int numBodies,btPersistentManifold[] manifoldPtr, int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer);
	void	applyDeltaVee(double* deltaV, double impulse, int velocityIndex, int ndof);
	void writeBackSolverBodyToMultiBody(btMultiBodySolverConstraint& constraint, double deltaTime);
public:

	

	///this method should not be called, it was just used during porting/integration of Featherstone btMultiBody, providing backwards compatibility but no support for btMultiBodyConstraint (only contact constraints)
	virtual double solveGroup(btCollisionObject bodies,int numBodies,btPersistentManifold[] manifold,int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& info, btIDebugDraw* debugDrawer,btDispatcher* dispatcher);
	virtual double solveGroupCacheFriendlyFinish(btCollisionObject bodies,int numBodies,btContactSolverInfo& infoGlobal);
	
	virtual void solveMultiBodyGroup(btCollisionObject bodies,int numBodies,btPersistentManifold[] manifold,int numManifolds,btTypedConstraint[] constraints,int numConstraints,btMultiBodyConstraint[] multiBodyConstraints, int numMultiBodyConstraints, btContactSolverInfo& info, btIDebugDraw* debugDrawer,btDispatcher* dispatcher);
};

	
	


#endif //BT_MULTIBODY_CONSTRAINT_SOLVER_H

