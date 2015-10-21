/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///original version written by Erwin Coumans, October 2013

#if ! BT_MLCP_SOLVER_H
#define BT_MLCP_SOLVER_H

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "LinearMath/btMatrixX.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolverInterface.h"

class btMLCPSolver : btSequentialImpulseConstraintSolver
{

protected:
	
	btMatrixXu m_A;
	btVectorXu m_b;
	btVectorXu m_x;
	btVectorXu m_lo;
	btVectorXu m_hi;
	
	///when using 'split impulse' we solve two separate (M)LCPs
	btVectorXu m_bSplit;
	btVectorXu m_xSplit;
	btVectorXu m_bSplit1;
	btVectorXu m_xSplit2;

	List<int> m_limitDependencies;
	List<btSolverConstraint*>	m_allConstraintPtrArray;
	btMLCPSolverInterface* m_solver;
	int m_fallback;
	double m_cfm;

	virtual double solveGroupCacheFriendlySetup(btCollisionObject bodies, int numBodies, btPersistentManifold[] manifoldPtr, int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer);
	virtual double solveGroupCacheFriendlyIterations(btCollisionObject bodies ,int numBodies,btPersistentManifold[] manifoldPtr, int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer);


	virtual void createMLCP(btContactSolverInfo& infoGlobal);
	virtual void createMLCPFast(btContactSolverInfo& infoGlobal);

	//return true is it solves the problem successfully
	virtual bool solveMLCP(btContactSolverInfo& infoGlobal);

public:

	btMLCPSolver(	 btMLCPSolverInterface* solver);
	virtual ~btMLCPSolver();

	void setMLCPSolver(btMLCPSolverInterface* solver)
	{
		m_solver = solver;
	}

	int getNumFallbacks()
	{
		return m_fallback;
	}
	void setNumFallbacks(int num)
	{
		m_fallback = num;
	}

	double	getCfm()
	{
		return m_cfm;
	}
	void setCfm(double cfm)
	{
		m_cfm = cfm;
	}

	virtual btConstraintSolverType	getSolverType()
	{
		return BT_MLCP_SOLVER;
	}

};


#endif //BT_MLCP_SOLVER_H
