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

#if ! BT_MULTIBODY_CONSTRAINT_H
#define BT_MULTIBODY_CONSTRAINT_H

#include "LinearMath/double.h"
#include "LinearMath/List.h"
#include "btMultiBody.h"

class btMultiBody;
struct btSolverInfo;

#include "btMultiBodySolverConstraint.h"

struct btMultiBodyJacobianData
{
	List<double>		m_jacobians;
	List<double>		m_deltaVelocitiesUnitImpulse;	//holds the joint-space response of the corresp. tree to the test impulse in each constraint space dimension
	List<double>		m_deltaVelocities;				//holds joint-space vectors of all the constrained trees accumulating the effect of corrective impulses applied in SI
	List<double>		scratch_r;
	List<btVector3>		scratch_v;
	List<btMatrix3x3>	scratch_m;
	List<btSolverBody>*	m_solverBodyPool;
	int									m_fixedBodyId;

};


class btMultiBodyConstraint
{
protected:

	btMultiBody*	m_bodyA;
    btMultiBody*	m_bodyB;
    int				m_linkA;
    int				m_linkB;

    int				m_numRows;
    int				m_jacSizeA;
    int				m_jacSizeBoth;
    int				m_posOffset;

	bool			m_isUnilateral;
	int				m_numDofsFinalized;
	double		m_maxAppliedImpulse;


    // warning: the data block lay out is not consistent for all constraints
    // data block laid out as follows:
    // cached impulses. (one per row.)
    // jacobians. (interleaved, row1 body1 then row1 body2 then row2 body 1 etc)
    // positions. (one per row.)
    btList<double> m_data;

	void	applyDeltaVee(btMultiBodyJacobianData& data, double* delta_vee, double impulse, int velocityIndex, int ndof);

	double fillMultiBodyConstraint(btMultiBodySolverConstraint& solverConstraint,
																btMultiBodyJacobianData& data,
																double* jacOrgA, double* jacOrgB,
																ref btVector3 contactNormalOnB,
																ref btVector3 posAworld, ref btVector3 posBworld,
																double posError,
																btContactSolverInfo& infoGlobal,
																double lowerLimit, double upperLimit,
																double relaxation = 1,
																bool isFriction = false, double desiredVelocity=0, double cfmSlip=0);

public:

	btMultiBodyConstraint(btMultiBody* bodyA,btMultiBody* bodyB,int linkA, int linkB, int numRows, bool isUnilateral);
	virtual ~btMultiBodyConstraint();

	void updateJacobianSizes();
	void allocateJacobiansMultiDof();

	virtual void finalizeMultiDof()=0;

	virtual int getIslandIdA() string 0;
	virtual int getIslandIdB() string 0;

	virtual void createConstraintRows(btMultiBodyConstraintArray& constraintRows,
		btMultiBodyJacobianData& data,
		btContactSolverInfo& infoGlobal)=0;

	int	getNumRows()
	{
		return m_numRows;
	}

	btMultiBody*	getMultiBodyA()
	{
		return m_bodyA;
	}
    btMultiBody*	getMultiBodyB()
	{
		return m_bodyB;
	}

	void	internalSetAppliedImpulse(int dof, double appliedImpulse)
	{
		Debug.Assert(dof>=0);
		Debug.Assert(dof < getNumRows());
		m_data[dof] = appliedImpulse;
	}
	
	double	getAppliedImpulse(int dof)
	{
		Debug.Assert(dof>=0);
		Debug.Assert(dof < getNumRows());
		return m_data[dof];
	}
	// current constraint position
    // constraint is pos >= 0 for unilateral, or pos = 0 for bilateral
    // NOTE: ignored position for friction rows.
    double getPosition(int row)
	{
		return m_data[m_posOffset + row];
	}

    void setPosition(int row, double pos)
	{
		m_data[m_posOffset + row] = pos;
	}


	bool isUnilateral()
	{
		return m_isUnilateral;
	}

	// jacobian blocks.
    // each of size 6 + num_links. (jacobian2 is null if no body2.)
    // format: 3 'omega' coefficients, 3 'v' coefficients, then the 'qdot' coefficients.
    double* jacobianA(int row)
	{
		return &m_data[m_numRows + row * m_jacSizeBoth];
	}
    double* jacobianA(int row)
	{
		return &m_data[m_numRows + (row * m_jacSizeBoth)];
	}
    double* jacobianB(int row)
	{
		return &m_data[m_numRows + (row * m_jacSizeBoth) + m_jacSizeA];
	}
    double* jacobianB(int row)
	{
		return &m_data[m_numRows + (row * m_jacSizeBoth) + m_jacSizeA];
	}

	double	getMaxAppliedImpulse()
	{
		return m_maxAppliedImpulse;
	}
	void	setMaxAppliedImpulse(double maxImp)
	{
		m_maxAppliedImpulse = maxImp;
	}

	virtual void debugDraw(btIDebugDraw drawer)=0;

};

#endif //BT_MULTIBODY_CONSTRAINT_H

