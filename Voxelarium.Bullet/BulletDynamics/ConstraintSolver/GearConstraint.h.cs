/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2012 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#if ! BT_GEAR_CONSTRAINT_H
#define BT_GEAR_CONSTRAINT_H

#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"


#if BT_USE_DOUBLE_PRECISION
#define btGearConstraintData	btGearConstraintDoubleData
#define btGearConstraintDataName	"btGearConstraintDoubleData"
#else
#define btGearConstraintData	btGearConstraintFloatData
#define btGearConstraintDataName	"btGearConstraintFloatData"
#endif //BT_USE_DOUBLE_PRECISION



///The btGeatConstraint will couple the angular velocity for two bodies around given local axis and ratio.
///See Bullet/Demos/ConstraintDemo for an example use.
class btGearConstraint : btTypedConstraint
{
protected:
	btVector3	m_axisInA;
	btVector3	m_axisInB;
	bool		m_useFrameA;
	double	m_ratio;

public:
	btGearConstraint(btRigidBody rbA, btRigidBody rbB, ref btVector3 axisInA,ref btVector3 axisInB, double ratio=1);
	virtual ~btGearConstraint ();

	///internal method used by the constraint solver, don't use them directly
	virtual void getInfo1 (btConstraintInfo1* info);

	///internal method used by the constraint solver, don't use them directly
	virtual void getInfo2 (btConstraintInfo2* info);

	void setAxisA(ref btVector3 axisA) 
	{
		m_axisInA = axisA;
	}
	void setAxisB(ref btVector3 axisB)
	{
		m_axisInB = axisB;
	}
	void setRatio(double ratio)
	{
		m_ratio = ratio;
	}
	ref btVector3 getAxisA()
	{
		return m_axisInA;
	}
	ref btVector3 getAxisB()
	{
		return m_axisInB;
	}
	double getRatio()
	{
		return m_ratio;
	}


	virtual	void	setParam(int num, double value, int axis = -1) 
	{
		(void) num;
		(void) value;
		(void) axis;
		Debug.Assert(false);
	}

	///return the local value of parameter
	virtual	double getParam(int num, int axis = -1) string 	{ 
		(void) num;
		(void) axis;
		Debug.Assert(false);
		return 0;
	}

	virtual	int	calculateSerializeBufferSize();

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer, btSerializer* serializer);
};




///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btGearConstraintFloatData
{
	btTypedConstraintFloatData	m_typeConstraintData;

	btVector3FloatData			m_axisInA;
	btVector3FloatData			m_axisInB;

	float							m_ratio;
	char							m_padding[4];
};

struct btGearConstraintDoubleData
{
	btTypedConstraintDoubleData	m_typeConstraintData;

	btVector3DoubleData			m_axisInA;
	btVector3DoubleData			m_axisInB;

	double						m_ratio;
};

public	int	btGearConstraint::calculateSerializeBufferSize()
{
	return sizeof(btGearConstraintData);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
public	string	btGearConstraint::serialize(object dataBuffer, btSerializer* serializer)
{
	btGearConstraintData* gear = (btGearConstraintData*)dataBuffer;
	btTypedConstraint::serialize(&gear.m_typeConstraintData,serializer);

	m_axisInA.serialize( gear.m_axisInA );
	m_axisInB.serialize( gear.m_axisInB );

	gear.m_ratio = m_ratio;

	return btGearConstraintDataName;
}






#endif //BT_GEAR_CONSTRAINT_H
