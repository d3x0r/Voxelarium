/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */

namespace Bullet.Dynamics.ConstraintSolver
{
/*
#if BT_USE_DOUBLE_PRECISION
#define btHingeConstraintData	btHingeConstraintDoubleData2 //rename to 2 for backwards compatibility, so we can still load the 'btHingeConstraintDoubleData' version
#define btHingeConstraintDataName	"btHingeConstraintDoubleData2" 
#else
#define btHingeConstraintData	btHingeConstraintFloatData
#define btHingeConstraintDataName	"btHingeConstraintFloatData"
#endif //BT_USE_DOUBLE_PRECISION
*/


enum btHingeFlags
{
	BT_HINGE_FLAGS_CFM_STOP = 1,
	BT_HINGE_FLAGS_ERP_STOP = 2,
	BT_HINGE_FLAGS_CFM_NORM = 4,
	BT_HINGE_FLAGS_ERP_NORM = 8
};


/// hinge constraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/// axis defines the orientation of the hinge axis
internal class btHingeConstraint : btTypedConstraint
{
#if IN_PARALLELL_SOLVER
public:
#endif
	btJacobianEntry	m_jac[3]; //3 orthogonal linear constraints
	btJacobianEntry	m_jacAng[3]; //2 orthogonal angular constraints+ 1 for limit/motor

	btTransform m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransform m_rbBFrame;

	double	m_motorTargetVelocity;
	double	m_maxMotorImpulse;


#if	_BT_USE_CENTER_LIMIT_
	btAngularLimit	m_limit;
#else
	double	m_lowerLimit;	
	double	m_upperLimit;	
	double	m_limitSign;
	double	m_correction;

	double	m_limitSoftness; 
	double	m_biasFactor; 
	double	m_relaxationFactor; 

	bool		m_solveLimit;
#endif

	double	m_kHinge;


	double	m_accLimitImpulse;
	double	m_hingeAngle;
	double	m_referenceSign;

	bool		m_angularOnly;
	bool		m_enableAngularMotor;
	bool		m_useSolveConstraintObsolete;
	bool		m_useOffsetForConstraintFrame;
	bool		m_useReferenceFrameA;

	double	m_accMotorImpulse;

	int			m_flags;
	double	m_normalCFM;
	double	m_normalERP;
	double	m_stopCFM;
	double	m_stopERP;

	
public:

	
	
	btHingeConstraint(btRigidBody rbA,btRigidBody rbB, ref btVector3 pivotInA,ref btVector3 pivotInB, ref btVector3 axisInA,ref btVector3 axisInB, bool useReferenceFrameA = false);

	btHingeConstraint(btRigidBody rbA,ref btVector3 pivotInA,ref btVector3 axisInA, bool useReferenceFrameA = false);
	
	btHingeConstraint(btRigidBody rbA,btRigidBody rbB, ref btTransform rbAFrame, ref btTransform rbBFrame, bool useReferenceFrameA = false);

	btHingeConstraint(btRigidBody rbA,ref btTransform rbAFrame, bool useReferenceFrameA = false);


	virtual void	buildJacobian();

	virtual void getInfo1 (btConstraintInfo1* info);

	void getInfo1NonVirtual(btConstraintInfo1* info);

	virtual void getInfo2 (btConstraintInfo2* info);

	void	getInfo2NonVirtual(btConstraintInfo2* info,ref btTransform transA,ref btTransform transB,ref btVector3 angVelA,ref btVector3 angVelB);

	void	getInfo2Internal(btConstraintInfo2* info,ref btTransform transA,ref btTransform transB,ref btVector3 angVelA,ref btVector3 angVelB);
	void	getInfo2InternalUsingFrameOffset(btConstraintInfo2* info,ref btTransform transA,ref btTransform transB,ref btVector3 angVelA,ref btVector3 angVelB);
		

	void	updateRHS(double	timeStep);

	btRigidBody getRigidBodyA()
	{
		return m_rbA;
	}
	btRigidBody getRigidBodyB()
	{
		return m_rbB;
	}

	btRigidBody getRigidBodyA()	
	{		
		return m_rbA;	
	}	

	btRigidBody getRigidBodyB()	
	{		
		return m_rbB;	
	}

	ref btTransform getFrameOffsetA()
	{
	return m_rbAFrame;
	}

	ref btTransform getFrameOffsetB()
	{
		return m_rbBFrame;
	}

	void setFrames(ref btTransform frameA, ref btTransform frameB);
	
	void	setAngularOnly(bool angularOnly)
	{
		m_angularOnly = angularOnly;
	}

	void	enableAngularMotor(bool enableMotor,double targetVelocity,double maxMotorImpulse)
	{
		m_enableAngularMotor  = enableMotor;
		m_motorTargetVelocity = targetVelocity;
		m_maxMotorImpulse = maxMotorImpulse;
	}

	// extra motor API, including ability to set a target rotation (as opposed to angular velocity)
	// note: setMotorTarget sets angular velocity under the hood, so you must call it every tick to
	//       maintain a given angular target.
	void enableMotor(bool enableMotor) 	{ m_enableAngularMotor = enableMotor; }
	void setMaxMotorImpulse(double maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; }
	void setMotorTarget(btQuaternion& qAinB, double dt); // qAinB is rotation of body A wrt body B.
	void setMotorTarget(double targetAngle, double dt);


	void	setLimit(double low,double high,double _softness = 0.9f, double _biasFactor = 0.3f, double _relaxationFactor = 1.0f)
	{
#if	_BT_USE_CENTER_LIMIT_
		m_limit.set(low, high, _softness, _biasFactor, _relaxationFactor);
#else
		m_lowerLimit = btNormalizeAngle(low);
		m_upperLimit = btNormalizeAngle(high);
		m_limitSoftness =  _softness;
		m_biasFactor = _biasFactor;
		m_relaxationFactor = _relaxationFactor;
#endif
	}

	void	setAxis(ref btVector3 axisInA)
	{
		btVector3 rbAxisA1, rbAxisA2;
		btPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);
		btVector3 pivotInA = m_rbAFrame.getOrigin();
//		m_rbAFrame.getOrigin() = pivotInA;
		m_rbAFrame.getBasis().setValue( rbAxisA1.x,rbAxisA2.x,axisInA.x,
										rbAxisA1.y,rbAxisA2.y,axisInA.y,
										rbAxisA1.z,rbAxisA2.z,axisInA.z );

		btVector3 axisInB = m_rbA.getCenterOfMassTransform().getBasis() * axisInA;

		btQuaternion rotationArc = shortestArcQuat(axisInA,axisInB);
		btVector3 rbAxisB1 =  quatRotate(rotationArc,rbAxisA1);
		btVector3 rbAxisB2 = axisInB.cross(rbAxisB1);

		m_rbBFrame.getOrigin() = m_rbB.getCenterOfMassTransform().inverse()(m_rbA.getCenterOfMassTransform()(pivotInA));

		m_rbBFrame.getBasis().setValue( rbAxisB1.x,rbAxisB2.x,axisInB.x,
										rbAxisB1.y,rbAxisB2.y,axisInB.y,
										rbAxisB1.z,rbAxisB2.z,axisInB.z );
		m_rbBFrame.getBasis() = m_rbB.getCenterOfMassTransform().getBasis().inverse() * m_rbBFrame.getBasis();

	}

    bool hasLimit() string 
#if  _BT_USE_CENTER_LIMIT_
        return m_limit.getHalfRange() > 0;
#else
        return m_lowerLimit <= m_upperLimit;
#endif
    }

	double	getLowerLimit()
	{
#if	_BT_USE_CENTER_LIMIT_
	return m_limit.getLow();
#else
	return m_lowerLimit;
#endif
	}

	double	getUpperLimit()
	{
#if	_BT_USE_CENTER_LIMIT_
	return m_limit.getHigh();
#else		
	return m_upperLimit;
#endif
	}


	///The getHingeAngle gives the hinge angle in range [-PI,PI]
	double getHingeAngle();

	double getHingeAngle(ref btTransform transA,ref btTransform transB);

	void testLimit(ref btTransform transA,ref btTransform transB);


	ref btTransform getAFrame() string  return m_rbAFrame; };	
	ref btTransform getBFrame() string  return m_rbBFrame; };

	ref btTransform getAFrame() { return m_rbAFrame; };	
	ref btTransform getBFrame() { return m_rbBFrame; };

	inline int getSolveLimit()
	{
#if	_BT_USE_CENTER_LIMIT_
	return m_limit.isLimit();
#else
	return m_solveLimit;
#endif
	}

	inline double getLimitSign()
	{
#if	_BT_USE_CENTER_LIMIT_
	return m_limit.getSign();
#else
		return m_limitSign;
#endif
	}

	inline bool getAngularOnly() 
	{ 
		return m_angularOnly; 
	}
	inline bool getEnableAngularMotor() 
	{ 
		return m_enableAngularMotor; 
	}
	inline double getMotorTargetVelosity() 
	{ 
		return m_motorTargetVelocity; 
	}
	inline double getMaxMotorImpulse() 
	{ 
		return m_maxMotorImpulse; 
	}
	// access for UseFrameOffset
	bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
	void setUseFrameOffset(bool frameOffsetOnOff) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }


	///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
	///If no axis is provided, it uses the default axis for this constraint.
	virtual	void	setParam(int num, double value, int axis = -1);
	///return the local value of parameter
	virtual	double getParam(int num, int axis = -1);

	virtual	int	calculateSerializeBufferSize();

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer, btSerializer* serializer);


};


//only for backward compatibility
#if BT_BACKWARDS_COMPATIBLE_SERIALIZATION
///this structure is not used, except for loading pre-2.82 .bullet files
struct	btHingeConstraintDoubleData
{
	btTypedConstraintData	m_typeConstraintData;
	btTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformDoubleData m_rbBFrame;
	int			m_useReferenceFrameA;
	int			m_angularOnly;
	int			m_enableAngularMotor;
	float	m_motorTargetVelocity;
	float	m_maxMotorImpulse;

	float	m_lowerLimit;
	float	m_upperLimit;
	float	m_limitSoftness;
	float	m_biasFactor;
	float	m_relaxationFactor;

};
#endif //BT_BACKWARDS_COMPATIBLE_SERIALIZATION

///The getAccumulatedHingeAngle returns the accumulated hinge angle, taking rotation across the -PI/PI boundary into account
internal class btHingeAccumulatedAngleConstraint : btHingeConstraint
{
protected:
	double	m_accumulatedAngle;
public:

	
	
	btHingeAccumulatedAngleConstraint(btRigidBody rbA,btRigidBody rbB, ref btVector3 pivotInA,ref btVector3 pivotInB, ref btVector3 axisInA,ref btVector3 axisInB, bool useReferenceFrameA = false)
	:btHingeConstraint(rbA,rbB,pivotInA,pivotInB, axisInA,axisInB, useReferenceFrameA )
	{
		m_accumulatedAngle=getHingeAngle();
	}

	btHingeAccumulatedAngleConstraint(btRigidBody rbA,ref btVector3 pivotInA,ref btVector3 axisInA, bool useReferenceFrameA = false)
	:btHingeConstraint(rbA,pivotInA,axisInA, useReferenceFrameA)
	{
		m_accumulatedAngle=getHingeAngle();
	}
	
	btHingeAccumulatedAngleConstraint(btRigidBody rbA,btRigidBody rbB, ref btTransform rbAFrame, ref btTransform rbBFrame, bool useReferenceFrameA = false)
	:btHingeConstraint(rbA,rbB, rbAFrame, rbBFrame, useReferenceFrameA )
	{
		m_accumulatedAngle=getHingeAngle();
	}

	btHingeAccumulatedAngleConstraint(btRigidBody rbA,ref btTransform rbAFrame, bool useReferenceFrameA = false)
	:btHingeConstraint(rbA,rbAFrame, useReferenceFrameA )
	{
		m_accumulatedAngle=getHingeAngle();
	}
	double getAccumulatedHingeAngle();
	void	setAccumulatedHingeAngle(double accAngle);
	virtual void getInfo1 (btConstraintInfo1* info);

};

struct	btHingeConstraintFloatData
{
	btTypedConstraintData	m_typeConstraintData;
	btTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformFloatData m_rbBFrame;
	int			m_useReferenceFrameA;
	int			m_angularOnly;
	
	int			m_enableAngularMotor;
	float	m_motorTargetVelocity;
	float	m_maxMotorImpulse;

	float	m_lowerLimit;
	float	m_upperLimit;
	float	m_limitSoftness;
	float	m_biasFactor;
	float	m_relaxationFactor;

};



///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btHingeConstraintDoubleData2
{
	btTypedConstraintDoubleData	m_typeConstraintData;
	btTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformDoubleData m_rbBFrame;
	int			m_useReferenceFrameA;
	int			m_angularOnly;
	int			m_enableAngularMotor;
	double		m_motorTargetVelocity;
	double		m_maxMotorImpulse;

	double		m_lowerLimit;
	double		m_upperLimit;
	double		m_limitSoftness;
	double		m_biasFactor;
	double		m_relaxationFactor;
	char	m_padding1[4];

};




public	int	btHingeConstraint::calculateSerializeBufferSize()
{
	return sizeof(btHingeConstraintData);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
public	string	btHingeConstraint::serialize(object dataBuffer, btSerializer* serializer)
{
	btHingeConstraintData* hingeData = (btHingeConstraintData*)dataBuffer;
	btTypedConstraint::serialize(&hingeData.m_typeConstraintData,serializer);

	m_rbAFrame.serialize(hingeData.m_rbAFrame);
	m_rbBFrame.serialize(hingeData.m_rbBFrame);

	hingeData.m_angularOnly = m_angularOnly;
	hingeData.m_enableAngularMotor = m_enableAngularMotor;
	hingeData.m_maxMotorImpulse = float(m_maxMotorImpulse);
	hingeData.m_motorTargetVelocity = float(m_motorTargetVelocity);
	hingeData.m_useReferenceFrameA = m_useReferenceFrameA;
#if	_BT_USE_CENTER_LIMIT_
	hingeData.m_lowerLimit = float(m_limit.getLow());
	hingeData.m_upperLimit = float(m_limit.getHigh());
	hingeData.m_limitSoftness = float(m_limit.getSoftness());
	hingeData.m_biasFactor = float(m_limit.getBiasFactor());
	hingeData.m_relaxationFactor = float(m_limit.getRelaxationFactor());
#else
	hingeData.m_lowerLimit = float(m_lowerLimit);
	hingeData.m_upperLimit = float(m_upperLimit);
	hingeData.m_limitSoftness = float(m_limitSoftness);
	hingeData.m_biasFactor = float(m_biasFactor);
	hingeData.m_relaxationFactor = float(m_relaxationFactor);
#endif

	return btHingeConstraintDataName;
}

}
