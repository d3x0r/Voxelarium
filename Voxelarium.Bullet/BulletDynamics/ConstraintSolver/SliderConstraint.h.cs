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

/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

TODO:
 - add clamping od accumulated impulse to improve stability
 - add conversion for ODE constraint solver
*/

namespace Bullet.Dynamics.ConstraintSolver
{
	/*
#if BT_USE_DOUBLE_PRECISION
#define btSliderConstraintData2		btSliderConstraintDoubleData
#define btSliderConstraintDataName  "btSliderConstraintDoubleData"
#else
#define btSliderConstraintData2		btSliderConstraintData 
#define btSliderConstraintDataName	"btSliderConstraintData"
#endif //BT_USE_DOUBLE_PRECISION


*/


#define SLIDER_CONSTRAINT_DEF_SOFTNESS		((double)(1.0))
#define SLIDER_CONSTRAINT_DEF_DAMPING		((double)(1.0))
#define SLIDER_CONSTRAINT_DEF_RESTITUTION	((double)(0.7))
#define SLIDER_CONSTRAINT_DEF_CFM			((double)(0))


enum btSliderFlags
{
	BT_SLIDER_FLAGS_CFM_DIRLIN = (1 << 0),
	BT_SLIDER_FLAGS_ERP_DIRLIN = (1 << 1),
	BT_SLIDER_FLAGS_CFM_DIRANG = (1 << 2),
	BT_SLIDER_FLAGS_ERP_DIRANG = (1 << 3),
	BT_SLIDER_FLAGS_CFM_ORTLIN = (1 << 4),
	BT_SLIDER_FLAGS_ERP_ORTLIN = (1 << 5),
	BT_SLIDER_FLAGS_CFM_ORTANG = (1 << 6),
	BT_SLIDER_FLAGS_ERP_ORTANG = (1 << 7),
	BT_SLIDER_FLAGS_CFM_LIMLIN = (1 << 8),
	BT_SLIDER_FLAGS_ERP_LIMLIN = (1 << 9),
	BT_SLIDER_FLAGS_CFM_LIMANG = (1 << 10),
	BT_SLIDER_FLAGS_ERP_LIMANG = (1 << 11)
};


internal class btSliderConstraint : btTypedConstraint
{
protected:
	///for backwards compatibility during the transition to 'getInfo/getInfo2'
	bool		m_useSolveConstraintObsolete;
	bool		m_useOffsetForConstraintFrame;
	btTransform	m_frameInA;
    btTransform	m_frameInB;
	// use frameA fo define limits, if true
	bool m_useLinearReferenceFrameA;
	// linear limits
	double m_lowerLinLimit;
	double m_upperLinLimit;
	// angular limits
	double m_lowerAngLimit;
	double m_upperAngLimit;
	// softness, restitution and damping for different cases
	// DirLin - moving inside linear limits
	// LimLin - hitting linear limit
	// DirAng - moving inside angular limits
	// LimAng - hitting angular limit
	// OrthoLin, OrthoAng - against constraint axis
	double m_softnessDirLin;
	double m_restitutionDirLin;
	double m_dampingDirLin;
	double m_cfmDirLin;

	double m_softnessDirAng;
	double m_restitutionDirAng;
	double m_dampingDirAng;
	double m_cfmDirAng;

	double m_softnessLimLin;
	double m_restitutionLimLin;
	double m_dampingLimLin;
	double m_cfmLimLin;

	double m_softnessLimAng;
	double m_restitutionLimAng;
	double m_dampingLimAng;
	double m_cfmLimAng;

	double m_softnessOrthoLin;
	double m_restitutionOrthoLin;
	double m_dampingOrthoLin;
	double m_cfmOrthoLin;

	double m_softnessOrthoAng;
	double m_restitutionOrthoAng;
	double m_dampingOrthoAng;
	double m_cfmOrthoAng;
	
	// for interlal use
	bool m_solveLinLim;
	bool m_solveAngLim;

	int m_flags;

	btJacobianEntry	m_jacLin[3];
	double		m_jacLinDiagABInv[3];

    btJacobianEntry	m_jacAng[3];

	double m_timeStep;
    btTransform m_calculatedTransformA;
    btTransform m_calculatedTransformB;

	btVector3 m_sliderAxis;
	btVector3 m_realPivotAInW;
	btVector3 m_realPivotBInW;
	btVector3 m_projPivotInW;
	btVector3 m_delta;
	btVector3 m_depth;
	btVector3 m_relPosA;
	btVector3 m_relPosB;

	double m_linPos;
	double m_angPos;

	double m_angDepth;
	double m_kAngle;

	bool	 m_poweredLinMotor;
    double m_targetLinMotorVelocity;
    double m_maxLinMotorForce;
    double m_accumulatedLinMotorImpulse;
	
	bool	 m_poweredAngMotor;
    double m_targetAngMotorVelocity;
    double m_maxAngMotorForce;
    double m_accumulatedAngMotorImpulse;

	//------------------------    
	void initParams();
public:
	
	
	// constructors
    btSliderConstraint(btRigidBody rbA, btRigidBody rbB, ref btTransform frameInA, ref btTransform frameInB ,bool useLinearReferenceFrameA);
    btSliderConstraint(btRigidBody rbB, ref btTransform frameInB, bool useLinearReferenceFrameA);

	// overrides

    virtual void getInfo1 (btConstraintInfo1* info);

	void getInfo1NonVirtual(btConstraintInfo1* info);
	
	virtual void getInfo2 (btConstraintInfo2* info);

	void getInfo2NonVirtual(btConstraintInfo2* info, ref btTransform transA, ref btTransform transB,ref btVector3 linVelA,ref btVector3 linVelB, double rbAinvMass,double rbBinvMass);


	// access
    btRigidBody getRigidBodyA() string  return m_rbA; }
    btRigidBody getRigidBodyB() string  return m_rbB; }
    btTransform  getCalculatedTransformA() string  return m_calculatedTransformA; }
    btTransform & getCalculatedTransformB() string  return m_calculatedTransformB; }
    btTransform & getFrameOffsetA() string  return m_frameInA; }
    btTransform & getFrameOffsetB() string  return m_frameInB; }
    btTransform & getFrameOffsetA() { return m_frameInA; }
    btTransform & getFrameOffsetB() { return m_frameInB; }
    double getLowerLinLimit() { return m_lowerLinLimit; }
    void setLowerLinLimit(double lowerLimit) { m_lowerLinLimit = lowerLimit; }
    double getUpperLinLimit() { return m_upperLinLimit; }
    void setUpperLinLimit(double upperLimit) { m_upperLinLimit = upperLimit; }
    double getLowerAngLimit() { return m_lowerAngLimit; }
    void setLowerAngLimit(double lowerLimit) { m_lowerAngLimit = btNormalizeAngle(lowerLimit); }
    double getUpperAngLimit() { return m_upperAngLimit; }
    void setUpperAngLimit(double upperLimit) { m_upperAngLimit = btNormalizeAngle(upperLimit); }
	bool getUseLinearReferenceFrameA() { return m_useLinearReferenceFrameA; }
	double getSoftnessDirLin() { return m_softnessDirLin; }
	double getRestitutionDirLin() { return m_restitutionDirLin; }
	double getDampingDirLin() { return m_dampingDirLin ; }
	double getSoftnessDirAng() { return m_softnessDirAng; }
	double getRestitutionDirAng() { return m_restitutionDirAng; }
	double getDampingDirAng() { return m_dampingDirAng; }
	double getSoftnessLimLin() { return m_softnessLimLin; }
	double getRestitutionLimLin() { return m_restitutionLimLin; }
	double getDampingLimLin() { return m_dampingLimLin; }
	double getSoftnessLimAng() { return m_softnessLimAng; }
	double getRestitutionLimAng() { return m_restitutionLimAng; }
	double getDampingLimAng() { return m_dampingLimAng; }
	double getSoftnessOrthoLin() { return m_softnessOrthoLin; }
	double getRestitutionOrthoLin() { return m_restitutionOrthoLin; }
	double getDampingOrthoLin() { return m_dampingOrthoLin; }
	double getSoftnessOrthoAng() { return m_softnessOrthoAng; }
	double getRestitutionOrthoAng() { return m_restitutionOrthoAng; }
	double getDampingOrthoAng() { return m_dampingOrthoAng; }
	void setSoftnessDirLin(double softnessDirLin) { m_softnessDirLin = softnessDirLin; }
	void setRestitutionDirLin(double restitutionDirLin) { m_restitutionDirLin = restitutionDirLin; }
	void setDampingDirLin(double dampingDirLin) { m_dampingDirLin = dampingDirLin; }
	void setSoftnessDirAng(double softnessDirAng) { m_softnessDirAng = softnessDirAng; }
	void setRestitutionDirAng(double restitutionDirAng) { m_restitutionDirAng = restitutionDirAng; }
	void setDampingDirAng(double dampingDirAng) { m_dampingDirAng = dampingDirAng; }
	void setSoftnessLimLin(double softnessLimLin) { m_softnessLimLin = softnessLimLin; }
	void setRestitutionLimLin(double restitutionLimLin) { m_restitutionLimLin = restitutionLimLin; }
	void setDampingLimLin(double dampingLimLin) { m_dampingLimLin = dampingLimLin; }
	void setSoftnessLimAng(double softnessLimAng) { m_softnessLimAng = softnessLimAng; }
	void setRestitutionLimAng(double restitutionLimAng) { m_restitutionLimAng = restitutionLimAng; }
	void setDampingLimAng(double dampingLimAng) { m_dampingLimAng = dampingLimAng; }
	void setSoftnessOrthoLin(double softnessOrthoLin) { m_softnessOrthoLin = softnessOrthoLin; }
	void setRestitutionOrthoLin(double restitutionOrthoLin) { m_restitutionOrthoLin = restitutionOrthoLin; }
	void setDampingOrthoLin(double dampingOrthoLin) { m_dampingOrthoLin = dampingOrthoLin; }
	void setSoftnessOrthoAng(double softnessOrthoAng) { m_softnessOrthoAng = softnessOrthoAng; }
	void setRestitutionOrthoAng(double restitutionOrthoAng) { m_restitutionOrthoAng = restitutionOrthoAng; }
	void setDampingOrthoAng(double dampingOrthoAng) { m_dampingOrthoAng = dampingOrthoAng; }
	void setPoweredLinMotor(bool onOff) { m_poweredLinMotor = onOff; }
	bool getPoweredLinMotor() { return m_poweredLinMotor; }
	void setTargetLinMotorVelocity(double targetLinMotorVelocity) { m_targetLinMotorVelocity = targetLinMotorVelocity; }
	double getTargetLinMotorVelocity() { return m_targetLinMotorVelocity; }
	void setMaxLinMotorForce(double maxLinMotorForce) { m_maxLinMotorForce = maxLinMotorForce; }
	double getMaxLinMotorForce() { return m_maxLinMotorForce; }
	void setPoweredAngMotor(bool onOff) { m_poweredAngMotor = onOff; }
	bool getPoweredAngMotor() { return m_poweredAngMotor; }
	void setTargetAngMotorVelocity(double targetAngMotorVelocity) { m_targetAngMotorVelocity = targetAngMotorVelocity; }
	double getTargetAngMotorVelocity() { return m_targetAngMotorVelocity; }
	void setMaxAngMotorForce(double maxAngMotorForce) { m_maxAngMotorForce = maxAngMotorForce; }
	double getMaxAngMotorForce() { return m_maxAngMotorForce; }

	double getLinearPos() string  return m_linPos; }
	double getAngularPos() string  return m_angPos; }
	
	

	// access for ODE solver
	bool getSolveLinLimit() { return m_solveLinLim; }
	double getLinDepth() { return m_depth; }
	bool getSolveAngLimit() { return m_solveAngLim; }
	double getAngDepth() { return m_angDepth; }
	// shared code used by ODE solver
	void	calculateTransforms(ref btTransform transA,ref btTransform transB);
	void	testLinLimits();
	void	testAngLimits();
	// access for PE Solver
	btVector3 getAncorInA();
	btVector3 getAncorInB();
	// access for UseFrameOffset
	bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
	void setUseFrameOffset(bool frameOffsetOnOff) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }

	void setFrames(ref btTransform frameA, ref btTransform frameB) 
	{ 
		m_frameInA=frameA; 
		m_frameInB=frameB;
		calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
		buildJacobian();
	} 


	///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
	///If no axis is provided, it uses the default axis for this constraint.
	virtual	void	setParam(int num, double value, int axis = -1);
	///return the local value of parameter
	virtual	double getParam(int num, int axis = -1);

	virtual	int	calculateSerializeBufferSize();

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer, btSerializer* serializer);


};


///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64


struct btSliderConstraintData
{
	btTypedConstraintData	m_typeConstraintData;
	btTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformFloatData m_rbBFrame;
	
	float	m_linearUpperLimit;
	float	m_linearLowerLimit;

	float	m_angularUpperLimit;
	float	m_angularLowerLimit;

	int	m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;

};


struct btSliderConstraintDoubleData
{
	btTypedConstraintDoubleData	m_typeConstraintData;
	btTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformDoubleData m_rbBFrame;
	
	double	m_linearUpperLimit;
	double	m_linearLowerLimit;

	double	m_angularUpperLimit;
	double	m_angularLowerLimit;

	int	m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;

};

public		int	btSliderConstraint::calculateSerializeBufferSize()
{
	return sizeof(btSliderConstraintData2);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
public	string	btSliderConstraint::serialize(object dataBuffer, btSerializer* serializer)
{

	btSliderConstraintData2* sliderData = (btSliderConstraintData2*) dataBuffer;
	btTypedConstraint::serialize(&sliderData.m_typeConstraintData,serializer);

	m_frameInA.serialize(sliderData.m_rbAFrame);
	m_frameInB.serialize(sliderData.m_rbBFrame);

	sliderData.m_linearUpperLimit = m_upperLinLimit;
	sliderData.m_linearLowerLimit = m_lowerLinLimit;

	sliderData.m_angularUpperLimit = m_upperAngLimit;
	sliderData.m_angularLowerLimit = m_lowerAngLimit;

	sliderData.m_useLinearReferenceFrameA = m_useLinearReferenceFrameA;
	sliderData.m_useOffsetForConstraintFrame = m_useOffsetForConstraintFrame;

	return btSliderConstraintDataName;
}



}
