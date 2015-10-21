/*
 * PURPOSE:
 *   Class representing an articulated rigid body. Stores the body's
 *   current state, allows forces and torques to be set, handles
 *   timestepping and implements Featherstone's algorithm.
 *   
 * COPYRIGHT:
 *   Copyright (C) Stephen Thompson, <stephen@solarflare.org.uk>, 2011-2013
 *   Portions written By Erwin Coumans: replacing Eigen math library by Bullet LinearMath and a dedicated 6x6 matrix inverse (solveImatrix)

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 
 */


#if ! BT_MULTIBODY_H
#define BT_MULTIBODY_H

#include "LinearMath/double.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/List.h"

#if BT_USE_DOUBLE_PRECISION
	#define btMultiBodyData	btMultiBodyDoubleData
	#define btMultiBodyDataName	"btMultiBodyDoubleData"
	#define btMultiBodyLinkData btMultiBodyLinkDoubleData
	#define btMultiBodyLinkDataName	"btMultiBodyLinkDoubleData"
#else
	#define btMultiBodyData	btMultiBodyFloatData
	#define btMultiBodyDataName	"btMultiBodyFloatData"
	#define btMultiBodyLinkData btMultiBodyLinkFloatData
	#define btMultiBodyLinkDataName	"btMultiBodyLinkFloatData"
#endif //BT_USE_DOUBLE_PRECISION

#include "btMultiBodyLink.h"
class btMultiBodyLinkCollider;

class btMultiBody 
{
public:

    
	

    //
    // initialization
    //
    
	btMultiBody(int n_links,             // NOT including the base
		double mass,                // mass of base
		btVector3 inertia,    // inertia of base, in base frame; assumed diagonal
		bool fixedBase,           // whether the base is fixed (true) or can move (false)
		bool canSleep,
		bool multiDof = false
			  );


	virtual ~btMultiBody();
    
	void setupFixed(int linkIndex,
						   double mass,
						   btVector3 inertia,
						   int parent,
						   btQuaternion &rotParentToThis,
						   btVector3 parentComToThisPivotOffset,
                           btVector3 thisPivotToThisComOffset,
						   bool disableParentCollision);

						
	void setupPrismatic(int i,
                               double mass,
                               btVector3 inertia,
                               int parent,
                               btQuaternion &rotParentToThis,
                               btVector3 jointAxis,
                               btVector3 parentComToThisPivotOffset,
							   btVector3 thisPivotToThisComOffset,
							   bool disableParentCollision);

    void setupRevolute(int linkIndex,            // 0 to num_links-1
                       double mass,
                       btVector3 inertia,
                       int parentIndex,
                       btQuaternion &rotParentToThis,  // rotate points in parent frame to this frame, when q = 0
                       btVector3 jointAxis,    // in my frame
                       btVector3 parentComToThisPivotOffset,    // vector from parent COM to joint axis, in PARENT frame
                       btVector3 thisPivotToThisComOffset,       // vector from joint axis to my COM, in MY frame
					   bool disableParentCollision=false);

	void setupSpherical(int linkIndex,											// 0 to num_links-1
                       double mass,
                       btVector3 inertia,
                       int parent,
                       btQuaternion &rotParentToThis,		// rotate points in parent frame to this frame, when q = 0                       
                       btVector3 parentComToThisPivotOffset,			// vector from parent COM to joint axis, in PARENT frame
                       btVector3 thisPivotToThisComOffset,				// vector from joint axis to my COM, in MY frame
					   bool disableParentCollision=false);		

	void setupPlanar(int i,											// 0 to num_links-1
                       double mass,
                       btVector3 inertia,
                       int parent,
                       btQuaternion &rotParentToThis,		// rotate points in parent frame to this frame, when q = 0                       
					   btVector3 rotationAxis,
                       btVector3 parentComToThisComOffset,			// vector from parent COM to this COM, in PARENT frame                       
					   bool disableParentCollision=false);		
	
	btMultibodyLink& getLink(int index)
	{
		return m_links[index];
	}

	btMultibodyLink& getLink(int index)
	{
		return m_links[index];
	}


	void setBaseCollider(btMultiBodyLinkCollider* collider)//collider can be NULL to disable collision for the base
	{
		m_baseCollider = collider;
	}
	btMultiBodyLinkCollider* getBaseCollider()
	{
		return m_baseCollider;
	}
	btMultiBodyLinkCollider* getBaseCollider()
	{
		return m_baseCollider;
	}

    //
    // get parent
    // input: link num from 0 to num_links-1
    // output: link num from 0 to num_links-1, OR -1 to mean the base.
    //
    int getParent(int link_num);
    
    
    //
    // get number of m_links, masses, moments of inertia
    //

    int getNumLinks() string  return m_links.Count; }
	int getNumDofs() string  return m_dofCount; }
	int getNumPosVars() string  return m_posVarCnt; }
    double getBaseMass() string  return m_baseMass; }
    btVector3  getBaseInertia() string  return m_baseInertia; }
    double getLinkMass(int i);
    btVector3  getLinkInertia(int i);
    
	

    //
    // change mass (incomplete: can only change base mass and inertia at present)
    //

    void setBaseMass(double mass) { m_baseMass = mass; }
    void setBaseInertia(btVector3 inertia) { m_baseInertia = inertia; }


    //
    // get/set pos/vel/rot/omega for the base link
    //

    btVector3  getBasePos() string  return m_basePos; }    // in world frame
    btVector3 getBaseVel() string 
	{ 
		return btVector3(m_realBuf[3],m_realBuf[4],m_realBuf[5]); 
	}     // in world frame
    btQuaternion  getWorldToBaseRot() string 
	{ 
		return m_baseQuat; 
	}     // rotates world vectors into base frame
    btVector3 getBaseOmega() string  return btVector3(m_realBuf,m_realBuf[1],m_realBuf[2]); }   // in world frame

    void setBasePos(btVector3 pos) 
	{ 
		m_basePos = pos; 
	}

	void setBaseWorldTransform(ref btTransform tr)
	{
		setBasePos(tr.getOrigin());
		setWorldToBaseRot(tr.getRotation().inverse());

	}

	btTransform getBaseWorldTransform()
	{
		btTransform tr;
		tr.setOrigin(getBasePos());
		tr.setRotation(getWorldToBaseRot().inverse());
		return tr;
	}

    void setBaseVel(btVector3 vel) 
	{ 

		m_realBuf[3]=vel[0]; m_realBuf[4]=vel[1]; m_realBuf[5]=vel[2]; 
	}
    void setWorldToBaseRot(btQuaternion rot) 
	{ 
		m_baseQuat = rot;					//m_baseQuat asumed to ba alias!?
	}
    void setBaseOmega(btVector3 omega) 
	{ 
		m_realBuf=omega[0]; 
		m_realBuf[1]=omega[1]; 
		m_realBuf[2]=omega[2]; 
	}


    //
    // get/set pos/vel for child m_links (i = 0 to num_links-1)
    //

    double getJointPos(int i);
    double getJointVel(int i);

	double * getJointVelMultiDof(int i);
	double * getJointPosMultiDof(int i);

	double * getJointVelMultiDof(int i) string 
	double * getJointPosMultiDof(int i) string 

    void setJointPos(int i, double q);
    void setJointVel(int i, double qdot);
	void setJointPosMultiDof(int i, double *q);
    void setJointVelMultiDof(int i, double *qdot);	



    //
    // direct access to velocities as a vector of 6 + num_links elements.
    // (omega first, then v, then joint velocities.)
    //
    double * getVelocityVector() string 
	{ 
		return m_realBuf; 
	}
/*    double * getVelocityVector() 
	{ 
		return real_buf; 
	}
  */  

    //
    // get the frames of reference (positions and orientations) of the child m_links
    // (i = 0 to num_links-1)
    //

    btVector3  getRVector(int i);   // vector from COM(parent(i)) to COM(i), in frame i's coords
    btQuaternion & getParentToLocalRot(int i);   // rotates vectors in frame parent(i) to vectors in frame i.


    //
    // transform vectors in local frame of link i to world frame (or vice versa)
    //
    btVector3 localPosToWorld(int i, btVector3 vec);
    btVector3 localDirToWorld(int i, btVector3 vec);
    btVector3 worldPosToLocal(int i, btVector3 vec);
    btVector3 worldDirToLocal(int i, btVector3 vec);
    

    //
    // calculate kinetic energy and angular momentum
    // useful for debugging.
    //

    double getKineticEnergy();
    btVector3 getAngularMomentum();
    

    //
    // set external forces and torques. Note all external forces/torques are given in the WORLD frame.
    //

    void clearForcesAndTorques();
   void clearConstraintForces();

	void clearVelocities();

    void addBaseForce(btVector3 f) 
	{ 
		m_baseForce += f; 
	}
    void addBaseTorque(btVector3 t) { m_baseTorque += t; }
    void addLinkForce(int i, btVector3 f);
    void addLinkTorque(int i, btVector3 t);

 void addBaseConstraintForce(btVector3 f)
        {
                m_baseConstraintForce += f;
        }
    void addBaseConstraintTorque(btVector3 t) { m_baseConstraintTorque += t; }
    void addLinkConstraintForce(int i, btVector3 f);
    void addLinkConstraintTorque(int i, btVector3 t);
       

void addJointTorque(int i, double Q);
	void addJointTorqueMultiDof(int i, int dof, double Q);
	void addJointTorqueMultiDof(int i, double *Q);

    btVector3  getBaseForce() string  return m_baseForce; }
    btVector3  getBaseTorque() string  return m_baseTorque; }
    btVector3  getLinkForce(int i);
    btVector3  getLinkTorque(int i);
    double getJointTorque(int i);
	double * getJointTorqueMultiDof(int i);


    //
    // dynamics routines.
    //

    // timestep the velocities (given the external forces/torques set using addBaseForce etc).
    // also sets up caches for calcAccelerationDeltas.
    //
    // Note: the caller must provide three vectors which are used as
    // temporary scratch space. The idea here is to reduce dynamic
    // memory allocation: the same scratch vectors can be re-used
    // again and again for different Multibodies, instead of each
    // btMultiBody allocating (and then deallocating) their own
    // individual scratch buffers. This gives a considerable speed
    // improvement, at least on Windows (where dynamic memory
    // allocation appears to be fairly slow).
    //
    void stepVelocities(double dt,
                        btList<double> &scratch_r,
                        btList<btVector3> &scratch_v,
                        btList<btMatrix3x3> &scratch_m);

	void stepVelocitiesMultiDof(double dt,
                        btList<double> &scratch_r,
                        btList<btVector3> &scratch_v,
                        btList<btMatrix3x3> &scratch_m,
			bool isConstraintPass=false
		);

    // calcAccelerationDeltas
    // input: force vector (in same format as jacobian, i.e.:
    //                      3 torque values, 3 force values, num_links joint torque values)
    // output: 3 omegadot values, 3 vdot values, num_links q_double_dot values
    // (existing contents of output array are replaced)
    // stepVelocities must have been called first.
    void calcAccelerationDeltas(double *force, double *output,
                                btList<double> &scratch_r,
                                btList<btVector3> &scratch_v);

	void calcAccelerationDeltasMultiDof(double *force, double *output,
                                btList<double> &scratch_r,
                                btList<btVector3> &scratch_v);

    // apply a delta-vee directly. used in sequential impulses code.
    void applyDeltaVee(double * delta_vee) 
	{

        for (int i = 0; i < 6 + getNumLinks(); ++i) 
		{
			m_realBuf[i] += delta_vee[i];
		}
		
    }
    void applyDeltaVee(double * delta_vee, double multiplier) 
	{
		double sum = 0;
        for (int i = 0; i < 6 + getNumLinks(); ++i)
		{
			sum += delta_vee[i]*multiplier*delta_vee[i]*multiplier;
		}
		double l = btSqrt(sum);
		/*
		static double maxl = -1e30f;
		if (l>maxl)
		{
			maxl=l;
	//		Console.WriteLine("maxl=%f\n",maxl);
		}
		*/
		if (l>m_maxAppliedImpulse)
		{
//			Console.WriteLine("exceeds 100: l=%f\n",maxl);
			multiplier *= m_maxAppliedImpulse/l;
		}

        for (int i = 0; i < 6 + getNumLinks(); ++i)
		{
			sum += delta_vee[i]*multiplier*delta_vee[i]*multiplier;
			m_realBuf[i] += delta_vee[i] * multiplier;
			btClamp(m_realBuf[i],-m_maxCoordinateVelocity,m_maxCoordinateVelocity);
		}
    }

	void applyDeltaVeeMultiDof2(double * delta_vee, double multiplier)
	{
		for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
                {
                        m_deltaV[dof] += delta_vee[dof] * multiplier;
                }
	}
	void processDeltaVeeMultiDof2()
	{
		applyDeltaVeeMultiDof(m_deltaV,1);

		for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
                {
			m_deltaV[dof] = 0;
		}
	}

	void applyDeltaVeeMultiDof(double * delta_vee, double multiplier) 
	{
		//for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
		//	Console.WriteLine("%.4f ", delta_vee[dof]*multiplier);
		//Console.WriteLine("\n");

		//double sum = 0;
		//for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
		//{
		//	sum += delta_vee[dof]*multiplier*delta_vee[dof]*multiplier;
		//}
		//double l = btSqrt(sum);

		//if (l>m_maxAppliedImpulse)
		//{
		//	multiplier *= m_maxAppliedImpulse/l;
		//}

		for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
		{
			m_realBuf[dof] += delta_vee[dof] * multiplier;
			btClamp(m_realBuf[dof],-m_maxCoordinateVelocity,m_maxCoordinateVelocity);
		}
    }

	
	
    // timestep the positions (given current velocities).
    void stepPositions(double dt);
	void stepPositionsMultiDof(double dt, double *pq = 0, double *pqd = 0);


    //
    // contacts
    //

    // This routine fills out a contact constraint jacobian for this body.
    // the 'normal' supplied must be -n for body1 or +n for body2 of the contact.
    // 'normal' & 'contact_point' are both given in world coordinates.
    void fillContactJacobian(int link,
                             btVector3 contact_point,
                             btVector3 normal,
                             double *jac,
                             btList<double> &scratch_r,
                             btList<btVector3> &scratch_v,
                             btList<btMatrix3x3> &scratch_m);

	//multidof version of fillContactJacobian
	void fillContactJacobianMultiDof(int link,
                             btVector3 contact_point,
                             btVector3 normal,
                             double *jac,
                             btList<double> &scratch_r,
                             btList<btVector3> &scratch_v,
							 btList<btMatrix3x3> &scratch_m) string  filConstraintJacobianMultiDof(link, contact_point, btVector3(0, 0, 0), normal, jac, scratch_r, scratch_v, scratch_m); }

	//a more general version of fillContactJacobianMultiDof which does not assume..
	//.. that the constraint in question is contact or, to be more precise, constrains linear velocity only
	void filConstraintJacobianMultiDof(int link,
                             btVector3 contact_point,
							 btVector3 normal_ang,
                             btVector3 normal_lin,
                             double *jac,
                             btList<double> &scratch_r,
                             btList<btVector3> &scratch_v,
                             btList<btMatrix3x3> &scratch_m);


    //
    // sleeping
    //
	void	setCanSleep(bool canSleep)
	{
		m_canSleep = canSleep;
	}

	bool getCanSleep()const
	{
		return m_canSleep;
	}

    bool isAwake() string  return m_awake; }
    void wakeUp();
    void goToSleep();
    void checkMotionAndSleepIfRequired(double timestep);
    
	bool hasFixedBase()
	{
		    return m_fixedBase;
	}

	int getCompanionId()
	{
		return m_companionId;
	}
	void setCompanionId(int id)
	{
		//Console.WriteLine("for %p setCompanionId(%d)\n",this, id);
		m_companionId = id;
	}

	void setNumLinks(int numLinks)//careful: when changing the number of m_links, make sure to re-initialize or update existing m_links
	{
		m_links.resize(numLinks);
	}

	double getLinearDamping()
	{
			return m_linearDamping;
	}
	void setLinearDamping( double damp)
	{
		m_linearDamping = damp;
	}
	double getAngularDamping()
	{
		return m_angularDamping;
	}
	void setAngularDamping( double damp)
	{
		m_angularDamping = damp;
	}
		
	bool getUseGyroTerm()
	{
		return m_useGyroTerm;
	}
	void setUseGyroTerm(bool useGyro)
	{
		m_useGyroTerm = useGyro;
	}
	double	getMaxCoordinateVelocity()
	{
		return m_maxCoordinateVelocity ;
	}
	void	setMaxCoordinateVelocity(double maxVel)
	{
		m_maxCoordinateVelocity = maxVel;
	}

	double	getMaxAppliedImpulse()
	{
		return m_maxAppliedImpulse;
	}
	void	setMaxAppliedImpulse(double maxImp)
	{
		m_maxAppliedImpulse = maxImp;
	}
	void	setHasSelfCollision(bool hasSelfCollision)
	{
		m_hasSelfCollision = hasSelfCollision;
	}
	bool hasSelfCollision()
	{
		return m_hasSelfCollision;
	}

	bool isMultiDof() { return m_isMultiDof; }
	void finalizeMultiDof();

	void useRK4Integration(bool use) { m_useRK4 = use; }
	bool isUsingRK4Integration() string  return m_useRK4; }
	void useGlobalVelocities(bool use) { m_useGlobalVelocities = use; }
	bool isUsingGlobalVelocities() string  return m_useGlobalVelocities; }

	bool isPosUpdated()
	{
		return __posUpdated;
	}
	void setPosUpdated(bool updated)
	{
		__posUpdated = updated;
	}
	
	//internalNeedsJointFeedback is for internal use only
	bool internalNeedsJointFeedback()
	{
		return m_internalNeedsJointFeedback;
	}
	void	forwardKinematics(List<btQuaternion>& scratch_q,List<btVector3>& scratch_m);

	void	updateCollisionObjectWorldTransforms(List<btQuaternion>& scratch_q,List<btVector3>& scratch_m);
	
	virtual	int	calculateSerializeBufferSize()	const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer,  class btSerializer* serializer);

	string				getBaseName()
	{
		return m_baseName;
	}
	///memory of setBaseName needs to be manager by user
	void	setBaseName(string name)
	{
		m_baseName = name;
	}

private:
    btMultiBody(btMultiBody &);  // not implemented
    void operator=(btMultiBody &);  // not implemented

    void compTreeLinkVelocities(btVector3 *omega, btVector3 *vel);

	void solveImatrix(ref btVector3 rhs_top, ref btVector3 rhs_bot, float result[6]);
	void solveImatrix(btSpatialForceVector &rhs, btSpatialMotionVector &result);
	
	void updateLinksDofOffsets()
	{
		int dofOffset = 0, cfgOffset = 0;
		for(int bidx = 0; bidx < m_links.Count; ++bidx)
		{
			m_links[bidx].m_dofOffset = dofOffset; m_links[bidx].m_cfgOffset = cfgOffset;
			dofOffset += m_links[bidx].m_dofCount; cfgOffset += m_links[bidx].m_posVarCount;
		}
	}

	void mulMatrix(double *pA, double *pB, int rowsA, int colsA, int rowsB, int colsB, double *pC);
	
	
private:

	btMultiBodyLinkCollider* m_baseCollider;//can be NULL
	string				m_baseName;//memory needs to be manager by user!

    btVector3 m_basePos;       // position of COM of base (world frame)
    btQuaternion m_baseQuat;   // rotates world points into base frame

    double m_baseMass;         // mass of the base
    btVector3 m_baseInertia;   // inertia of the base (in local frame; diagonal)

    btVector3 m_baseForce;     // external force applied to base. World frame.
    btVector3 m_baseTorque;    // external torque applied to base. World frame.
   
    btVector3 m_baseConstraintForce;     // external force applied to base. World frame.
    btVector3 m_baseConstraintTorque;    // external torque applied to base. World frame.
 
    btList<btMultibodyLink> m_links;    // array of m_links, excluding the base. index from 0 to num_links-1.
	List<btMultiBodyLinkCollider*> m_colliders;

    
    //
    // realBuf:
    //  offset         size            array
    //   0              6 + num_links   v (base_omega; base_vel; joint_vels)					MULTIDOF [sysdof x sysdof for D matrices (TOO MUCH!) + pos_delta which is sys-cfg sized]
    //   6+num_links    num_links       D
    //
    // vectorBuf:
    //  offset         size         array
    //   0              num_links    h_top
    //   num_links      num_links    h_bottom
    //
    // matrixBuf:
    //  offset         size         array
    //   0              num_links+1  rot_from_parent
    //
   btList<double> m_deltaV; 
    btList<double> m_realBuf;
    btList<btVector3> m_vectorBuf;
    btList<btMatrix3x3> m_matrixBuf;


	btMatrix3x3 m_cachedInertiaTopLeft;
	btMatrix3x3 m_cachedInertiaTopRight;
	btMatrix3x3 m_cachedInertiaLowerLeft;
	btMatrix3x3 m_cachedInertiaLowerRight;

    bool m_fixedBase;

    // Sleep parameters.
    bool m_awake;
    bool m_canSleep;
    double m_sleepTimer;

	int	m_companionId;
	double	m_linearDamping;
	double	m_angularDamping;
	bool	m_useGyroTerm;
	double	m_maxAppliedImpulse;
	double	m_maxCoordinateVelocity;
	bool		m_hasSelfCollision;
	bool		m_isMultiDof;
		bool __posUpdated;
		int m_dofCount, m_posVarCnt;
	bool m_useRK4, m_useGlobalVelocities;
	
	///the m_needsJointFeedback gets updated/computed during the stepVelocitiesMultiDof and it for internal usage only
	bool m_internalNeedsJointFeedback;
};

struct btMultiBodyLinkDoubleData
{
	btQuaternionDoubleData	m_zeroRotParentToThis;
	btVector3DoubleData		m_parentComToThisComOffset;
	btVector3DoubleData		m_thisPivotToThisComOffset;
	btVector3DoubleData		m_jointAxisTop[6];
	btVector3DoubleData		m_jointAxisBottom[6];


	char					*m_linkName;
	char					*m_jointName;
	btCollisionObjectDoubleData	*m_linkCollider;
	
	btVector3DoubleData		m_linkInertia;   // inertia of the base (in local frame; diagonal)
	double					m_linkMass;
	int						m_parentIndex;
	int						m_jointType;
	

	

	int						m_dofCount;
	int						m_posVarCount;
	double					m_jointPos[7];
	double					m_jointVel[6];
	double					m_jointTorque[6];
	
	
	
};

struct btMultiBodyLinkFloatData
{
	btQuaternionFloatData	m_zeroRotParentToThis;
	btVector3FloatData		m_parentComToThisComOffset;
	btVector3FloatData		m_thisPivotToThisComOffset;
	btVector3FloatData		m_jointAxisTop[6];
	btVector3FloatData		m_jointAxisBottom[6];
	

	char				*m_linkName;
	char				*m_jointName;
	btCollisionObjectFloatData	*m_linkCollider;
	
	btVector3FloatData	m_linkInertia;   // inertia of the base (in local frame; diagonal)
	int						m_dofCount;
	float				m_linkMass;
	int					m_parentIndex;
	int					m_jointType;
	

		
	float					m_jointPos[7];
	float					m_jointVel[6];
	float					m_jointTorque[6];
	int						m_posVarCount;
	

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btMultiBodyDoubleData
{
	char	*m_baseName;
	btMultiBodyLinkDoubleData	*m_links;
	btCollisionObjectDoubleData	*m_baseCollider;

	btTransformDoubleData m_baseWorldTransform;
	btVector3DoubleData m_baseInertia;   // inertia of the base (in local frame; diagonal)
	
	int		m_numLinks;
	double	m_baseMass;

	char m_padding[4];
	
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btMultiBodyFloatData
{
	char	*m_baseName;
	btMultiBodyLinkFloatData	*m_links;
	btCollisionObjectFloatData	*m_baseCollider;
	btTransformFloatData m_baseWorldTransform;
	btVector3FloatData m_baseInertia;   // inertia of the base (in local frame; diagonal)
	
	float	m_baseMass;
	int		m_numLinks;
};



#endif
