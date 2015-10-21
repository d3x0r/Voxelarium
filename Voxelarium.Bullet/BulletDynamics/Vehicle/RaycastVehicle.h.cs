/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#if ! BT_RAYCASTVEHICLE_H
#define BT_RAYCASTVEHICLE_H

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "btVehicleRaycaster.h"
class btDynamicsWorld;
#include "LinearMath/List.h"
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"

class btVehicleTuning;

///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
class btRaycastVehicle : btActionInterface
{

		List<btVector3>	m_forwardWS;
		List<btVector3>	m_axle;
		List<double>	m_forwardImpulse;
		List<double>	m_sideImpulse;
	
		///backwards compatibility
		int	m_userConstraintType;
		int	m_userConstraintId;

public:
	class btVehicleTuning
		{
			public:

			btVehicleTuning()
				:m_suspensionStiffness((double)(5.88)),
				m_suspensionCompression((double)(0.83)),
				m_suspensionDamping((double)(0.88)),
				m_maxSuspensionTravelCm((double)(50 0.0)),
				m_frictionSlip((double)(10.5)),
				m_maxSuspensionForce((double)(600 0.0))
			{
			}
			double	m_suspensionStiffness;
			double	m_suspensionCompression;
			double	m_suspensionDamping;
			double	m_maxSuspensionTravelCm;
			double	m_frictionSlip;
			double	m_maxSuspensionForce;

		};
private:

	btVehicleRaycaster*	m_vehicleRaycaster;
	double		m_pitchControl;
	double	m_steeringValue; 
	double m_currentVehicleSpeedKmHour;

	btRigidBody m_chassisBody;

	int m_indexRightAxis;
	int m_indexUpAxis;
	int	m_indexForwardAxis;

	void defaultInit(btVehicleTuning tuning);

public:

	//constructor to create a car from an existing rigidbody
	btRaycastVehicle(btVehicleTuning& tuning,btRigidBody chassis,	btVehicleRaycaster* raycaster );

	virtual ~btRaycastVehicle() ;


	///btActionInterface interface
	virtual void updateAction( btCollisionWorld* collisionWorld, double step)
	{
        (void) collisionWorld;
		updateVehicle(step);
	}
	

	///btActionInterface interface
	void	debugDraw(btIDebugDraw* debugDrawer);
			
	ref btTransform getChassisWorldTransform();
	
	double rayCast(btWheelInfo& wheel);

	virtual void updateVehicle(double step);
	
	
	void resetSuspension();

	double	getSteeringValue(int wheel);

	void	setSteeringValue(double steering,int wheel);


	void	applyEngineForce(double force, int wheel);

	ref btTransform	getWheelTransformWS( int wheelIndex );

	void	updateWheelTransform( int wheelIndex, bool interpolatedTransform = true );
	
//	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, ref btVector3 hitPoint, ref btVector3 hitNormal,double depth);

	btWheelInfo&	addWheel( ref btVector3 connectionPointCS0, ref btVector3 wheelDirectionCS0,ref btVector3 wheelAxleCS,double suspensionRestLength,double wheelRadius,btVehicleTuning& tuning, bool isFrontWheel);

	inline int		getNumWheels() string 
		return int (m_wheelInfo.Count);
	}
	
	List<btWheelInfo>	m_wheelInfo;


	btWheelInfo&	getWheelInfo(int index);

	btWheelInfo&	getWheelInfo(int index);

	void	updateWheelTransformsWS(btWheelInfo& wheel , bool interpolatedTransform = true);

	
	void setBrake(double brake,int wheelIndex);

	void	setPitchControl(double pitch)
	{
		m_pitchControl = pitch;
	}
	
	void	updateSuspension(double deltaTime);

	virtual void	updateFriction(double	timeStep);



	inline btRigidBody getRigidBody()
	{
		return m_chassisBody;
	}

	btRigidBody getRigidBody()
	{
		return m_chassisBody;
	}

	inline int	getRightAxis()
	{
		return m_indexRightAxis;
	}
	inline int getUpAxis()
	{
		return m_indexUpAxis;
	}

	inline int getForwardAxis()
	{
		return m_indexForwardAxis;
	}

	
	///Worldspace forward vector
	btVector3 getForwardVector()
	{
		ref btTransform chassisTrans = getChassisWorldTransform(); 

		btVector3 forwardW ( 
			  chassisTrans.getBasis()[m_indexForwardAxis], 
			  chassisTrans.getBasis()[1][m_indexForwardAxis], 
			  chassisTrans.getBasis()[2][m_indexForwardAxis]); 

		return forwardW;
	}

	///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
	double	getCurrentSpeedKmHour()
	{
		return m_currentVehicleSpeedKmHour;
	}

	virtual void	setCoordinateSystem(int rightIndex,int upIndex,int forwardIndex)
	{
		m_indexRightAxis = rightIndex;
		m_indexUpAxis = upIndex;
		m_indexForwardAxis = forwardIndex;
	}


	///backwards compatibility
	int getUserConstraintType()
	{
		return m_userConstraintType ;
	}

	void	setUserConstraintType(int userConstraintType)
	{
		m_userConstraintType = userConstraintType;
	};

	void	setUserConstraintId(int uid)
	{
		m_userConstraintId = uid;
	}

	int getUserConstraintId()
	{
		return m_userConstraintId;
	}

};

class btDefaultVehicleRaycaster : btVehicleRaycaster
{
	btDynamicsWorld*	m_dynamicsWorld;
public:
	btDefaultVehicleRaycaster(btDynamicsWorld* world)
		:m_dynamicsWorld(world)
	{
	}

	virtual object castRay(ref btVector3 from,ref btVector3 to, btVehicleRaycasterResult& result);

};


#endif //BT_RAYCASTVEHICLE_H

