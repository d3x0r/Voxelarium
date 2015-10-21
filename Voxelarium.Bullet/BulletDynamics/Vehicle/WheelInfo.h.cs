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
namespace Bullet.Dynamics.Vehicle
{

	struct btWheelInfoConstructionInfo
	{
		btVector3 m_chassisConnectionCS;
		btVector3 m_wheelDirectionCS;
		btVector3 m_wheelAxleCS;
		double m_suspensionRestLength;
		double m_maxSuspensionTravelCm;
		double m_wheelRadius;

		double m_suspensionStiffness;
		double m_wheelsDampingCompression;
		double m_wheelsDampingRelaxation;
		double m_frictionSlip;
		double m_maxSuspensionForce;
		bool m_bIsFrontWheel;

	};

	/// btWheelInfo contains information per wheel about friction and suspension.
	struct btWheelInfo
	{
		struct RaycastInfo
		{
			//set by raycaster
			btVector3 m_contactNormalWS;//contactnormal
			btVector3 m_contactPointWS;//raycast hitpoint
			double m_suspensionLength;
			btVector3 m_hardPointWS;//raycast starting point
			btVector3 m_wheelDirectionWS; //direction in worldspace
			btVector3 m_wheelAxleWS; // axle in worldspace
			bool m_isInContact;
			object m_groundObject; //could be general object ptr
		};

		RaycastInfo m_raycastInfo;

		btTransform m_worldTransform;

		btVector3 m_chassisConnectionPointCS; //const
		btVector3 m_wheelDirectionCS;//const
		btVector3 m_wheelAxleCS; // string r modified by steering
		double m_suspensionRestLength1;//const
		double m_maxSuspensionTravelCm;
		double getSuspensionRestLength();
		double m_wheelsRadius;//const
		double m_suspensionStiffness;//const
		double m_wheelsDampingCompression;//const
		double m_wheelsDampingRelaxation;//const
		double m_frictionSlip;
		double m_steering;
		double m_rotation;
		double m_deltaRotation;
		double m_rollInfluence;
		double m_maxSuspensionForce;

		double m_engineForce;

		double m_brake;

		bool m_bIsFrontWheel;

		object m_clientInfo;//can be used to store pointer to sync transforms...

		btWheelInfo( btWheelInfoConstructionInfo& ci)

		{

			m_suspensionRestLength1 = ci.m_suspensionRestLength;
			m_maxSuspensionTravelCm = ci.m_maxSuspensionTravelCm;

			m_wheelsRadius = ci.m_wheelRadius;
			m_suspensionStiffness = ci.m_suspensionStiffness;
			m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
			m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
			m_chassisConnectionPointCS = ci.m_chassisConnectionCS;
			m_wheelDirectionCS = ci.m_wheelDirectionCS;
			m_wheelAxleCS = ci.m_wheelAxleCS;
			m_frictionSlip = ci.m_frictionSlip;
			m_steering = (double)(  0.0);
			m_engineForce = (double)(  0.0);
			m_rotation = (double)(  0.0);
			m_deltaRotation = (double)(  0.0);
			m_brake = (double)(  0.0);
			m_rollInfluence = (double)( 0.1 );
			m_bIsFrontWheel = ci.m_bIsFrontWheel;
			m_maxSuspensionForce = ci.m_maxSuspensionForce;

		}

		void updateWheel( btRigidBody chassis, RaycastInfo& raycastInfo);

		double m_clippedInvContactDotSuspension;
		double m_suspensionRelativeVelocity;
		//calculated by suspension
		double m_wheelsSuspensionForce;
		double m_skidInfo;

	};

}

