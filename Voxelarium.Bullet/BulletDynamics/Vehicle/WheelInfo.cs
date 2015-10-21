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
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btRigidBody.h" // for pointvelocity


double btWheelInfo::getSuspensionRestLength()
{

	return m_suspensionRestLength1;

}

void	btWheelInfo::updateWheel(btRigidBody chassis,RaycastInfo& raycastInfo)
{
	(void)raycastInfo;

	
	if (m_raycastInfo.m_isInContact)

	{
		double	project= m_raycastInfo.m_contactNormalWS.dot( m_raycastInfo.m_wheelDirectionWS );
		btVector3	 chassis_velocity_at_contactPoint;
		btVector3 relpos = m_raycastInfo.m_contactPointWS - chassis.getCenterOfMassPosition();
		chassis_velocity_at_contactPoint = chassis.getVelocityInLocalPoint( relpos );
		double projVel = m_raycastInfo.m_contactNormalWS.dot( chassis_velocity_at_contactPoint );
		if ( project >= (double)(-0.1))
		{
			m_suspensionRelativeVelocity = (double)(0.0);
			m_clippedInvContactDotSuspension = (double)(1.0) / (double)(0.1);
		}
		else
		{
			double inv = (double)(-1.) / project;
			m_suspensionRelativeVelocity = projVel * inv;
			m_clippedInvContactDotSuspension = inv;
		}
		
	}

	else	// Not in contact : position wheel in a nice (rest length) position
	{
		m_raycastInfo.m_suspensionLength = this.getSuspensionRestLength();
		m_suspensionRelativeVelocity = (double)(0.0);
		m_raycastInfo.m_contactNormalWS = -m_raycastInfo.m_wheelDirectionWS;
		m_clippedInvContactDotSuspension = (double)(1.0);
	}
}
