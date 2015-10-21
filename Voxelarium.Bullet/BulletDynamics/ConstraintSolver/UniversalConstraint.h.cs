/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#if ! BT_UNIVERSAL_CONSTRAINT_H
#define BT_UNIVERSAL_CONSTRAINT_H



#include "LinearMath/btVector3.h"
#include "btTypedConstraint.h"
#include "btGeneric6DofConstraint.h"



/// Constraint similar to ODE Universal Joint
/// has 2 rotatioonal degrees of freedom, similar to Euler rotations around Z (axis 1)
/// and Y (axis 2)
/// Description from ODE manual : 
/// "Given axis 1 on body 1, and axis 2 on body 2 that is perpendicular to axis 1, it keeps them perpendicular. 
/// In other words, rotation of the two bodies about the direction perpendicular to the two axes will be equal."

internal class btUniversalConstraint : btGeneric6DofConstraint
{
protected:
	btVector3	m_anchor;
	btVector3	m_axis1;
	btVector3	m_axis2;
public:
	
	
	
	// constructor
	// anchor, axis1 and axis2 are in world coordinate system
	// axis1 must be orthogonal to axis2
    btUniversalConstraint(btRigidBody rbA, btRigidBody rbB, ref btVector3 anchor, ref btVector3 axis1, ref btVector3 axis2);
	// access
	ref btVector3 getAnchor() { return m_calculatedTransformA.getOrigin(); }
	ref btVector3 getAnchor2() { return m_calculatedTransformB.getOrigin(); }
	ref btVector3 getAxis1() { return m_axis1; }
	ref btVector3 getAxis2() { return m_axis2; }
	double getAngle1() { return getAngle(2); }
	double getAngle2() { return getAngle(1); }
	// limits
	void setUpperLimit(double ang1max, double ang2max) { setAngularUpperLimit(btVector3(0, ang1max, ang2max)); }
	void setLowerLimit(double ang1min, double ang2min) { setAngularLowerLimit(btVector3(0, ang1min, ang2min)); }

	void setAxis( ref btVector3 axis1, ref btVector3 axis2);
};



#endif // BT_UNIVERSAL_CONSTRAINT_H

