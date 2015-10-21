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
namespace Bullet.Dynamics.ConstraintSolver
{

///btContactConstraint can be automatically created to solve contact constraints using the unified btTypedConstraint interface
internal class btContactConstraint : btTypedConstraint
{
protected:

	btPersistentManifold m_contactManifold;

public:


	btContactConstraint(btPersistentManifold* contactManifold,btRigidBody rbA,btRigidBody rbB);

	void	setContactManifold(btPersistentManifold* contactManifold);

	btPersistentManifold* getContactManifold()
	{
		return &m_contactManifold;
	}

	btPersistentManifold* getContactManifold()
	{
		return &m_contactManifold;
	}

	virtual ~btContactConstraint();

	virtual void getInfo1 (btConstraintInfo1* info);

	virtual void getInfo2 (btConstraintInfo2* info);

	///obsolete methods
	virtual void	buildJacobian();


};

///very basic collision resolution without friction
double resolveSingleCollision(btRigidBody body1, btCollisionObject colObj2, ref btVector3 contactPositionWorld,ref btVector3 contactNormalOnB, btruct btContactSolverInfo& solverInfo,double distance);


///resolveSingleBilateral is an obsolete methods used for vehicle friction between two dynamic objects
void resolveSingleBilateral(btRigidBody body1, ref btVector3 pos1,
                      btRigidBody body2, ref btVector3 pos2,
                      double distance, ref btVector3 normal,double impulse ,double timeStep);



}
