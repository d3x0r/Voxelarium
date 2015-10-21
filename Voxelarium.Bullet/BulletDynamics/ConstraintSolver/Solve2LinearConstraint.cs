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



#include "btSolve2LinearConstraint.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"


void btSolve2LinearConstraint::resolveUnilateralPairConstraint(
												   btRigidBody body1,
		btRigidBody body2,

						btMatrix3x3& world2A,
						btMatrix3x3& world2B,
						
						ref btVector3 invInertiaADiag,
						double invMassA,
						ref btVector3 linvelA,ref btVector3 angvelA,
						ref btVector3 rel_posA1,
						ref btVector3 invInertiaBDiag,
						double invMassB,
						ref btVector3 linvelB,ref btVector3 angvelB,
						ref btVector3 rel_posA2,

					  double depthA, ref btVector3 normalA, 
					  ref btVector3 rel_posB1,ref btVector3 rel_posB2,
					  double depthB, ref btVector3 normalB, 
					  double imp0,double imp1)
{
	(void)linvelA;
	(void)linvelB;
	(void)angvelB;
	(void)angvelA;



	imp0 = btScalar.BT_ZERO;
	imp1 = btScalar.BT_ZERO;

	double len = btFabs(normalA.length()) - btScalar.BT_ONE;
	if (btFabs(len) >= SIMD_EPSILON)
		return;

	Debug.Assert(len < SIMD_EPSILON);


	//this jacobian entry could be re-used for all iterations
	btJacobianEntry jacA(world2A,world2B,rel_posA1,rel_posA2,normalA,invInertiaADiag,invMassA,
		invInertiaBDiag,invMassB);
	btJacobianEntry jacB(world2A,world2B,rel_posB1,rel_posB2,normalB,invInertiaADiag,invMassA,
		invInertiaBDiag,invMassB);
	
	//double vel0 = jacA.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);
	//double vel1 = jacB.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);

	double vel0 = normalA.dot(body1.getVelocityInLocalPoint(rel_posA1)-body2.getVelocityInLocalPoint(rel_posA1));
	double vel1 = normalB.dot(body1.getVelocityInLocalPoint(rel_posB1)-body2.getVelocityInLocalPoint(rel_posB1));

//	double penetrationImpulse = (depth*contactTau*timeCorrection)  * massTerm;//jacDiagABInv
	double massTerm = btScalar.BT_ONE / (invMassA + invMassB);


	// calculate rhs (or error) terms
	double dv0 = depthA  * m_tau * massTerm - vel0 * m_damping;
	double dv1 = depthB  * m_tau * massTerm - vel1 * m_damping;


	// dC/dv * dv = -C
	
	// jacobian * impulse = -error
	//

	//impulse = jacobianInverse * -error

	// inverting 2x2 symmetric system (offdiagonal are equal!)
	// 


	double nonDiag = jacA.getNonDiagonal(jacB,invMassA,invMassB);
	double	invDet = (double)(1.0) / (jacA.getDiagonal() * jacB.getDiagonal() - nonDiag * nonDiag );
	
	//imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
	//imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

	imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
	imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

	//[a b]								  [d -c]
	//[c d] inverse = (1 / determinant) * [-b a] where determinant is (ad - bc)

	//[jA nD] * [imp0] = [dv0]
	//[nD jB]   [imp1]   [dv1]

}



void btSolve2LinearConstraint::resolveBilateralPairConstraint(
						btRigidBody body1,
						btRigidBody body2,
						btMatrix3x3& world2A,
						btMatrix3x3& world2B,
						
						ref btVector3 invInertiaADiag,
						double invMassA,
						ref btVector3 linvelA,ref btVector3 angvelA,
						ref btVector3 rel_posA1,
						ref btVector3 invInertiaBDiag,
						double invMassB,
						ref btVector3 linvelB,ref btVector3 angvelB,
						ref btVector3 rel_posA2,

					  double depthA, ref btVector3 normalA, 
					  ref btVector3 rel_posB1,ref btVector3 rel_posB2,
					  double depthB, ref btVector3 normalB, 
					  double imp0,double imp1)
{

	(void)linvelA;
	(void)linvelB;
	(void)angvelA;
	(void)angvelB;



	imp0 = btScalar.BT_ZERO;
	imp1 = btScalar.BT_ZERO;

	double len = btFabs(normalA.length()) - btScalar.BT_ONE;
	if (btFabs(len) >= SIMD_EPSILON)
		return;

	Debug.Assert(len < SIMD_EPSILON);


	//this jacobian entry could be re-used for all iterations
	btJacobianEntry jacA(world2A,world2B,rel_posA1,rel_posA2,normalA,invInertiaADiag,invMassA,
		invInertiaBDiag,invMassB);
	btJacobianEntry jacB(world2A,world2B,rel_posB1,rel_posB2,normalB,invInertiaADiag,invMassA,
		invInertiaBDiag,invMassB);
	
	//double vel0 = jacA.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);
	//double vel1 = jacB.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);

	double vel0 = normalA.dot(body1.getVelocityInLocalPoint(rel_posA1)-body2.getVelocityInLocalPoint(rel_posA1));
	double vel1 = normalB.dot(body1.getVelocityInLocalPoint(rel_posB1)-body2.getVelocityInLocalPoint(rel_posB1));

	// calculate rhs (or error) terms
	double dv0 = depthA  * m_tau - vel0 * m_damping;
	double dv1 = depthB  * m_tau - vel1 * m_damping;

	// dC/dv * dv = -C
	
	// jacobian * impulse = -error
	//

	//impulse = jacobianInverse * -error

	// inverting 2x2 symmetric system (offdiagonal are equal!)
	// 


	double nonDiag = jacA.getNonDiagonal(jacB,invMassA,invMassB);
	double	invDet = (double)(1.0) / (jacA.getDiagonal() * jacB.getDiagonal() - nonDiag * nonDiag );
	
	//imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
	//imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

	imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
	imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

	//[a b]								  [d -c]
	//[c d] inverse = (1 / determinant) * [-b a] where determinant is (ad - bc)

	//[jA nD] * [imp0] = [dv0]
	//[nD jB]   [imp1]   [dv1]

	if ( imp0 > (double)(0.0))
	{
		if ( imp1 > (double)(0.0) )
		{
			//both positive
		}
		else
		{
			imp1 = btScalar.BT_ZERO;

			// now imp0>0 imp1<0
			imp0 = dv0 / jacA.getDiagonal();
			if ( imp0 > (double)(0.0) )
			{
			} else
			{
				imp0 = btScalar.BT_ZERO;
			}
		}
	}
	else
	{
		imp0 = btScalar.BT_ZERO;

		imp1 = dv1 / jacB.getDiagonal();
		if ( imp1 <= (double)(0.0) )
		{
			imp1 = btScalar.BT_ZERO;
			// now imp0>0 imp1<0
			imp0 = dv0 / jacA.getDiagonal();
			if ( imp0 > (double)(0.0) )
			{
			} else
			{
				imp0 = btScalar.BT_ZERO;
			}
		} else
		{
		}
	}
}


/*
void btSolve2LinearConstraint::resolveAngularConstraint(	btMatrix3x3& invInertiaAWS,
											double invMassA,
											ref btVector3 linvelA,ref btVector3 angvelA,
											ref btVector3 rel_posA1,
											btMatrix3x3& invInertiaBWS,
											double invMassB,
											ref btVector3 linvelB,ref btVector3 angvelB,
											ref btVector3 rel_posA2,

											double depthA, ref btVector3 normalA, 
											ref btVector3 rel_posB1,ref btVector3 rel_posB2,
											double depthB, ref btVector3 normalB, 
											double imp0,double imp1)
{

}
*/

