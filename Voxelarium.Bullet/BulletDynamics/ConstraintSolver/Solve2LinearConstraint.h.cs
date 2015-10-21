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

	/// constraint class used for lateral tyre friction.
	class btSolve2LinearConstraint
	{
		double m_tau;
		double m_damping;

		public:

	btSolve2LinearConstraint( double tau, double damping )
		{
			m_tau = tau;
			m_damping = damping;
		}
		//
		// solve unilateral constraint (equality, direct method)
		//
		void resolveUnilateralPairConstraint(
															   btRigidBody body0,
			btRigidBody body1,

			btMatrix3x3& world2A,
							btMatrix3x3& world2B,

							ref btVector3 invInertiaADiag,
							double invMassA,
							ref btVector3 linvelA, ref btVector3 angvelA,
							ref btVector3 rel_posA1,
							ref btVector3 invInertiaBDiag,
							double invMassB,
							ref btVector3 linvelB, ref btVector3 angvelB,
							ref btVector3 rel_posA2,

						  double depthA, ref btVector3 normalA,
						  ref btVector3 rel_posB1, ref btVector3 rel_posB2,
						  double depthB, ref btVector3 normalB,
						  double imp0, double imp1 );


		//
		// solving 2x2 lcp problem (inequality, direct solution )
		//
		void resolveBilateralPairConstraint(
				btRigidBody body0,
							btRigidBody body1,
			btMatrix3x3& world2A,
							btMatrix3x3& world2B,

							ref btVector3 invInertiaADiag,
							double invMassA,
							ref btVector3 linvelA, ref btVector3 angvelA,
							ref btVector3 rel_posA1,
							ref btVector3 invInertiaBDiag,
							double invMassB,
							ref btVector3 linvelB, ref btVector3 angvelB,
							ref btVector3 rel_posA2,

						  double depthA, ref btVector3 normalA,
						  ref btVector3 rel_posB1, ref btVector3 rel_posB2,
						  double depthB, ref btVector3 normalB,
						  double imp0, double imp1 );

		/*
			void resolveAngularConstraint(	btMatrix3x3& invInertiaAWS,
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
							  double imp0,double imp1);

		*/

	};

}
