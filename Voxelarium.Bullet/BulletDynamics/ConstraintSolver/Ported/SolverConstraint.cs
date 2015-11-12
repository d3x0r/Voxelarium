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

using Bullet.Dynamics.ConstraintSolver;
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.Dynamics
{

	//#define NO_FRICTION_TANGENTIALS 1


	internal delegate void useSolverConstraint( ref btSolverConstraint solver );

	///1D constraint along a normal axis between bodyA and bodyB. It can be combined to solve contact and friction constraints.
	///do want to keep these packed.
	public class btSolverConstraint
	{

		internal btVector3 m_relpos1CrossNormal;
		internal btVector3 m_contactNormal1;

		internal btVector3 m_relpos2CrossNormal;
		internal btVector3 m_contactNormal2; //usually m_contactNormal2 == -m_contactNormal1, but not always

		internal btVector3 m_angularComponentA;
		internal btVector3 m_angularComponentB;

		internal double m_appliedPushImpulse;
		internal double m_appliedImpulse;

		internal double m_friction;
		internal double m_jacDiagABInv;
		internal double m_rhs;
		internal double m_cfm;

		internal double m_lowerLimit;
		internal double m_upperLimit;
		internal double m_rhsPenetration;
		//union
		//{
		internal object m_originalContactPoint;
		//double m_unusedPadding4;
		//int m_numRowsForNonContactConstraint;
		//};

		internal int m_overrideNumSolverIterations;
		internal int m_frictionIndex;
		//internal int m_solverBodyIdA;
		//internal int m_solverBodyIdB;
		internal btSolverBody m_solverBodyA;
		internal btSolverBody m_solverBodyB;

		public void Clear()
		{
			m_relpos1CrossNormal = btVector3.Zero;
			m_contactNormal1 = btVector3.Zero;

			m_relpos2CrossNormal = btVector3.Zero;
			m_contactNormal2 = btVector3.Zero; //usually m_contactNormal2 == -m_contactNormal1, but not always

			m_angularComponentA = btVector3.Zero;
			m_angularComponentB = btVector3.Zero;

			m_appliedPushImpulse = 0;
			m_appliedImpulse = 0;

			m_friction = 0;
			m_jacDiagABInv = 0;
			m_rhs = 0;
			m_cfm = 0;

			m_lowerLimit = 0;
			m_upperLimit = 0;
			m_rhsPenetration = 0;
			//union
			//{
			m_originalContactPoint = null;
			//double m_unusedPadding4;
			//int m_numRowsForNonContactConstraint;
			//};

			m_overrideNumSolverIterations = 0;
			m_frictionIndex = 0;
			//m_solverBodyIdA = 0;
			//m_solverBodyIdB = 0;
			m_solverBodyA = null;
			m_solverBodyB = null;
		}

		enum btSolverConstraintType
		{
			BT_SOLVER_CONTACT_1D = 0,
			BT_SOLVER_FRICTION_1D
		};
	};

	public class btConstraintArray : btList<btSolverConstraint> { }
}



