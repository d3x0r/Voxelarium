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

using Bullet.Collision.BroadPhase;
using Bullet.Collision.Dispatch;
using Bullet.Collision.NarrowPhase;
using Bullet.LinearMath;

namespace Bullet.Dynamics.ConstraintSolver
{

	/// btConstraintSolver provides solver interface


	public enum btConstraintSolverType
	{
		BT_SEQUENTIAL_IMPULSE_SOLVER = 1,
		BT_MLCP_SOLVER = 2,
		BT_NNCG_SOLVER = 4
	};

	public abstract class btConstraintSolver
	{

		internal abstract void prepareSolve( int numBodies, int numManifolds );

		///solve a group of constraints
		internal abstract double solveGroup( btCollisionObject[] bodies, int numBodies
					, btPersistentManifold[] manifold, int first_manifold, int numManifolds
					, btTypedConstraint[] constraints, int startConstraint, int numConstraints
					, btContactSolverInfo info
					, btIDebugDraw debugDrawer
					, btDispatcher dispatcher );

		internal abstract void allSolved( btContactSolverInfo info, btIDebugDraw debugDrawer );

		///clear internal cached data and reset random seed
		internal abstract void reset();

		internal abstract btConstraintSolverType getSolverType();


	};


}
