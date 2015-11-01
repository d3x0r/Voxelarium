/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

EPA Copyright (c) Ricardo Padrela 2006 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using Bullet.Collision.Shapes;
using Bullet.LinearMath;

namespace Bullet.Collision.NarrowPhase
{

	///EpaPenetrationDepthSolver uses the Expanding Polytope Algorithm to
	///calculate the penetration depth between two convex shapes.
	internal class btGjkEpaPenetrationDepthSolver: btConvexPenetrationDepthSolver
	{

		 // this is a 

		internal override bool calcPenDepth( btSimplexSolverInterface simplexSolver,
											btConvexShape pConvexA, btConvexShape pConvexB,
											ref btTransform transformA, ref btTransform transformB,
											ref btVector3 v, out btVector3 wWitnessOnA, out btVector3 wWitnessOnB,
											btIDebugDraw debugDraw )
		{

			//(void)debugDraw;
			//(void)v;
			//(void)simplexSolver;

			//	double				radialmargin(btScalar.BT_ZERO);

			btVector3 guessVector; transformB.m_origin.Sub( ref transformA.m_origin, out guessVector );
			btGjkEpaSolver2.sResults results;


			if( btGjkEpaSolver2.Penetration( pConvexA, ref transformA,
										pConvexB, ref transformB,
										ref guessVector, out results ) )

			{
				//	debugDraw.drawLine(results.witnesses[1],results.witnesses[1]+results.normal,btVector3(255,0,0));
				//resultOut.addContactPoint(results.normal,results.witnesses[1],-results.depth);
				wWitnessOnA = results.witness0;
				wWitnessOnB = results.witness1;
				v = results.normal;
				return true;
			}
			else
			{
				if( btGjkEpaSolver2.Distance( pConvexA, ref transformA, pConvexB, ref transformB, ref guessVector, out results ) )
				{
					wWitnessOnA = results.witness0;
					wWitnessOnB = results.witness1;
					v = results.normal;
					return false;
				}
			}
			wWitnessOnA = results.witness0;
			wWitnessOnB = results.witness1;
			return false;
		}



	};

}
