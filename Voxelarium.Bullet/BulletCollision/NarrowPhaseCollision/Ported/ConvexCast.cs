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

using Bullet.LinearMath;

namespace Bullet.Collision.NarrowPhase
{

	/// btConvexCast is an interface for Casting
	public abstract class btConvexCast
	{


		///RayResult stores the closest result
		/// alternatively, add a callback method to decide about closest/all results
		internal class CastResult
		{
			//virtual bool	addRayResult(ref btVector3 normal,double	fraction) = 0;

			public virtual void DebugDraw( double fraction ) { }
			public virtual void drawCoordSystem( ref btTransform trans ) { }
			public virtual void reportFailure( int errNo, int numIterations ) { }
			public CastResult()
			{
				m_fraction = ( (double)( btScalar.BT_LARGE_FLOAT ) );
				m_debugDrawer = null;
				m_allowedPenetration = ( (double)( 0 ) );
			}
			internal void Initialize()
			{
				m_fraction = btScalar.BT_LARGE_FLOAT;
				m_debugDrawer = null;
				m_allowedPenetration = 0;
			}

			internal btTransform m_hitTransformA;
			internal btTransform m_hitTransformB;
			internal btVector3 m_normal;
			internal btVector3 m_hitPoint;
			internal double m_fraction; //input and output
			internal btIDebugDraw m_debugDrawer;
			internal double m_allowedPenetration;
		};


		/// cast a convex against another convex object
		internal abstract bool calcTimeOfImpact(
						btITransform fromA,
						btITransform toA,
						btITransform fromB,
						btITransform toB,
						CastResult result );
	};

}
