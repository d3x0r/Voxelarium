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

using System;
using Bullet.LinearMath;
using Bullet.Collision.Dispatch;

namespace Bullet.Collision.NarrowPhase
{

	/// This interface is made to be used by an iterative approach to do TimeOfImpact calculations
	/// This interface allows to query for closest points and penetration depth between two (convex) objects
	/// the closest point is on the second object (B), and the normal points from the surface on B towards A.
	/// distance is between closest points on B and closest point on A. So you can calculate closest point on A
	/// by taking closestPointInA = closestPointInB + m_distance * m_normalOnSurfaceB
	public abstract class btDiscreteCollisionDetectorInterface
	{

		internal interface Result
		{
			///setShapeIdentifiersA/B provides experimental support for per-triangle material / custom material combiner
			void setShapeIdentifiersA( int partId0, int index0 );
			void setShapeIdentifiersB( int partId1, int index1 );
			void addContactPoint(  ref btVector3 normalOnBInWorld, ref btVector3 pointInWorld, double depth );
		};

		internal class ClosestPointInput
		{
			public ClosestPointInput()
			{
				m_maximumDistanceSquared = btScalar.BT_LARGE_FLOAT;
			}

			public void Initialize()
			{
				m_maximumDistanceSquared = btScalar.BT_LARGE_FLOAT;
			}
			//internal btCollisionObjectWrapper objA;
			//internal btCollisionObjectWrapper objB;
			internal btTransform m_transformA;
			internal btTransform m_transformB;
			internal double m_maximumDistanceSquared;
		};

		//
		// give either closest points (distance > 0) or penetration (distance)
		// the normal always points from B towards A
		//
		internal abstract void getClosestPoints( ClosestPointInput input, Result output, btIDebugDraw debugDraw
						, bool swapResults = false );

	};

	public class btStorageResult : btDiscreteCollisionDetectorInterface.Result
	{
		btVector3 m_normalOnSurfaceB;
		btVector3 m_closestPointInB;
		double m_distance; //negative means penetration !

		public btStorageResult()
		{
			m_distance = btScalar.BT_LARGE_FLOAT;
        }

		public virtual void addContactPoint( ref btVector3 normalOnBInWorld, ref btVector3 pointInWorld, double depth )
		{
			if( depth < m_distance )
			{
				m_normalOnSurfaceB = normalOnBInWorld;
				m_closestPointInB = pointInWorld;
				m_distance = depth;
			}
		}

		void btDiscreteCollisionDetectorInterface.Result.setShapeIdentifiersA( int partId0, int index0 )
		{
			throw new NotImplementedException();
		}

		void btDiscreteCollisionDetectorInterface.Result.setShapeIdentifiersB( int partId1, int index1 )
		{
			throw new NotImplementedException();
		}

		void btDiscreteCollisionDetectorInterface.Result.addContactPoint( ref btVector3 normalOnBInWorld, ref btVector3 pointInWorld, double depth )
		{
			throw new NotImplementedException();
		}
	};

}

