//#define DEBUG_PART_INDEX
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

using System.Diagnostics;
using Bullet.Collision.NarrowPhase;
using Bullet.LinearMath;

namespace Bullet.Collision.Dispatch
{


	delegate bool ContactAddedCallback( btManifoldPoint cp, btCollisionObjectWrapper colObj0Wrap, int partId0, int index0, btCollisionObjectWrapper colObj1Wrap, int partId1, int index1 );

	//#define DEBUG_PART_INDEX 1


	///btManifoldResult is a helper class to manage  contact results.
	internal class btManifoldResult : btDiscreteCollisionDetectorInterface.Result
	{
		static double MAX_FRICTION = (double)( 10.0 );
		static ContactAddedCallback gContactAddedCallback = null;

		internal btPersistentManifold m_manifoldPtr;

		internal btCollisionObjectWrapper m_body0Wrap;
		internal btCollisionObjectWrapper m_body1Wrap;
		protected int m_partId0;
		protected int m_partId1;
		protected int m_index0;
		protected int m_index1;



		public btManifoldResult()
		{
#if DEBUG_PART_INDEX
	m_partId0 = (-1);
	m_partId1 = (-1);
	m_index0 = (-1);
	m_index1 = (-1);
#endif //DEBUG_PART_INDEX
		}


		public void setPersistentManifold( btPersistentManifold manifoldPtr )
		{
			m_manifoldPtr = manifoldPtr;
		}

		//public btPersistentManifold getPersistentManifold()
		//{
		//	return m_manifoldPtr;
		//}

		public virtual void setShapeIdentifiersA( int partId0, int index0 )
		{
			m_partId0 = partId0;
			m_index0 = index0;
		}

		public virtual void setShapeIdentifiersB( int partId1, int index1 )
		{
			m_partId1 = partId1;
			m_index1 = index1;
		}

		public void refreshContactPoints()
		{
			Debug.Assert( m_manifoldPtr != null );
			if( m_manifoldPtr.m_cachedPoints == 0 )
				return;

			bool isSwapped = m_manifoldPtr.m_body0 != m_body0Wrap.m_collisionObject;

			if( isSwapped )
			{
				m_manifoldPtr.refreshContactPoints( ref m_body1Wrap.m_collisionObject.m_worldTransform, ref m_body0Wrap.m_collisionObject.m_worldTransform );
			}
			else
			{
				m_manifoldPtr.refreshContactPoints( ref m_body0Wrap.m_collisionObject.m_worldTransform, ref m_body1Wrap.m_collisionObject.m_worldTransform );
			}
		}

		//internal btCollisionObjectWrapper getBody0Wrap()
		//{
		//	return m_body0Wrap;
		//}
		//internal btCollisionObjectWrapper getBody1Wrap()
	//	{
		//	return m_body1Wrap;
		//}

		//internal void setBody0Wrap( btCollisionObjectWrapper obj0Wrap )
	//	{
	//		m_body0Wrap = obj0Wrap;
	//	}

		//internal void setBody1Wrap( btCollisionObjectWrapper obj1Wrap )
	//	{
	//		m_body1Wrap = obj1Wrap;
	//	}

		internal btCollisionObject getBody0Internal()
		{
			return m_body0Wrap.m_collisionObject;
		}

		internal btCollisionObject getBody1Internal()
		{
			return m_body1Wrap.m_collisionObject;
		}

		/// in the future we can let the user override the methods to combine restitution and friction

		///This is to allow MaterialCombiner/Custom Friction/Restitution values



		///User can override this material combiner by implementing gContactAddedCallback and setting body0.m_collisionFlags |= btCollisionObject::customMaterialCallback;
		public virtual double calculateCombinedRollingFriction( btCollisionObject body0, btCollisionObject body1 )
		{
			double friction = body0.m_rollingFriction * body1.m_rollingFriction;

			if( friction < -MAX_FRICTION )
				friction = -MAX_FRICTION;
			if( friction > MAX_FRICTION )
				friction = MAX_FRICTION;
			return friction;

		}


		///User can override this material combiner by implementing gContactAddedCallback and setting body0.m_collisionFlags |= btCollisionObject::customMaterialCallback;
		public static double calculateCombinedFriction( btCollisionObject body0, btCollisionObject body1 )
		{
			double friction = body0.getFriction() * body1.getFriction();

			if( friction < -MAX_FRICTION )
				friction = -MAX_FRICTION;
			if( friction > MAX_FRICTION )
				friction = MAX_FRICTION;
			return friction;

		}

		public static double calculateCombinedRestitution( btCollisionObject body0, btCollisionObject body1 )
		{
			return body0.getRestitution() * body1.getRestitution();
		}



		internal void Initialize( btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
		{
			m_manifoldPtr = null;
			m_body0Wrap = ( body0Wrap );
			m_body1Wrap = ( body1Wrap );
#if DEBUG_PART_INDEX
		m_partId0 = (-1);
    m_partId1 = (-1);
    m_index0 = (-1);
    m_index1 = (-1);
#endif //DEBUG_PART_INDEX
		}


		public virtual void addContactPoint( ref btVector3 normalOnBInWorld, ref btVector3 pointInWorld, double depth )
		{
			Debug.Assert( m_manifoldPtr != null );
			//order in manifold needs to match

			if( depth > m_manifoldPtr.getContactBreakingThreshold() )
				//	if (depth > m_manifoldPtr.getContactProcessingThreshold())
				return;

			bool isSwapped = m_manifoldPtr.m_body0 != m_body0Wrap.m_collisionObject;

			btVector3 pointA;// = pointInWorld + normalOnBInWorld * depth;
			pointInWorld.AddScale( ref normalOnBInWorld, depth, out pointA );

			btVector3 localA;
			btVector3 localB;

			if( isSwapped )
			{
				m_body1Wrap.m_collisionObject.m_worldTransform.invXform( ref pointA, out localA );
				m_body0Wrap.m_collisionObject.m_worldTransform.invXform( ref pointInWorld, out localB );
			}
			else
			{
				m_body0Wrap.m_collisionObject.m_worldTransform.invXform( ref pointA, out localA );
				m_body1Wrap.m_collisionObject.m_worldTransform.invXform( ref pointInWorld, out localB );
			}

			btManifoldPoint newPt = BulletGlobals.ManifoldPointPool.Get();
			newPt.Initialize( ref localA, ref localB, ref normalOnBInWorld, depth );

			newPt.m_positionWorldOnA = pointA;
			newPt.m_positionWorldOnB = pointInWorld;

			int insertIndex = m_manifoldPtr.getCacheEntry( ref newPt );
			btScalar.Dbg( "new point goes into cache at " + insertIndex );
			newPt.m_combinedFriction = calculateCombinedFriction( m_body0Wrap.m_collisionObject, m_body1Wrap.m_collisionObject );
			newPt.m_combinedRestitution = calculateCombinedRestitution( m_body0Wrap.m_collisionObject, m_body1Wrap.m_collisionObject );
			newPt.m_combinedRollingFriction = calculateCombinedRollingFriction( m_body0Wrap.m_collisionObject, m_body1Wrap.m_collisionObject );
			btVector3.btPlaneSpace1( ref newPt.m_normalWorldOnB, out newPt.m_lateralFrictionDir1, out newPt.m_lateralFrictionDir2 );



			//BP mod, store contact triangles.
			if( isSwapped )
			{
				newPt.m_partId0 = m_partId1;
				newPt.m_partId1 = m_partId0;
				newPt.m_index0 = m_index1;
				newPt.m_index1 = m_index0;
			}
			else
			{
				newPt.m_partId0 = m_partId0;
				newPt.m_partId1 = m_partId1;
				newPt.m_index0 = m_index0;
				newPt.m_index1 = m_index1;
			}
			//Console.WriteLine("depth=%f\n",depth);
			///@todo, check this for any side effects
			if( insertIndex >= 0 )
			{
				//btManifoldPoint oldPoint = m_manifoldPtr.getContactPoint(insertIndex);
				m_manifoldPtr.replaceContactPoint( newPt, insertIndex );
			}
			else
			{
				insertIndex = m_manifoldPtr.addManifoldPoint( newPt );
			}

			//User can override friction and/or restitution
			if( gContactAddedCallback != null &&
				 //and if either of the two bodies requires custom material
				 ( ( m_body0Wrap.m_collisionObject.getCollisionFlags() & btCollisionObject.CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK ) != 0 ||
				   ( m_body1Wrap.m_collisionObject.getCollisionFlags() & btCollisionObject.CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK ) != 0 ) )
			{
				//experimental feature info, for per-triangle material etc.
				btCollisionObjectWrapper obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
				btCollisionObjectWrapper obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
				gContactAddedCallback( m_manifoldPtr.getContactPoint( insertIndex ), obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1, newPt.m_index1 );
			}

		}


	};

}
