#define MAINTAIN_PERSISTENCY
#define KEEP_DEEPEST_POINT
#define DEBUG_PERSISTENCY
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
using System.Diagnostics;
using Bullet.Collision.Dispatch;
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.Collision.NarrowPhase
{


	delegate bool ContactDestroyedCallback( object userPersistentData );
	delegate bool ContactProcessedCallback( btManifoldPoint cp, object body0, object body1 );

	public class btPersistentManifold : btTypedObject
	{
		const int MANIFOLD_CACHE_SIZE = 4;

		public const double gContactBreakingThreshold = (double)( 0.02 );
		static ContactDestroyedCallback gContactDestroyedCallback;
		static ContactProcessedCallback gContactProcessedCallback;
		///gContactCalcArea3Points will approximate the convex hull area using 3 points
		///when setting it to false, it will use 4 points to compute the area: it is more accurate but slower
		bool gContactCalcArea3Points = true;

		btManifoldPoint[] m_pointCache = new btManifoldPoint[MANIFOLD_CACHE_SIZE];

		/// this two body pointers can point to the physics rigidbody class.
		btCollisionObject m_body0;
		btCollisionObject m_body1;

		int m_cachedPoints;

		double m_contactBreakingThreshold;
		double m_contactProcessingThreshold;


		internal int m_companionIdA;
		internal int m_companionIdB;

		internal int m_index1a;

		public btPersistentManifold() { }

		internal void Initialize(btCollisionObject body0,btCollisionObject body1,int unused, double contactBreakingThreshold, double contactProcessingThreshold )
		{
			base.Initialize(  btObjectTypes.BT_PERSISTENT_MANIFOLD_TYPE );
			m_body0 = ( body0 ); m_body1 = ( body1 ); m_cachedPoints = ( 0 );
			m_contactBreakingThreshold = ( contactBreakingThreshold );
			m_contactProcessingThreshold = ( contactProcessingThreshold );
        }

		internal void Initialize()
		{
			base.Initialize( btObjectTypes.BT_PERSISTENT_MANIFOLD_TYPE );
            m_body0 = null;
			m_body1 = null;
			m_cachedPoints = ( 0 );
			m_index1a = ( 0 );

		}

#if DEBUG_PERSISTENCY
		void DebugPersistency()
		{
			int i;
			Console.WriteLine( "DebugPersistency : numPoints %d\n", m_cachedPoints );
			for( i = 0; i < m_cachedPoints; i++ )
			{
				Console.WriteLine( "m_pointCache[%d].m_userPersistentData = %x\n", i, m_pointCache[i].m_userPersistentData );
			}
		}
#endif //DEBUG_PERSISTENCY

		void clearUserCache( btManifoldPoint pt )
		{
			object oldPtr = pt.m_userPersistentData;
			if( oldPtr != null )
			{
#if DEBUG_PERSISTENCY
				int i;
				int occurance = 0;
				for( i = 0; i < m_cachedPoints; i++ )
				{
					if( m_pointCache[i].m_userPersistentData == oldPtr )
					{
						occurance++;
						if( occurance > 1 )
							Console.WriteLine( "error in clearUserCache\n" );
					}
				}
				Debug.Assert( occurance <= 0 );
#endif //DEBUG_PERSISTENCY

				if( pt.m_userPersistentData != null && gContactDestroyedCallback != null )
				{
					gContactDestroyedCallback( pt.m_userPersistentData );
					pt.m_userPersistentData = null;
				}

#if DEBUG_PERSISTENCY
				DebugPersistency();
#endif
			}


		}

		static double calcArea4Points( ref btVector3 p0, ref btVector3 p1, ref btVector3 p2, ref btVector3 p3 )
		{
			// It calculates possible 3 area constructed from random 4 points and returns the biggest one.
			btVector3 a1, a2, a3;
			btVector3 b1, b2, b3;
			p0.Sub( ref p1, out a1 );
			p0.Sub( ref p2, out a2 );
			p0.Sub( ref p3, out a3 );
			p2.Sub( ref p3, out b1 );
			p1.Sub( ref p2, out b2 );
			p1.Sub( ref p3, out b3 );

			//todo: Following 3 cross production can be easily optimized by SIMD.
			btVector3 tmp0; a1.cross( ref b1, out tmp0 );
			btVector3 tmp1; a2.cross( ref b2, out tmp1 );
			btVector3 tmp2; a3.cross( ref b3, out tmp2 );

			return btScalar.btMax( btScalar.btMax( tmp0.length2(), tmp1.length2() ), tmp2.length2() );
		}

		/// sort cached points so most isolated points come first
		int sortCachedPoints( btManifoldPoint pt )
		{
			//calculate 4 possible cases areas, and take biggest area
			//also need to keep 'deepest'

			int maxPenetrationIndex = -1;
#if KEEP_DEEPEST_POINT
			double maxPenetration = pt.getDistance();
			for( int i = 0; i < 4; i++ )
			{
				if( m_pointCache[i].getDistance() < maxPenetration )
				{
					maxPenetrationIndex = i;
					maxPenetration = m_pointCache[i].getDistance();
				}
			}
#endif //KEEP_DEEPEST_POINT

			double res0 = ( 0 )
				, res1 = ( (double)( 0 ) ), res2 = ( (double)( 0 ) )
				, res3 = ( (double)( 0 ) );

			if( gContactCalcArea3Points )
			{
				if( maxPenetrationIndex != 0 )
				{
					btVector3 a0; pt.m_localPointA.Sub( ref m_pointCache[1].m_localPointA, out a0 );
					btVector3 b0; m_pointCache[3].m_localPointA.Sub( ref m_pointCache[2].m_localPointA, out b0 );
					btVector3 cross; a0.cross( ref b0, out cross );
					res0 = cross.length2();
				}
				if( maxPenetrationIndex != 1 )
				{
					btVector3 a1; pt.m_localPointA.Sub( ref m_pointCache[0].m_localPointA, out a1 );
					btVector3 b1; m_pointCache[3].m_localPointA.Sub( ref m_pointCache[2].m_localPointA, out b1 );
					btVector3 cross; a1.cross( ref b1, out cross );
					res1 = cross.length2();
				}

				if( maxPenetrationIndex != 2 )
				{
					btVector3 a2; pt.m_localPointA.Sub( ref m_pointCache[0].m_localPointA, out a2 );
					btVector3 b2; m_pointCache[3].m_localPointA.Sub( ref m_pointCache[1].m_localPointA, out b2 );
					btVector3 cross; a2.cross( ref b2, out cross );
					res2 = cross.length2();
				}

				if( maxPenetrationIndex != 3 )
				{
					btVector3 a3; pt.m_localPointA.Sub( ref m_pointCache[0].m_localPointA, out a3 );
					btVector3 b3; m_pointCache[2].m_localPointA.Sub( ref m_pointCache[1].m_localPointA, out b3 );
					btVector3 cross; a3.cross( ref b3, out cross );
					res3 = cross.length2();
				}
			}
			else
			{
				if( maxPenetrationIndex != 0 )
				{
					res0 = calcArea4Points( ref pt.m_localPointA, ref m_pointCache[1].m_localPointA, ref m_pointCache[2].m_localPointA, ref m_pointCache[3].m_localPointA );
				}

				if( maxPenetrationIndex != 1 )
				{
					res1 = calcArea4Points( ref pt.m_localPointA, ref m_pointCache[0].m_localPointA, ref m_pointCache[2].m_localPointA, ref m_pointCache[3].m_localPointA );
				}

				if( maxPenetrationIndex != 2 )
				{
					res2 = calcArea4Points( ref pt.m_localPointA, ref m_pointCache[0].m_localPointA, ref m_pointCache[1].m_localPointA, ref m_pointCache[3].m_localPointA );
				}

				if( maxPenetrationIndex != 3 )
				{
					res3 = calcArea4Points( ref pt.m_localPointA, ref m_pointCache[0].m_localPointA, ref m_pointCache[1].m_localPointA, ref m_pointCache[2].m_localPointA );
				}
			}
			btVector4 maxvec = new btVector4( res0, res1, res2, res3 );
			return maxvec.closestAxis4();
		}


		internal int getCacheEntry( ref btManifoldPoint newPoint )
		{
			double shortestDist = getContactBreakingThreshold() * getContactBreakingThreshold();
			int size = getNumContacts();
			int nearestPoint = -1;
			for( int i = 0; i < size; i++ )
			{
				btManifoldPoint mp = m_pointCache[i];

				btVector3 diffA; mp.m_localPointA.Sub( ref newPoint.m_localPointA, out diffA );
				double distToManiPoint = diffA.dot( ref diffA );
				if( distToManiPoint < shortestDist )
				{
					shortestDist = distToManiPoint;
					nearestPoint = i;
				}
			}
			return nearestPoint;
		}

		internal int addManifoldPoint( btManifoldPoint newPoint, bool isPredictive = false )
		{
			if( !isPredictive )
			{
				Debug.Assert( validContactDistance( newPoint ) );
			}

			int insertIndex = getNumContacts();
			if( insertIndex == MANIFOLD_CACHE_SIZE )
			{
				if( MANIFOLD_CACHE_SIZE >= 4 )
					//sort cache so best points come first, based on area
					insertIndex = sortCachedPoints( newPoint );
				else
					insertIndex = 0;
				clearUserCache( m_pointCache[insertIndex] );

			}
			else
			{
				m_cachedPoints++;


			}
			if( insertIndex < 0 )
				insertIndex = 0;

			Debug.Assert( m_pointCache[insertIndex].m_userPersistentData == null );
			m_pointCache[insertIndex] = newPoint;
			return insertIndex;
		}

		public double getContactBreakingThreshold()
		{
			return m_contactBreakingThreshold;
		}



		/// calculated new worldspace coordinates and depth, and reject points that exceed the collision margin
		public void refreshContactPoints( ref btTransform trA, ref btTransform trB )
		{
			int i;
#if DEBUG_PERSISTENCY
			Console.WriteLine( "refreshContactPoints posA = ("
				+ trA.m_origin.x
				+ ","
				+ trA.m_origin.y
				+ ","
				+ trA.m_origin.z
				+ ") posB = ("
				+ trB.m_origin.x
				+ ","
				+ trB.m_origin.y
				+ ","
				+ trB.m_origin.z
				+ ")\n" );
#endif //DEBUG_PERSISTENCY
			/// first refresh worldspace positions and distance
			for( i = getNumContacts() - 1; i >= 0; i-- )
			{
				btManifoldPoint manifoldPoint = m_pointCache[i];
				trA.Apply( ref manifoldPoint.m_localPointA, out manifoldPoint.m_positionWorldOnA );
				trB.Apply( ref manifoldPoint.m_localPointB, out manifoldPoint.m_positionWorldOnB );
				btVector3 tmp;
				manifoldPoint.m_positionWorldOnA.Sub( ref manifoldPoint.m_positionWorldOnB, out tmp );
				manifoldPoint.m_distance1 = tmp.dot( ref manifoldPoint.m_normalWorldOnB );
				manifoldPoint.m_lifeTime++;
			}

			/// then 
			double distance2d;
			btVector3 projectedDifference, projectedPoint;
			for( i = getNumContacts() - 1; i >= 0; i-- )
			{

				btManifoldPoint manifoldPoint = m_pointCache[i];
				//contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
				if( !validContactDistance( manifoldPoint ) )
				{
					removeContactPoint( i );
				}
				else
				{
					//contact also becomes invalid when relative movement orthogonal to normal exceeds margin
					manifoldPoint.m_positionWorldOnA.SubScale( ref manifoldPoint.m_normalWorldOnB, manifoldPoint.m_distance1, out projectedPoint );
					manifoldPoint.m_positionWorldOnB.Sub( ref projectedPoint, out projectedDifference );
					distance2d = projectedDifference.dot( ref projectedDifference );
					if( distance2d > getContactBreakingThreshold() * getContactBreakingThreshold() )
					{
						removeContactPoint( i );
					}
					else
					{
						//contact point processed callback
						if( gContactProcessedCallback != null )
							gContactProcessedCallback( manifoldPoint, m_body0, m_body1 );
					}
				}
			}
		}
		//the enum starts at 1024 to avoid type conflicts with btTypedConstraint


		///btPersistentManifold is a contact point cache, it stays persistent as long as objects are overlapping in the broadphase.
		///Those contact points are created by the collision narrow phase.
		///The cache can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might only add one point at a time.
		///updates/refreshes old contact points, and throw them away if necessary (distance becomes too large)
		///reduces the cache to 4 points, when more then 4 points are added, using following rules:
		///the contact point with deepest penetration is always kept, and it tries to maximuze the area covered by the points
		///note that some pairs of objects might have more then one contact manifold.


		public btCollisionObject getBody0() { return m_body0; }
		public btCollisionObject getBody1() { return m_body1; }

		void setBodies( btCollisionObject body0, btCollisionObject body1 )
		{
			m_body0 = body0;
			m_body1 = body1;
		}


		public int getNumContacts() { return m_cachedPoints; }
		/// the setNumContacts API is usually not used, except when you gather/fill all contacts manually
		void setNumContacts( int cachedPoints )
		{
			m_cachedPoints = cachedPoints;
		}


		internal btManifoldPoint getContactPoint( int index )
		{
			Debug.Assert( index < m_cachedPoints );
			return m_pointCache[index];
		}

		///@todo: get this margin from the current physics / collision environment
		internal double getContactProcessingThreshold()
		{
			return m_contactProcessingThreshold;
		}

		void setContactBreakingThreshold( double contactBreakingThreshold )
		{
			m_contactBreakingThreshold = contactBreakingThreshold;
		}

		void setContactProcessingThreshold( double contactProcessingThreshold )
		{
			m_contactProcessingThreshold = contactProcessingThreshold;
		}





		void removeContactPoint( int index )
		{
			clearUserCache( m_pointCache[index] );

			int lastUsedIndex = getNumContacts() - 1;
			//		m_pointCache[index] = m_pointCache[lastUsedIndex];
			if( index != lastUsedIndex )
			{
				m_pointCache[index] = m_pointCache[lastUsedIndex];
				//get rid of duplicated userPersistentData pointer
				m_pointCache[lastUsedIndex].m_userPersistentData = 0;
				m_pointCache[lastUsedIndex].m_appliedImpulse = 0;
				m_pointCache[lastUsedIndex].m_lateralFrictionInitialized = false;
				m_pointCache[lastUsedIndex].m_appliedImpulseLateral1 = 0;
				m_pointCache[lastUsedIndex].m_appliedImpulseLateral2 = 0;
				m_pointCache[lastUsedIndex].m_lifeTime = 0;
			}

			Debug.Assert( m_pointCache[lastUsedIndex].m_userPersistentData == null );
			m_cachedPoints--;
		}

		internal void replaceContactPoint( btManifoldPoint newPoint, int insertIndex )
		{
			Debug.Assert( validContactDistance( newPoint ) );

#if MAINTAIN_PERSISTENCY
			int lifeTime = m_pointCache[insertIndex].getLifeTime();
			double appliedImpulse = m_pointCache[insertIndex].m_appliedImpulse;
			double appliedLateralImpulse1 = m_pointCache[insertIndex].m_appliedImpulseLateral1;
			double appliedLateralImpulse2 = m_pointCache[insertIndex].m_appliedImpulseLateral2;
			//		bool isLateralFrictionInitialized = m_pointCache[insertIndex].m_lateralFrictionInitialized;



			Debug.Assert( lifeTime >= 0 );
			object cache = m_pointCache[insertIndex].m_userPersistentData;

			m_pointCache[insertIndex] = newPoint;

			m_pointCache[insertIndex].m_userPersistentData = cache;
			m_pointCache[insertIndex].m_appliedImpulse = appliedImpulse;
			m_pointCache[insertIndex].m_appliedImpulseLateral1 = appliedLateralImpulse1;
			m_pointCache[insertIndex].m_appliedImpulseLateral2 = appliedLateralImpulse2;

			m_pointCache[insertIndex].m_appliedImpulse = appliedImpulse;
			m_pointCache[insertIndex].m_appliedImpulseLateral1 = appliedLateralImpulse1;
			m_pointCache[insertIndex].m_appliedImpulseLateral2 = appliedLateralImpulse2;


			m_pointCache[insertIndex].m_lifeTime = lifeTime;
#else
	clearUserCache( m_pointCache[insertIndex] );
	m_pointCache[insertIndex] = newPoint;

#endif
		}


		bool validContactDistance( btManifoldPoint pt )
		{
			return pt.m_distance1 <= getContactBreakingThreshold();
		}


		public void clearManifold()
		{
			int i;
			for( i = 0; i < m_cachedPoints; i++ )
			{
				clearUserCache( m_pointCache[i] );
			}
			m_cachedPoints = 0;
		}

	}

}
