using Bullet.LinearMath;
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
namespace Bullet.Collision.NarrowPhase
{

	/// ManifoldContactPoint collects and maintains persistent contactpoints.
	/// used to improve stability and performance of rigidbody dynamics response.

	internal class btManifoldPoint
	{
		public btManifoldPoint()
		{
		}

		internal void Initialize( ref btVector3 pointA, ref btVector3 pointB,
				ref btVector3 normal,
				double distance )
		{
			m_lateralFrictionDir1 = btVector3.Zero;
			m_lateralFrictionDir2 = btVector3.Zero;
			m_lifeTime = 0;
			m_appliedImpulseLateral1 = 0f;
			m_appliedImpulseLateral2 = 0f;
			m_contactMotion1 = 0f;
			m_contactMotion2 = 0f;
			m_contactCFM1 = 0f;
			m_contactCFM2 = 0f;

			m_lateralFrictionInitialized = false;
			m_userPersistentData = null;
			m_appliedImpulse = 0f;
			m_partId0 = 0;
			m_partId1 = 0;
			m_index0 = 0;
			m_index1 = 0;
			m_combinedRestitution = 0f;
			m_combinedFriction = 0f;
			m_positionWorldOnA = btVector3.Zero;
			m_positionWorldOnB = btVector3.Zero;

			m_localPointA = pointA;
			m_localPointB = pointB;
			m_normalWorldOnB = normal;
			m_distance1 = distance;

			//m_constraintRow[0].Reset();
			//m_constraintRow[1].Reset();
			//m_constraintRow[2].Reset();
		}

		 btManifoldPoint( ref btVector3 pointA, ref btVector3 pointB,
				ref btVector3 normal,
				double distance )
		{
			m_localPointA = pointA;
			m_localPointB = pointB;
			m_normalWorldOnB = normal;
			m_distance1 = distance;
			m_combinedFriction = 0;
			m_combinedRollingFriction = 0;
			m_combinedRestitution = 0;
			m_userPersistentData = 0;
			m_lateralFrictionInitialized = false;
			m_appliedImpulse = 0;
			m_appliedImpulseLateral1 = 0;
			m_appliedImpulseLateral2 = 0;
			m_contactMotion1 = 0;
			m_contactMotion2 = 0;
			m_contactCFM1 = 0;
			m_contactCFM2 = 0;
			m_lifeTime = 0;

		}



		internal btVector3 m_localPointA;
		internal btVector3 m_localPointB;
		internal btVector3 m_positionWorldOnB;
		///m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity
		internal btVector3 m_positionWorldOnA;
		internal btVector3 m_normalWorldOnB;

		internal double m_distance1;
		internal double m_combinedFriction;
		internal double m_combinedRollingFriction;
		internal double m_combinedRestitution;

		//BP mod, store contact triangles.
		internal int m_partId0;
		internal int m_partId1;
		internal int m_index0;
		internal int m_index1;

		internal object m_userPersistentData;
		internal bool m_lateralFrictionInitialized;

		internal double m_appliedImpulse;
		internal double m_appliedImpulseLateral1;
		internal double m_appliedImpulseLateral2;
		internal double m_contactMotion1;
		internal double m_contactMotion2;
		internal double m_contactCFM1;
		internal double m_contactCFM2;

		internal int m_lifeTime;//lifetime of the contactpoint in frames

		internal btVector3 m_lateralFrictionDir1;
		internal btVector3 m_lateralFrictionDir2;

		internal double getDistance()
		{
			return m_distance1;
		}
		internal int getLifeTime()
		{
			return m_lifeTime;
		}

		internal btVector3 getPositionWorldOnA()
		{
			return m_positionWorldOnA;
			//				return m_positionWorldOnB + m_normalWorldOnB  m_distance1;
		}

		internal btVector3 getPositionWorldOnB()
		{
			return m_positionWorldOnB;
		}

		internal void setDistance( double dist )
		{
			m_distance1 = dist;
		}

		///this returns the most recent applied impulse, to satisfy contact constraints by the constraint solver
		internal double getAppliedImpulse()
		{
			return m_appliedImpulse;
		}

	}
}
