/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


/*
  @mainpage Bullet Documentation
 
  @section intro_sec Introduction
  Bullet is a Collision Detection and Rigid Body Dynamics Library. The Library is Open Source and free for commercial use, under the ZLib license ( http://opensource.org/licenses/zlib-license.php ).
 
  The main documentation is Bullet_User_Manual.pdf, included in the source code distribution.
  There is the Physics Forum for feedback and general Collision Detection and Physics discussions.
  Please visit http://www.bulletphysics.org
 
  @section install_sec Installation
 
  @subsection step1 Step 1: Download
  You can download the Bullet Physics Library from the github repository: https://github.com/bulletphysics/bullet3/releases 
 
  @subsection step2 Step 2: Building
  Bullet has multiple build systems, including premake, cmake and autotools. Premake and cmake support all platforms.
  Premake is included in the Bullet/build folder for Windows, Mac OSX and Linux. 
  Under Windows you can click on Bullet/build/vs2010bat to create Microsoft Visual Studio projects. 
  On Mac OSX and Linux you can open a terminal and generate Makefile, codeblocks or Xcode4 projects:
cd Bullet/build
./premake4_osx gmake or ./premake4_linux gmake or ./premake4_linux64 gmake or (for Mac) ./premake4_osx xcode4
cd Bullet/build/gmake
make

An alternative to premake is cmake. You can download cmake from http://www.cmake.org
cmake can autogenerate projectfiles for Microsoft Visual Studio, Apple Xcode, KDevelop and Unix Makefiles.
The easiest is to run the CMake cmake-gui graphical user interface and choose the options and generate projectfiles.
You can also use cmake in the command-line. Here are some examples for various platforms:
cmake . -G "Visual Studio 9 2008"
cmake . -G Xcode
cmake . -G "Unix Makefiles"
Although cmake is recommended, you can also use autotools for UNIX: ./autogen.sh ./configure to create a Makefile and then run make.

@subsection step3 Step 3: Testing demos
Try to run and experiment with BasicDemo executable as a starting point.
Bullet can be used in several ways, as Full Rigid Body simulation, as Collision Detector Library or Low Level / Snippets like the GJK Closest Point calculation.
The Dependencies can be seen in this documentation under Directories

@subsection step4 Step 4: Integrating in your application, full Rigid Body and Soft Body simulation
Check out BasicDemo how to create a btDynamicsWorld, btRigidBody and btCollisionShape, Stepping the simulation and synchronizing your graphics object transform.
Check out SoftDemo how to use soft body dynamics, using btSoftRigidDynamicsWorld.
@subsection step5 Step 5 : Integrate the Collision Detection Library (without Dynamics and other Extras)
Bullet Collision Detection can also be used without the Dynamics/Extras.
  Check out btCollisionWorld and btCollisionObject, and the CollisionInterfaceDemo.
  @subsection step6 Step 6 : Use Snippets like the GJK Closest Point calculation.
  Bullet has been designed in a modular way keeping dependencies to a minimum. The ConvexHullDistance demo demonstrates direct use of btGjkPairDetector.
 
  @section copyright Copyright
  For up-to-data information and copyright and contributors list check out the Bullet_User_Manual.pdf
  
 */

using System.Diagnostics;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.NarrowPhase;
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.Collision.Dispatch
{
	//class btCollisionShape;
	//class btConvexShape;
	//class btBroadphaseInterface;
	//class btSerializer;


	///CollisionWorld is interface and container for the collision detection
	public partial class btCollisionWorld
	{

		internal btCollisionObjectArray m_collisionObjects;

		internal btDispatcher m_dispatcher1;

		internal btDispatcherInfo m_dispatchInfo;

		protected btBroadphaseInterface m_broadphasePairCache;

		protected btIDebugDraw m_debugDrawer;

		///m_forceUpdateAllAabbs can be set to false as an optimization to only update active object AABBs
		///it is true by default, because it is error-prone (setting the position of static objects wouldn't update their AABB)
		protected bool m_forceUpdateAllAabbs;

		//protected void serializeCollisionObjects(btSerializer serializer);

		protected btVector3 m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
		protected btVector3 m_rayToWorld;

		btList<btVector3> m_hitNormalWorld;
		btList<btVector3> m_hitPointWorld;
		btList<double> m_hitFractions;



		//this constructor doesn't own the dispatcher and paircache/broadphase
		//public btCollisionWorld(btDispatcher dispatcher,btBroadphaseInterface broadphasePairCache, btCollisionConfiguration collisionConfiguration);

		//virtual ~btCollisionWorld();

		public void setBroadphase( btBroadphaseInterface pairCache )
		{
			m_broadphasePairCache = pairCache;
		}

		public btBroadphaseInterface getBroadphase()
		{
			return m_broadphasePairCache;
		}

		public btOverlappingPairCache getPairCache()
		{
			return m_broadphasePairCache.getOverlappingPairCache();
		}


		public btDispatcher getDispatcher()
		{
			return m_dispatcher1;
		}

		//void updateSingleAabb( btCollisionObject colObj );


		public virtual void setDebugDrawer( btIDebugDraw debugDrawer )
		{
			m_debugDrawer = debugDrawer;
		}

		public virtual btIDebugDraw getDebugDrawer()
		{
			return m_debugDrawer;
		}


		///LocalShapeInfo gives extra information for complex shapes
		///Currently, only btTriangleMeshShape is available, so it just contains triangleIndex and subpart
		public class LocalShapeInfo
		{
			public int m_shapePart;
			public int m_triangleIndex;

			//btCollisionShape	m_shapeTemp;
			//btTransform	m_shapeLocalTransform;
		};

		public class LocalRayResult
		{
			public LocalRayResult( btCollisionObject collisionObject,
				LocalShapeInfo localShapeInfo,
				btVector3 hitNormalLocal,
				double hitFraction )
			{
				m_collisionObject = ( collisionObject );
				m_localShapeInfo = ( localShapeInfo );
				m_hitNormalLocal = ( hitNormalLocal );
				m_hitFraction = ( hitFraction );
			}

			public btCollisionObject m_collisionObject;
			public LocalShapeInfo m_localShapeInfo;
			public btVector3 m_hitNormalLocal;
			public double m_hitFraction;

		};

		///RayResultCallback is used to report new raycast results
		public abstract class RayResultCallback
		{
			public double m_closestHitFraction;
			public btCollisionObject m_collisionObject;
			public btBroadphaseProxy.CollisionFilterGroups m_collisionFilterGroup;
			public btBroadphaseProxy.CollisionFilterGroups m_collisionFilterMask;
			//@BP Mod - Custom flags, currently used to enable backface culling on tri-meshes, see btRaycastCallback.h. Apply any of the EFlags defined there on m_flags here to invoke.
			public uint m_flags;

			public bool hasHit()
			{
				return ( m_collisionObject != null );
			}

			public RayResultCallback( )
			{
				m_closestHitFraction = ( (double)( 1.0 ) );
				m_collisionObject = null;
				m_collisionFilterGroup = ( btBroadphaseProxy.CollisionFilterGroups.DefaultFilter );
				m_collisionFilterMask = ( btBroadphaseProxy.CollisionFilterGroups.AllFilter );
				//@BP Mod
				m_flags = ( 0 );
			}

			public virtual bool needsCollision( btBroadphaseProxy proxy0 )
			{
				bool collides = ( proxy0.m_collisionFilterGroup & m_collisionFilterMask ) != 0;
				collides = collides && ( m_collisionFilterGroup & proxy0.m_collisionFilterMask ) != 0;
				return collides;
			}


			public abstract double addSingleResult( LocalRayResult rayResult, bool normalInWorldSpace );
		};

		public class ClosestRayResultCallback : RayResultCallback
		{
			public ClosestRayResultCallback( ref btVector3 rayFromWorld, ref btVector3 rayToWorld )
			{
				m_rayFromWorld = ( rayFromWorld );
				m_rayToWorld = ( rayToWorld );
			}

			public btVector3 m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
			public btVector3 m_rayToWorld;

			public btVector3 m_hitNormalWorld;
			public btVector3 m_hitPointWorld;

			public override double addSingleResult( LocalRayResult rayResult, bool normalInWorldSpace )
			{
				//caller already does the filter on the m_closestHitFraction
				Debug.Assert( rayResult.m_hitFraction <= m_closestHitFraction );

				m_closestHitFraction = rayResult.m_hitFraction;
				m_collisionObject = rayResult.m_collisionObject;
				if( normalInWorldSpace )
				{
					m_hitNormalWorld = rayResult.m_hitNormalLocal;
				}
				else
				{
					///need to transform normal into worldspace
					m_collisionObject.m_worldTransform.m_basis.Apply( ref rayResult.m_hitNormalLocal, out m_hitNormalWorld );
					//m_hitNormalWorld = m_collisionObject.getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;
				}
				btVector3.setInterpolate3( out m_hitPointWorld, ref m_rayFromWorld, ref m_rayToWorld, rayResult.m_hitFraction );
				return rayResult.m_hitFraction;
			}
		};

		public class AllHitsRayResultCallback : RayResultCallback
		{
			public AllHitsRayResultCallback( ref btVector3 rayFromWorld, ref btVector3 rayToWorld )
			{
				m_rayFromWorld = ( rayFromWorld );
				m_rayToWorld = ( rayToWorld );
			}

			btList<btCollisionObject> m_collisionObjects;

			btVector3 m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
			btVector3 m_rayToWorld;

			btList<btVector3> m_hitNormalWorld;
			btList<btVector3> m_hitPointWorld;
			btList<double> m_hitFractions;

			public override double addSingleResult( LocalRayResult rayResult, bool normalInWorldSpace )
			{
				m_collisionObject = rayResult.m_collisionObject;
				m_collisionObjects.Add( rayResult.m_collisionObject );
				btVector3 hitNormalWorld;
				if( normalInWorldSpace )
				{
					hitNormalWorld = rayResult.m_hitNormalLocal;
				}
				else
				{
					///need to transform normal into worldspace
					m_collisionObject.m_worldTransform.m_basis.Apply( ref rayResult.m_hitNormalLocal, out hitNormalWorld );
					//hitNormalWorld = m_collisionObject.getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;
				}
				m_hitNormalWorld.Add( ref hitNormalWorld );
				btVector3 hitPointWorld;
				btVector3.setInterpolate3( out hitPointWorld, ref m_rayFromWorld, ref m_rayToWorld, rayResult.m_hitFraction );
				m_hitPointWorld.Add( hitPointWorld );
				m_hitFractions.Add( rayResult.m_hitFraction );
				return m_closestHitFraction;
			}
		};


		public struct LocalConvexResult
		{
			public LocalConvexResult( btCollisionObject hitCollisionObject,
				LocalShapeInfo localShapeInfo,
				ref btVector3 hitNormalLocal,
				ref btVector3 hitPointLocal,
				double hitFraction
				)
			{
				m_hitCollisionObject = ( hitCollisionObject );
				m_localShapeInfo = ( localShapeInfo );
				m_hitNormalLocal = ( hitNormalLocal );
				m_hitPointLocal = ( hitPointLocal );
				m_hitFraction = ( hitFraction );
			}

			public btCollisionObject m_hitCollisionObject;
			public LocalShapeInfo m_localShapeInfo;
			public btVector3 m_hitNormalLocal;
			public btVector3 m_hitPointLocal;
			public double m_hitFraction;
		};

		///RayResultCallback is used to report new raycast results
		public abstract class ConvexResultCallback
		{
			public double m_closestHitFraction;
			public btBroadphaseProxy.CollisionFilterGroups m_collisionFilterGroup;
			public btBroadphaseProxy.CollisionFilterGroups m_collisionFilterMask;

			public ConvexResultCallback()
			{
				m_closestHitFraction = ( (double)( 1.0 ) );
				m_collisionFilterGroup = ( btBroadphaseProxy.CollisionFilterGroups.DefaultFilter );
				m_collisionFilterMask = ( btBroadphaseProxy.CollisionFilterGroups.AllFilter );
			}

			public bool hasHit()
			{
				return ( m_closestHitFraction < (double)( 1.0 ) );
			}



			public virtual bool needsCollision( btBroadphaseProxy proxy0 )
			{
				bool collides = ( proxy0.m_collisionFilterGroup & m_collisionFilterMask ) != 0;
				collides = collides && ( m_collisionFilterGroup & proxy0.m_collisionFilterMask ) != 0;
				return collides;
			}

			public abstract double addSingleResult( ref LocalConvexResult convexResult, bool normalInWorldSpace );
		};

		public class ClosestConvexResultCallback : ConvexResultCallback
		{
			public ClosestConvexResultCallback( ref btVector3 convexFromWorld, ref btVector3 convexToWorld )
			{
				m_convexFromWorld = ( convexFromWorld );
				m_convexToWorld = ( convexToWorld );
				m_hitCollisionObject = null;
			}

			btVector3 m_convexFromWorld;//used to calculate hitPointWorld from hitFraction
			btVector3 m_convexToWorld;

			btVector3 m_hitNormalWorld;
			btVector3 m_hitPointWorld;
			btCollisionObject m_hitCollisionObject;

			public override double addSingleResult( ref LocalConvexResult convexResult, bool normalInWorldSpace )
			{
				//caller already does the filter on the m_closestHitFraction
				Debug.Assert( convexResult.m_hitFraction <= m_closestHitFraction );

				m_closestHitFraction = convexResult.m_hitFraction;
				m_hitCollisionObject = convexResult.m_hitCollisionObject;
				if( normalInWorldSpace )
				{
					m_hitNormalWorld = convexResult.m_hitNormalLocal;
				}
				else
				{
					///need to transform normal into worldspace
					m_hitCollisionObject.m_worldTransform.m_basis.Apply( ref convexResult.m_hitNormalLocal, out m_hitNormalWorld );
					//m_hitNormalWorld = 
				}
				m_hitPointWorld = convexResult.m_hitPointLocal;
				return convexResult.m_hitFraction;
			}
		};

		///ContactResultCallback is used to report contact points
		public abstract class ContactResultCallback
		{
			btBroadphaseProxy.CollisionFilterGroups m_collisionFilterGroup;
			btBroadphaseProxy.CollisionFilterGroups m_collisionFilterMask;

			public ContactResultCallback()
			{
				m_collisionFilterGroup = ( btBroadphaseProxy.CollisionFilterGroups.DefaultFilter );
				m_collisionFilterMask = ( btBroadphaseProxy.CollisionFilterGroups.AllFilter );
			}

			public virtual bool needsCollision( btBroadphaseProxy proxy0 )
			{
				bool collides = ( proxy0.m_collisionFilterGroup & m_collisionFilterMask ) != 0;
				collides = collides && ( m_collisionFilterGroup & proxy0.m_collisionFilterMask ) != 0;
				return collides;
			}

			internal abstract double addSingleResult( btManifoldPoint cp, btCollisionObjectWrapper colObj0Wrap, int partId0, int index0, btCollisionObjectWrapper colObj1Wrap, int partId1, int index1 ) ;
		};



		public int getNumCollisionObjects()
		{
			return m_collisionObjects.Count;
		}


		btCollisionObjectArray getCollisionObjectArray()
		{
			return m_collisionObjects;
		}

		public btDispatcherInfo getDispatchInfo()
		{
			return m_dispatchInfo;
		}

		public bool getForceUpdateAllAabbs()
		{
			return m_forceUpdateAllAabbs;
		}
		public void setForceUpdateAllAabbs( bool forceUpdateAllAabbs )
		{
			m_forceUpdateAllAabbs = forceUpdateAllAabbs;
		}

		///Preliminary serialization test for Bullet 2.76. Loading those files requires a separate parser (Bullet/Demos/SerializeDemo)
		//virtual void serialize( btSerializer serializer );

	};


}