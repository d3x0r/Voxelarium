//#define DBVT_BP_PROFILE
//#define DBVT_BP_SORTPAIRS
//#define DBVT_BP_PREVENTFALSEUPDATE		
//#define DBVT_BP_ACCURATESLEEPING		
//#define DBVT_BP_ENABLE_BENCHMARK		

/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

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
using Bullet.Types;
using System.Collections.Generic;
using System.Diagnostics;

namespace Bullet.Collision.BroadPhase
{
	///btDbvtBroadphase implementation by Nathanael Presson

	//
	// Compile time config
	//


	//
	// btDbvtProxy
	//
	internal class btDbvtProxy : btBroadphaseProxy
	{
		/* Fields		*/
		//btDbvtAabbMm	aabb;
		internal btDbvtNode leaf;
		internal btDbvtProxy[] links = new btDbvtProxy[2];
		internal int stage;
		/* ctor			*/
		internal btDbvtProxy( ref btVector3 aabbMin, ref btVector3 aabbMax, object userPtr
			  , btBroadphaseProxy.CollisionFilterGroups collisionFilterGroup, btBroadphaseProxy.CollisionFilterGroups collisionFilterMask )
				: base( ref aabbMin, ref aabbMax, userPtr, collisionFilterGroup, collisionFilterMask )
		{
			links[0] = links[1] = null;
		}
	};

	internal class btDbvtProxyArray : btList<btDbvtProxy> { }

	///The btDbvtBroadphase implements a broadphase using two dynamic AABB bounding volume hierarchies/trees (see btDbvt).
	///One tree is used for static/non-moving objects, and another tree is used for dynamic objects. Objects can move from one tree to the other.
	///This is a very fast broadphase, especially for very dynamic worlds where many objects are moving. Its insert/add and remove of objects is generally faster than the sweep and prune broadphases btAxisSweep3 and bt32BitAxisSweep3.
	internal class btDbvtBroadphase : btBroadphaseInterface
	{
#if DBVT_BP_PROFILE
	const int DBVT_BP_PROFILING_RAT =256;
#endif
		/* Config		*/
		const int DYNAMIC_SET = 0;  /* Dynamic set index	*/
		const int FIXED_SET = 1;  /* Fixed set index		*/
		const int STAGECOUNT = 2;/* Number of stages		*/

		/* Fields		*/
		btDbvt[] m_sets = new btDbvt[2];                    // Dbvt sets
		LinkedList<btDbvtProxy>[] m_stageRoots = new LinkedList<btDbvtProxy>[STAGECOUNT + 1];   // Stages list
		btOverlappingPairCache m_paircache;             // Pair cache
		double m_prediction;                // Velocity prediction
		int m_stageCurrent;             // Current stage
		int m_fupdates;                 // % of fixed updates per frame
		int m_dupdates;                 // % of dynamic updates per frame
		int m_cupdates;                 // % of cleanup updates per frame
		int m_newpairs;                 // Number of pairs created
		int m_fixedleft;                // Fixed optimization left
		uint m_updates_call;                // Number of updates call
		uint m_updates_done;                // Number of updates done
		double m_updates_ratio;         // m_updates_done/m_updates_call
		int m_pid;                      // Parse id
		int m_cid;                      // Cleanup index
		int m_gid;                      // Gen id
		bool m_releasepaircache;            // Release pair cache on delete
		internal protected bool m_deferedcollide;          // Defere dynamic/static collision to collide call
		bool m_needcleanup;              // Need to run cleanup?
#if DBVT_BP_PROFILE
	btClock					m_clock;
	struct	{
		ulong		m_total;
		ulong		m_ddcollide;
		ulong		m_fdcollide;
		ulong		m_cleanup;
		ulong		m_jobcount;
	}				m_profiling;
#endif
		/* Methods		*/

#if asdfasdf
		btDbvtBroadphase(btOverlappingPairCache* paircache=0);
	~btDbvtBroadphase();
	void							collide(btDispatcher dispatcher);
	void							optimize();
	
	/* btBroadphaseInterface Implementation	*/
	btBroadphaseProxy				createProxy(ref btVector3 aabbMin,ref btVector3 aabbMax,int shapeType,object userPtr,btBroadphaseProxy.CollisionFilterGroups collisionFilterGroup,btBroadphaseProxy.CollisionFilterGroups collisionFilterMask,btDispatcher dispatcher,object multiSapProxy);
	virtual void					destroyProxy(btBroadphaseProxy proxy,btDispatcher dispatcher);
	virtual void					setAabb(btBroadphaseProxy proxy,ref btVector3 aabbMin,ref btVector3 aabbMax,btDispatcher dispatcher);
	virtual void					rayTest(ref btVector3 rayFrom,ref btVector3 rayTo, btBroadphaseRayCallback& rayCallback, ref btVector3 aabbMin=btVector3(0,0,0), ref btVector3 aabbMax = btVector3(0,0,0));
	virtual void					aabbTest(ref btVector3 aabbMin, ref btVector3 aabbMax, btBroadphaseAabbCallback& callback);

	virtual void					getAabb(btBroadphaseProxy proxy,ref btVector3 aabbMin, ref btVector3 aabbMax );
	virtual	void					calculateOverlappingPairs(btDispatcher dispatcher);
	virtual	btOverlappingPairCache*	getOverlappingPairCache();
	virtual	btOverlappingPairCache*	getOverlappingPairCache();
	virtual	void					getBroadphaseAabb(ref btVector3 aabbMin,ref btVector3 aabbMax);

	///reset broadphase internal structures, to ensure determinism/reproducability
	virtual void resetPool(btDispatcher dispatcher);

	void	performDeferredRemoval(btDispatcher dispatcher);
#endif

		void setVelocityPrediction( double prediction )
		{
			m_prediction = prediction;
		}
		double getVelocityPrediction()
		{
			return m_prediction;
		}



#if DBVT_BP_PROFILE
	struct ProfileScope
	{
		__forceinline ProfileScope( btClock clock, ulong& value) :
    m_clock(&clock),m_value(&value),m_base( clock.getTimeMicroseconds())
	{
	}
	__forceinline ~ProfileScope()
		{
			( *m_value ) += m_clock.getTimeMicroseconds() - m_base;
		}
		btClock* m_clock;
		ulong* m_value;
		ulong m_base;
	};
	//#define //SPC(_value_)	ProfileScope	spc_scope(m_clock,_value_)
#else
		//#define //SPC(_value_)
#endif

		//
		// Colliders
		//

		/* Tree collider	*/
		internal class btDbvtTreeCollider : DefaultCollide, btDbvt.ICollide
		{
			internal btDbvtBroadphase pbp;
			internal btDbvtProxy proxy;
			internal btDbvtTreeCollider( btDbvtBroadphase p ) { pbp = p; }
			public override void Process( btDbvtNode na, btDbvtNode nb )
			{
				if( na != nb )
				{
					btDbvtProxy pa = (btDbvtProxy)na.data;
					btDbvtProxy pb = (btDbvtProxy)nb.data;
#if DBVT_BP_SORTPAIRS
			if(pa.m_uniqueId>pb.m_uniqueId) 
				btSwap(pa,pb);
#endif
					pbp.m_paircache.addOverlappingPair( pa, pb );
					++pbp.m_newpairs;
				}
			}
			public override void Process( btDbvtNode n )
			{
				Process( n, proxy.leaf );
			}
		};

		//
		// btDbvtBroadphase
		//

		//
		internal btDbvtBroadphase( btOverlappingPairCache paircache )
		{
			m_deferedcollide = false;
			m_needcleanup = true;
			m_releasepaircache = ( paircache != null ) ? false : true;
			m_prediction = 0;
			m_stageCurrent = 0;
			m_fixedleft = 0;
			m_fupdates = 1;
			m_dupdates = 0;
			m_cupdates = 10;
			m_newpairs = 1;
			m_updates_call = 0;
			m_updates_done = 0;
			m_updates_ratio = 0;
			m_paircache = paircache != null ? paircache : new btHashedOverlappingPairCache();
			m_gid = 0;
			m_pid = 0;
			m_cid = 0;
			for( int i = 0; i <= STAGECOUNT; ++i )
			{
				m_stageRoots[i] = null;
			}
#if DBVT_BP_PROFILE
	clear( m_profiling);
#endif
		}

		//
		~btDbvtBroadphase()
		{
			if( m_releasepaircache )
			{
				m_paircache = null;
				//m_paircache.~btOverlappingPairCache();
				//btAlignedFree( m_paircache );
			}
		}

		//
		public  btBroadphaseProxy createProxy( ref btVector3 aabbMin,
																	  ref btVector3 aabbMax,
																	  BroadphaseNativeTypes shapeType,
																	  object userPtr,
																	  btBroadphaseProxy.CollisionFilterGroups collisionFilterGroup,
																	  btBroadphaseProxy.CollisionFilterGroups collisionFilterMask,
																	  btDispatcher dispatcher,
																	  object multiSapProxy)
		{
			btDbvtProxy proxy = new btDbvtProxy( ref aabbMin, ref aabbMax, userPtr,
				  collisionFilterGroup,
				  collisionFilterMask );

			btDbvtVolume aabb = btDbvtVolume.FromMM( ref aabbMin, ref aabbMax );

			//bproxy.aabb			=	btDbvtVolume::FromMM(aabbMin,aabbMax);
			proxy.stage = m_stageCurrent;
			proxy.m_uniqueId = ++m_gid;
			proxy.leaf = m_sets[0].insert( ref aabb, proxy );
			m_stageRoots[m_stageCurrent].AddLast( proxy );
			//listappend( proxy, m_stageRoots[m_stageCurrent] );
			if( !m_deferedcollide )
			{
				btDbvtTreeCollider collider = new btDbvtTreeCollider( this );
				collider.proxy = proxy;
				btDbvt.CollideTV( m_sets[0].m_root, ref aabb, collider );
				btDbvt.CollideTV( m_sets[1].m_root, ref aabb, collider );
			}
			return ( proxy );
		}

		//
		public void destroyProxy( btBroadphaseProxy absproxy,
																	   btDispatcher dispatcher )
		{
			btDbvtProxy proxy = (btDbvtProxy)absproxy;
			if( proxy.stage == STAGECOUNT )
				m_sets[1].remove( proxy.leaf );
			else
				m_sets[0].remove( proxy.leaf );
			m_stageRoots[proxy.stage].Remove( proxy );
			//listremove( proxy, m_stageRoots[proxy.stage] );
			m_paircache.removeOverlappingPairsContainingProxy( proxy, dispatcher );
			//btAlignedFree( proxy );
			m_needcleanup = true;
		}

		public void getAabb( btBroadphaseProxy absproxy, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			btDbvtProxy proxy = (btDbvtProxy)absproxy;
			aabbMin = proxy.m_aabbMin;
			aabbMax = proxy.m_aabbMax;
		}

		internal class BroadphaseRayTester : DefaultCollide, btDbvt.ICollide
		{
			btBroadphaseRayCallback m_rayCallback;
			internal BroadphaseRayTester( btBroadphaseRayCallback orgCallback)
			{
				m_rayCallback = ( orgCallback );
            }
			public override void Process( btDbvtNode leaf )
			{
				btDbvtProxy proxy = (btDbvtProxy)leaf.data;
				m_rayCallback.process( proxy );
			}
		};

		public void rayTest( ref btVector3 rayFrom, ref btVector3 rayTo, btBroadphaseRayCallback rayCallback
			, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			BroadphaseRayTester callback = new BroadphaseRayTester( rayCallback );

			m_sets[0].rayTestInternal( m_sets[0].m_root,
				ref rayFrom,
				ref rayTo,
				ref rayCallback.m_rayDirectionInverse,
				rayCallback.m_signs,
				rayCallback.m_lambda_max,
				ref aabbMin,
				ref aabbMax,
				callback );

			m_sets[1].rayTestInternal( m_sets[1].m_root,
				ref rayFrom,
				ref rayTo,
				ref rayCallback.m_rayDirectionInverse,
				rayCallback.m_signs,
				rayCallback.m_lambda_max,
				ref aabbMin,
				ref aabbMax,
				callback );

		}


		internal class BroadphaseAabbTester : DefaultCollide, btDbvt.ICollide
		{
			btBroadphaseAabbCallback m_aabbCallback;
			internal BroadphaseAabbTester( btBroadphaseAabbCallback orgCallback)
			{
				m_aabbCallback = ( orgCallback );
            }
			public override void Process( btDbvtNode leaf )
			{
				btDbvtProxy proxy = (btDbvtProxy)leaf.data;
				m_aabbCallback.process( proxy );
			}
		};

		public void aabbTest( ref btVector3 aabbMin, ref btVector3 aabbMax, btBroadphaseAabbCallback aabbCallback)
		{
			BroadphaseAabbTester callback = new BroadphaseAabbTester( aabbCallback );

			btDbvtVolume  bounds = btDbvtVolume.FromMM( ref aabbMin, ref aabbMax );
			//process all children, that overlap with  the given AABB bounds
			btDbvt.CollideTV( m_sets[0].m_root, ref bounds, callback );
			btDbvt.CollideTV( m_sets[1].m_root, ref bounds, callback );

		}



		//
		public void setAabb( btBroadphaseProxy absproxy,
																  ref btVector3 aabbMin,
																  ref btVector3 aabbMax,
																  btDispatcher dispatcher )
		{
			btDbvtProxy proxy = (btDbvtProxy)absproxy;
			btDbvtVolume aabb = btDbvtVolume.FromMM( ref aabbMin, ref aabbMax );
#if DBVT_BP_PREVENTFALSEUPDATE
	if( NotEqual( aabb, proxy.leaf.volume ) )
#endif
			{
				bool docollide = false;
				if( proxy.stage == STAGECOUNT )
				{/* fixed . dynamic set	*/
					m_sets[1].remove( proxy.leaf );
					proxy.leaf = m_sets[0].insert( ref aabb, proxy );
					docollide = true;
				}
				else
				{/* dynamic set				*/
					++m_updates_call;
					if( btDbvtVolume.Intersect( ref proxy.leaf.volume,ref aabb ) )
					{/* Moving				*/

						btVector3 delta = aabbMin - proxy.m_aabbMin;
						btVector3 tmp;
						proxy.m_aabbMax.Sub( ref proxy.m_aabbMin, out tmp );
						btVector3 velocity;// =  ( ( ( proxy.m_aabbMax - proxy.m_aabbMin ) / 2 ) * m_prediction );
						tmp.Mult( m_prediction / 2, out velocity );

						if( delta[0] < 0 ) velocity[0] = -velocity[0];
						if( delta[1] < 0 ) velocity[1] = -velocity[1];
						if( delta[2] < 0 ) velocity[2] = -velocity[2];
						//#define DBVT_BP_MARGIN					(double)0.05
						// if not def margin, pass empty argument
						if( m_sets[0].update( proxy.leaf, ref aabb, ref velocity, 0.05/*DBVT_BP_MARGIN*/ ) )
						{
							++m_updates_done;
							docollide = true;
						}
					}
					else
					{/* Teleporting			*/
						m_sets[0].update( proxy.leaf, ref aabb );
						++m_updates_done;
						docollide = true;
					}
				}
				m_stageRoots[proxy.stage].Remove( proxy );
				//listremove( proxy, m_stageRoots[proxy.stage] );
				proxy.m_aabbMin = aabbMin;
				proxy.m_aabbMax = aabbMax;
				proxy.stage = m_stageCurrent;
				m_stageRoots[m_stageCurrent].AddLast( proxy );
				//listappend( proxy, m_stageRoots[m_stageCurrent] );
				if( docollide )
				{
					m_needcleanup = true;
					if( !m_deferedcollide )
					{
						btDbvtTreeCollider collider = new btDbvtTreeCollider( this );
						btDbvt.CollideTTpersistentStack( m_sets[1].m_root, proxy.leaf, collider );
						btDbvt.CollideTTpersistentStack( m_sets[0].m_root, proxy.leaf, collider );
					}
				}
			}
		}


		//
		///this setAabbForceUpdate is similar to setAabb but always forces the aabb update. 
		///it is not part of the btBroadphaseInterface but specific to btDbvtBroadphase.
		///it bypasses certain optimizations that prevent aabb updates (when the aabb shrinks), see
		///http://code.google.com/p/bullet/issues/detail?id=223
		public void setAabbForceUpdate( btBroadphaseProxy absproxy,
																  ref btVector3 aabbMin,
																  ref btVector3 aabbMax,
																  btDispatcher dispatcher)
		{
			btDbvtProxy proxy = (btDbvtProxy)absproxy;
			btDbvtVolume   aabb = btDbvtVolume.FromMM( ref aabbMin, ref aabbMax );
			bool docollide = false;
			if( proxy.stage == STAGECOUNT )
			{/* fixed . dynamic set	*/
				m_sets[1].remove( proxy.leaf );
				proxy.leaf = m_sets[0].insert( ref aabb, proxy );
				docollide = true;
			}
			else
			{/* dynamic set				*/
				++m_updates_call;
				/* Teleporting			*/
				m_sets[0].update( proxy.leaf, ref aabb );
				++m_updates_done;
				docollide = true;
			}
			m_stageRoots[m_stageCurrent].Remove( proxy );
			//listremove( proxy, m_stageRoots[proxy.stage] );
			proxy.m_aabbMin = aabbMin;
			proxy.m_aabbMax = aabbMax;
			proxy.stage = m_stageCurrent;
			m_stageRoots[m_stageCurrent].AddLast( proxy );
			//listappend( proxy, m_stageRoots[m_stageCurrent] );
			if( docollide )
			{
				m_needcleanup = true;
				if( !m_deferedcollide )
				{
					btDbvtTreeCollider collider = new btDbvtTreeCollider( this );
					btDbvt.CollideTTpersistentStack( m_sets[1].m_root, proxy.leaf, collider );
					btDbvt.CollideTTpersistentStack( m_sets[0].m_root, proxy.leaf, collider );
				}
			}
		}

		//
		public void calculateOverlappingPairs( btDispatcher dispatcher )
		{
			collide( dispatcher );
#if DBVT_BP_PROFILE
	if( 0 == ( m_pid % DBVT_BP_PROFILING_RATE ) )
	{
		Console.WriteLine( "fixed(%u) dynamics(%u) pairs(%u)\r\n", m_sets[1].m_leaves, m_sets[0].m_leaves, m_paircache.getNumOverlappingPairs() );
		uint total = m_profiling.m_total;
		if( total <= 0 ) total = 1;
		Console.WriteLine( "ddcollide: %u%% (%uus)\r\n", ( 50 + m_profiling.m_ddcollide * 100 ) / total, m_profiling.m_ddcollide / DBVT_BP_PROFILING_RATE );
		Console.WriteLine( "fdcollide: %u%% (%uus)\r\n", ( 50 + m_profiling.m_fdcollide * 100 ) / total, m_profiling.m_fdcollide / DBVT_BP_PROFILING_RATE );
		Console.WriteLine( "cleanup:   %u%% (%uus)\r\n", ( 50 + m_profiling.m_cleanup * 100 ) / total, m_profiling.m_cleanup / DBVT_BP_PROFILING_RATE );
		Console.WriteLine( "total:     %uus\r\n", total / DBVT_BP_PROFILING_RATE );
		ulong sum = m_profiling.m_ddcollide +
			m_profiling.m_fdcollide +
			m_profiling.m_cleanup;
		Console.WriteLine( "leaked: %u%% (%uus)\r\n", 100 - ( ( 50 + sum * 100 ) / total ), ( total - sum ) / DBVT_BP_PROFILING_RATE );
		Console.WriteLine( "job counts: %u%%\r\n", ( m_profiling.m_jobcount * 100 ) / ( ( m_sets[0].m_leaves + m_sets[1].m_leaves ) * DBVT_BP_PROFILING_RATE ) );
		clear( m_profiling );
		m_clock.reset();
	}
#endif

			performDeferredRemoval( dispatcher );

		}

		void performDeferredRemoval( btDispatcher dispatcher )
		{

			if( m_paircache.hasDeferredRemoval() )
			{

				btBroadphasePairArray overlappingPairArray = m_paircache.getOverlappingPairArray();

				//perform a sort, to find duplicates and to sort 'invalid' pairs to the end
				overlappingPairArray.quickSort( btBroadphasePair.qsCompare );

				int invalidPair = 0;


				int i;

				btBroadphasePair previousPair = new btBroadphasePair();
				previousPair.m_pProxy0 = null;
				previousPair.m_pProxy1 = null;
				previousPair.m_algorithm = null;


				for( i = 0; i < overlappingPairArray.Count; i++ )
				{

					btBroadphasePair pair = overlappingPairArray[i];

					bool isDuplicate = ( pair.Equals( previousPair ) );

					previousPair = pair;

					bool needsRemoval = false;

					if( !isDuplicate )
					{
						//important to perform AABB check that is consistent with the broadphase
						btDbvtProxy pa = (btDbvtProxy)pair.m_pProxy0;
						btDbvtProxy pb = (btDbvtProxy)pair.m_pProxy1;
						bool hasOverlap = btDbvtVolume.Intersect( ref pa.leaf.volume, ref pb.leaf.volume );

						if( hasOverlap )
						{
							needsRemoval = false;
						}
						else
						{
							needsRemoval = true;
						}
					}
					else
					{
						//remove duplicate
						needsRemoval = true;
						//should have no algorithm
						Debug.Assert( pair.m_algorithm == null );
					}

					if( needsRemoval )
					{
						m_paircache.cleanOverlappingPair( ref pair, dispatcher );
						pair.m_pProxy0 = null;
						pair.m_pProxy1 = null;
						invalidPair++;
					}
				}

				//perform a sort, to sort 'invalid' pairs to the end
				overlappingPairArray.quickSort( btBroadphasePair.qsCompare );
				overlappingPairArray.Count = ( overlappingPairArray.Count - invalidPair );
			}
		}

		//
		void collide( btDispatcher dispatcher )
		{
			/*Console.WriteLine("---------------------------------------------------------\n");
			Console.WriteLine("m_sets[0].m_leaves=%d\n",m_sets[0].m_leaves);
			Console.WriteLine("m_sets[1].m_leaves=%d\n",m_sets[1].m_leaves);
			Console.WriteLine("numPairs = %d\n",getOverlappingPairCache().getNumOverlappingPairs());
			{
				int i;
				for (i=0;i<getOverlappingPairCache().getNumOverlappingPairs();i++)
				{
					Console.WriteLine("pair[%d]=(%d,%d),",i,getOverlappingPairCache().getOverlappingPairArray()[i].m_pProxy0.getUid(),
						getOverlappingPairCache().getOverlappingPairArray()[i].m_pProxy1.getUid());
				}
				Console.WriteLine("\n");
			}
		*/



			////SPC( m_profiling.m_total );
			/* optimize				*/
			m_sets[0].OptimizeIncremental( 1 + ( m_sets[0].m_leaves * m_dupdates ) / 100 );
			if( m_fixedleft != 0 )
			{
				int count = 1 + ( m_sets[1].m_leaves * m_fupdates ) / 100;
				m_sets[1].OptimizeIncremental( 1 + ( m_sets[1].m_leaves * m_fupdates ) / 100 );
				m_fixedleft = btScalar.btMax( 0, m_fixedleft - count );
			}
			/* dynamic . fixed set	*/
			m_stageCurrent = ( m_stageCurrent + 1 ) % STAGECOUNT;
			btDbvtProxy current = m_stageRoots[m_stageCurrent].First.Value;
			if( current != null )
			{
				btDbvtTreeCollider collider = new btDbvtTreeCollider( this );
				do
				{
					btDbvtProxy next = current.links[1];
					m_stageRoots[current.stage].Remove( current );
					//listremove( current, m_stageRoots[current.stage] );
					m_stageRoots[m_stageCurrent].AddLast( current );
					//listappend( current, m_stageRoots[STAGECOUNT] );
#if DBVT_BP_ACCURATESLEEPING
			m_paircache.removeOverlappingPairsContainingProxy( current, dispatcher );
			collider.proxy = current;
			btDbvt::collideTV( m_sets[0].m_root, current.aabb, collider );
			btDbvt::collideTV( m_sets[1].m_root, current.aabb, collider );
#endif
					m_sets[0].remove( current.leaf );
					btDbvtVolume   curAabb = btDbvtVolume.FromMM( ref current.m_aabbMin, ref current.m_aabbMax );
					current.leaf = m_sets[1].insert( ref curAabb, current );
					current.stage = STAGECOUNT;
					current = next;
				} while( current != null );
				m_fixedleft = m_sets[1].m_leaves;
				m_needcleanup = true;
			}
			/* collide dynamics		*/
			{
				btDbvtTreeCollider collider = new btDbvtTreeCollider( this );
				if( m_deferedcollide )
				{
					//SPC( m_profiling.m_fdcollide );
					btDbvt.CollideTTpersistentStack( m_sets[0].m_root, m_sets[1].m_root, collider );
				}
				if( m_deferedcollide )
				{
					//SPC( m_profiling.m_ddcollide );
					btDbvt.CollideTTpersistentStack( m_sets[0].m_root, m_sets[0].m_root, collider );
				}
			}
			/* clean up				*/
			if( m_needcleanup )
			{
				//SPC( m_profiling.m_cleanup );
				btBroadphasePairArray  pairs = m_paircache.getOverlappingPairArray();
				if( pairs.Count > 0 )
				{

					int ni = btScalar.btMin( pairs.Count, btScalar.btMax( m_newpairs, ( pairs.Count * m_cupdates ) / 100 ) );
					for( int i = 0; i < ni; ++i )
					{
						btBroadphasePair p = pairs[( m_cid + i ) % pairs.Count];
						btDbvtProxy pa = (btDbvtProxy)p.m_pProxy0;
						btDbvtProxy pb = (btDbvtProxy)p.m_pProxy1;
						if( !btDbvtVolume.Intersect( ref pa.leaf.volume, ref pb.leaf.volume ) )
						{
#if DBVT_BP_SORTPAIRS
					if(pa.m_uniqueId>pb.m_uniqueId) 
						btSwap(pa,pb);
#endif
							m_paircache.removeOverlappingPair( pa, pb, dispatcher );
							--ni; --i;
						}
					}
					if( pairs.Count > 0 ) m_cid = ( m_cid + ni ) % pairs.Count; else m_cid = 0;
				}
			}
			++m_pid;
			m_newpairs = 1;
			m_needcleanup = false;
			if( m_updates_call > 0 )
			{ m_updates_ratio = m_updates_done / (double)m_updates_call; }
			else
			{ m_updates_ratio = 0; }
			m_updates_done /= 2;
			m_updates_call /= 2;
		}

		//
		void optimize()
		{
			m_sets[0].OptimizeTopDown();
			m_sets[1].OptimizeTopDown();
		}

		//
		public btOverlappingPairCache getOverlappingPairCache()
		{
			return ( m_paircache );
		}

		//
		public void getBroadphaseAabb( out btVector3 aabbMin, out btVector3 aabbMax )
		{

			btDbvtVolume   bounds;

			if( !m_sets[0].Empty() )
				if( !m_sets[1].Empty() )
					btDbvtVolume.Merge( ref m_sets[0].m_root.volume,
						ref m_sets[1].m_root.volume, out bounds );
				else
					bounds = m_sets[0].m_root.volume;
			else if( !m_sets[1].Empty() ) bounds = m_sets[1].m_root.volume;
			else
				bounds = btDbvtVolume.FromCR( ref btVector3.Zero, 0 );
			aabbMin = bounds.Mins();
			aabbMax = bounds.Maxs();
		}

		public void resetPool( btDispatcher dispatcher )
		{

			int totalObjects = m_sets[0].m_leaves + m_sets[1].m_leaves;
			if( totalObjects == 0 )
			{
				//reset internal dynamic tree data structures
				m_sets[0].Clear();
				m_sets[1].Clear();

				m_deferedcollide = false;
				m_needcleanup = true;
				m_stageCurrent = 0;
				m_fixedleft = 0;
				m_fupdates = 1;
				m_dupdates = 0;
				m_cupdates = 10;
				m_newpairs = 1;
				m_updates_call = 0;
				m_updates_done = 0;
				m_updates_ratio = 0;

				m_gid = 0;
				m_pid = 0;
				m_cid = 0;
				for( int i = 0; i <= STAGECOUNT; ++i )
				{
					m_stageRoots[i] = null;
				}
			}
		}

		//
		public void printStats()
		{ }

		//
#if DBVT_BP_ENABLE_BENCHMARK

struct btBroadphaseBenchmark
{
	struct Experiment
	{
		string name;
		int object_count;
		int update_count;
		int spawn_count;
		int iterations;
		double speed;
		double amplitude;
	};
	struct Object
	{
		btVector3 center;
		btVector3 extents;
		btBroadphaseProxy proxy;
		double time;
		void update( double speed, double amplitude, btBroadphaseInterface* pbi )
		{
			time += speed;
			center[0] = btCos( time * (double)2.17 ) * amplitude +
				btSin( time ) * amplitude / 2;
			center[1] = btCos( time * (double)1.38 ) * amplitude +
				btSin( time ) * amplitude;
			center[2] = btSin( time * (double)0.777 ) * amplitude;
			pbi.setAabb( proxy, center - extents, center + extents, 0 );
		}
	};
	static int UnsignedRand( int range = RAND_MAX - 1 ) { return ( rand() % ( range + 1 ) ); }
	static double UnitRand() { return ( UnsignedRand( 16384 ) / (double)16384 ); }
	static void OutputTime( string name, btClock& c, unsigned count = 0 )
	{
		ulong us = c.getTimeMicroseconds();
		ulong ms = ( us + 500 ) / 1000;
		double sec = us / (double)( 1000 * 1000 );
		if( count > 0 )
			Console.WriteLine( "%s : %u us (%u ms), %.2f/s\r\n", name, us, ms, count / sec );
		else
			Console.WriteLine( "%s : %u us (%u ms)\r\n", name, us, ms );
	}
};

static void benchmark( btBroadphaseInterface* pbi )
{
	static btBroadphaseBenchmark::Experiment experiments[] =
	{
		{"1024o.10%",1024,10,0,8192,(double)0.005,(double)100},
		/*{"4096o.10%",4096,10,0,8192,(double)0.005,(double)100},
		{"8192o.10%",8192,10,0,8192,(double)0.005,(double)100},*/
	};
	static int nexperiments = sizeof( experiments ) / sizeof( experiments[0] );
	List<btBroadphaseBenchmark::Object*> objects;
	btClock wallclock;
	/* Begin			*/
	for( int iexp = 0; iexp < nexperiments; ++iexp )
	{
		btBroadphaseBenchmark::Experiment & experiment = experiments[iexp];
		int object_count = experiment.object_count;
		int update_count = ( object_count * experiment.update_count ) / 100;
		int spawn_count = ( object_count * experiment.spawn_count ) / 100;
		double speed = experiment.speed;
		double amplitude = experiment.amplitude;
		Console.WriteLine( "Experiment #%u '%s':\r\n", iexp, experiment.name );
		Console.WriteLine( "\tObjects: %u\r\n", object_count );
		Console.WriteLine( "\tUpdate: %u\r\n", update_count );
		Console.WriteLine( "\tSpawn: %u\r\n", spawn_count );
		Console.WriteLine( "\tSpeed: %f\r\n", speed );
		Console.WriteLine( "\tAmplitude: %f\r\n", amplitude );
		srand( 180673 );
		/* Create objects	*/
		wallclock.reset();
		objects.reserve( object_count );
		for( int i = 0; i < object_count; ++i )
		{
			btBroadphaseBenchmark::Object* po = new btBroadphaseBenchmark::Object();
			po.center[0] = btBroadphaseBenchmark::UnitRand() * 50;
			po.center[1] = btBroadphaseBenchmark::UnitRand() * 50;
			po.center[2] = btBroadphaseBenchmark::UnitRand() * 50;
			po.extents[0] = btBroadphaseBenchmark::UnitRand() * 2 + 2;
			po.extents[1] = btBroadphaseBenchmark::UnitRand() * 2 + 2;
			po.extents[2] = btBroadphaseBenchmark::UnitRand() * 2 + 2;
			po.time = btBroadphaseBenchmark::UnitRand() * 2000;
			po.proxy = pbi.createProxy( po.center - po.extents, po.center + po.extents, 0, po, 1, 1, 0, 0 );
			objects.Add( po );
		}
		btBroadphaseBenchmark::OutputTime( "\tInitialization", wallclock );
		/* First update		*/
		wallclock.reset();
		for( int i = 0; i < objects.Count; ++i )
		{
			objects[i].update( speed, amplitude, pbi );
		}
		btBroadphaseBenchmark::OutputTime( "\tFirst update", wallclock );
		/* Updates			*/
		wallclock.reset();
		for( int i = 0; i < experiment.iterations; ++i )
		{
			for( int j = 0; j < update_count; ++j )
			{
				objects[j].update( speed, amplitude, pbi );
			}
			pbi.calculateOverlappingPairs( 0 );
		}
		btBroadphaseBenchmark::OutputTime( "\tUpdate", wallclock, experiment.iterations );
		/* Clean up			*/
		wallclock.reset();
		for( int i = 0; i < objects.Count; ++i )
		{
			pbi.destroyProxy( objects[i].proxy, 0 );
			delete objects[i];
		}
		objects.resize( 0 );
		btBroadphaseBenchmark::OutputTime( "\tRelease", wallclock );
	}

}
#else
		void benchmark( btBroadphaseInterface unused )
		{ }
#endif

#if DBVT_BP_PROFILE
//#undef //SPC
#endif


	};

}