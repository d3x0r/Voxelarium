#define CLEAN_INVALID_PAIRS 
#define USE_OVERLAP_TEST_ON_REMOVES

//Bullet Continuous Collision Detection and Physics Library
//Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

//
// btAxisSweep3.h
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.


using Bullet.LinearMath;
using Bullet.Types;
using System.Diagnostics;

// ushort might be uint (can global find replace safely)
// 

namespace Bullet.Collision.BroadPhase
{
	//#define DEBUG_BROADPHASE 1

	/// The internal templace class btAxisSweep3Internal implements the sweep and prune broadphase.
	/// It uses quantized integers to represent the begin and end points for each of the 3 axis.
	/// Dont use this class directly, use btAxisSweep3 or bt32BitAxisSweep3 instead.
	internal class btAxisSweep3 : btBroadphaseInterface
	{
		protected ushort m_bpHandleMask;
		protected ushort m_handleSentinel;

		internal class Edge
		{
			internal ushort m_pos;           // low bit is min/max
			internal ushort m_handle;

			internal ushort IsMax() { return ( (ushort)(m_pos & 1) ); }
		};

		internal class Handle : btBroadphaseProxy
		{
			internal Handle( ref btVector3 aabbMin, ref btVector3 aabbMax
				, object pOwner, CollisionFilterGroups collisionFilterGroup
				, CollisionFilterGroups collisionFilterMask
				, btDispatcher dispatcher, object multiSapProxy )
				: base( ref aabbMin, ref aabbMax, pOwner, collisionFilterGroup
					  , collisionFilterMask )
			{
				//, dispatcher, multiSapProxy
			}
			// indexes into the edge arrays
			internal ushort[] m_minEdges = new ushort[3]
								, m_maxEdges = new ushort[3];       // 6 * 2 = 12
																			//		ushort m_uniqueId;
			internal btBroadphaseProxy m_dbvtProxy;//for faster raycast
												   //object m_pOwner; this is now in btBroadphaseProxy.m_clientObject

			internal void SetNextFree( ushort next ) { m_minEdges[0] = next; }
			internal ushort GetNextFree() { return m_minEdges[0]; }
		};      // 24 bytes + 24 for Edge structures = 44 bytes total per entry


		protected btVector3 m_worldAabbMin;                       // overall system bounds
		protected btVector3 m_worldAabbMax;                     // overall system bounds

		protected btVector3 m_quantize;                     // scaling factor for quantization

		protected ushort m_numHandles;                      // number of active handles
		protected ushort m_maxHandles;                      // max number of handles
		protected btList<Handle> m_pHandles;                       // handles pool

		protected ushort m_firstFreeHandle;     // free handles list

		protected Edge[][] m_pEdges = new Edge[3][];                        // edge arrays for the 3 axes (each array has m_maxHandles * 2 + 2 sentinel entries)
		//Edge[][] m_pEdgesRawPtr = new Edge[3][];

		protected btOverlappingPairCache m_pairCache;

		///btOverlappingPairCallback is an additional optional user callback for adding/removing overlapping pairs, similar interface to btOverlappingPairCache.
		protected btOverlappingPairCallback m_userPairCallback;

		bool m_ownsPairCache;

		int m_invalidPair;

		///additional dynamic aabb structure, used to accelerate ray cast queries.
		///can be disabled using a optional argument in the constructor
		btDbvtBroadphase m_raycastAccelerator;
		btOverlappingPairCache m_nullPairCache;


		// allocation/deallocation
#if asdfasdf
		bool testOverlap2D( Handle pHandleA, Handle pHandleB, int axis0, int axis1 );
		void sortMinDown( int axis, ushort edge, btDispatcher dispatcher, bool updateOverlaps );
		void sortMinUp( int axis, ushort edge, btDispatcher dispatcher, bool updateOverlaps );
		void sortMaxDown( int axis, ushort edge, btDispatcher dispatcher, bool updateOverlaps );
		void sortMaxUp( int axis, ushort edge, btDispatcher dispatcher, bool updateOverlaps );
#endif

#if DEBUG_BROADPHASE
	void debugPrintAxis(int axis,bool checkCardinality=true);
#endif //DEBUG_BROADPHASE

		//Overlap* AddOverlap(ushort handleA, ushort handleB);
		//void RemoveOverlap(ushort handleA, ushort handleB);

		internal ushort getNumHandles()
		{
			return m_numHandles;
		}

		public Handle getHandle( ushort index ) { return m_pHandles[index]; }
#if asdfasdf
		virtual void calculateOverlappingPairs( btDispatcher dispatcher );

		ushort addHandle( ref btVector3 aabbMin, ref btVector3 aabbMax, object pOwner, short int collisionFilterGroup, short int collisionFilterMask, btDispatcher dispatcher, object multiSapProxy );
		void removeHandle( ushort handle, btDispatcher dispatcher );
		void updateHandle( ushort handle, ref btVector3 aabbMin, ref btVector3 aabbMax, btDispatcher dispatcher );

		virtual void resetPool( btDispatcher dispatcher );

		void processAllOverlappingPairs( btOverlapCallback* callback );

		//Broadphase Interface
		virtual btBroadphaseProxy createProxy( ref btVector3 aabbMin, ref btVector3 aabbMax, int shapeType, object userPtr, short int collisionFilterGroup, short int collisionFilterMask, btDispatcher dispatcher, object multiSapProxy );
		virtual void destroyProxy( btBroadphaseProxy proxy, btDispatcher dispatcher );
		virtual void setAabb( btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax, btDispatcher dispatcher );
		virtual void getAabb( btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax );

		virtual void rayTest( ref btVector3 rayFrom, ref btVector3 rayTo, btBroadphaseRayCallback rayCallback, ref btVector3 aabbMin = btVector3( 0, 0, 0 ), ref btVector3 aabbMax = btVector3( 0, 0, 0 ) );
		virtual void aabbTest( ref btVector3 aabbMin, ref btVector3 aabbMax, btBroadphaseAabbCallback callback);


		void quantize( ushort[] outdata, ref btVector3 point, int isMax );
		///unQuantize should be conservative: aabbMin/aabbMax should be larger then 'getAabb' result
		void unQuantize( btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax );

		bool testAabbOverlap( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );
#endif

		public btOverlappingPairCache getOverlappingPairCache()
		{
			return m_pairCache;
		}

		internal void setOverlappingPairUserCallback( btOverlappingPairCallback pairCallback )
		{
			m_userPairCallback = pairCallback;
		}
		internal btOverlappingPairCallback getOverlappingPairUserCallback()
		{
			return m_userPairCallback;
		}

		///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
		///will add some transform later
		public virtual void getBroadphaseAabb( out btVector3 aabbMin, out btVector3 aabbMax )
		{
			aabbMin = m_worldAabbMin;
			aabbMax = m_worldAabbMax;
		}

		public void printStats()
		{
			/*		Console.WriteLine("btAxisSweep3.h\n");
					Console.WriteLine("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
					Console.WriteLine("aabbMin=%f,%f,%f,aabbMax=%f,%f,%f\n",m_worldAabbMin.x,m_worldAabbMin.y,m_worldAabbMin.z,
						m_worldAabbMax.x,m_worldAabbMax.y,m_worldAabbMax.z);
						*/

		}


		////////////////////////////////////////////////////////////////////




#if DEBUG_BROADPHASE
			void btAxisSweep3<ushort>::debugPrintAxis(int axis, bool checkCardinality)
			{
				int numEdges = m_pHandles.m_maxEdges[axis];
				Console.WriteLine("SAP Axis %d, numEdges=%d\n",axis,numEdges);

				int i;
				for (i=0;i<numEdges+1;i++)
				{
					Edge pEdge = m_pEdges[axis] + i;
					Handle pHandlePrev = getHandle(pEdge.m_handle);
					int handleIndex = pEdge.IsMax()? pHandlePrev.m_maxEdges[axis] : pHandlePrev.m_minEdges[axis];
					char beginOrEnd;
					beginOrEnd=pEdge.IsMax()?'E':'B';
					Console.WriteLine("	[%c,h=%d,p=%x,i=%d]\n",beginOrEnd,pEdge.m_handle,pEdge.m_pos,handleIndex);
				}

				if (checkCardinality)
					Debug.Assert(numEdges == m_numHandles*2+1);
			}
#endif //DEBUG_BROADPHASE


		public btBroadphaseProxy createProxy( ref btVector3 aabbMin, ref btVector3 aabbMax, BroadphaseNativeTypes shapeType, object userPtr
			, btBroadphaseProxy.CollisionFilterGroups collisionFilterGroup
			, btBroadphaseProxy.CollisionFilterGroups collisionFilterMask
			, btDispatcher dispatcher, object multiSapProxy )
		{
			//(void)shapeType;
			ushort handleId = addHandle( ref aabbMin, ref aabbMax, userPtr, collisionFilterGroup, collisionFilterMask, dispatcher, multiSapProxy );

			Handle handle = getHandle( handleId );

			if( m_raycastAccelerator != null )
			{
				btBroadphaseProxy rayProxy = m_raycastAccelerator.createProxy( ref aabbMin, ref aabbMax
					, shapeType, userPtr
					, collisionFilterGroup, collisionFilterMask, dispatcher, 0 );
				handle.m_dbvtProxy = rayProxy;
			}
			return handle;
		}




		public void destroyProxy( btBroadphaseProxy proxy, btDispatcher dispatcher )
		{
			Handle handle = (Handle)( proxy );
			if( m_raycastAccelerator != null )
				m_raycastAccelerator.destroyProxy( handle.m_dbvtProxy, dispatcher );
			removeHandle( (ushort)( handle.m_uniqueId ), dispatcher );
		}


		public void setAabb( btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax, btDispatcher dispatcher )
		{
			Handle handle = (Handle)( proxy );
			handle.m_aabbMin = aabbMin;
			handle.m_aabbMax = aabbMax;
			updateHandle( (ushort)( handle.m_uniqueId ), ref aabbMin, ref aabbMax, dispatcher );
			if( m_raycastAccelerator != null )
				m_raycastAccelerator.setAabb( handle.m_dbvtProxy, ref aabbMin, ref aabbMax, dispatcher );

		}

		public void rayTest( ref btVector3 rayFrom, ref btVector3 rayTo, btBroadphaseRayCallback rayCallback )
		{
			Debug.Assert( false, "Are there default parameters?" );
		}

		public void rayTest( ref btVector3 rayFrom, ref btVector3 rayTo, btBroadphaseRayCallback rayCallback, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			if( m_raycastAccelerator != null )
			{
				m_raycastAccelerator.rayTest( ref rayFrom, ref rayTo, rayCallback, ref aabbMin, ref aabbMax );
			}
			else
			{
				//choose axis?
				ushort axis = 0;
				//for each proxy
				for( ushort i = 1; i < m_numHandles * 2 + 1; i++ )
				{
					if( m_pEdges[axis][i].IsMax() != 0 )
					{
						rayCallback.process( getHandle( m_pEdges[axis][i].m_handle ) );
					}
				}
			}
		}


		public void aabbTest( ref btVector3 aabbMin, ref btVector3 aabbMax, btBroadphaseAabbCallback callback)
		{
			if( m_raycastAccelerator != null )
			{
				m_raycastAccelerator.aabbTest( ref aabbMin, ref aabbMax, callback );
			}
			else
			{
				//choose axis?
				ushort axis = 0;
				//for each proxy
				for( ushort i = 1; i < m_numHandles * 2 + 1; i++ )
				{
					if( m_pEdges[axis][i].IsMax() != 0 )
					{
						Handle handle = getHandle( m_pEdges[axis][i].m_handle );
						if( btAabbUtil.TestAabbAgainstAabb2( ref aabbMin, ref aabbMax, ref handle.m_aabbMin, ref handle.m_aabbMax ) )
						{
							callback.process( handle );
						}
					}
				}
			}
		}




		public void getAabb( btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			Handle pHandle = (Handle)( proxy );
			aabbMin = pHandle.m_aabbMin;
			aabbMax = pHandle.m_aabbMax;
		}



		void unQuantize( btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			Handle pHandle = (Handle)( proxy );

			ushort[] vecInMin = new ushort[3];
			ushort[] vecInMax = new ushort[3];

			vecInMin[0] = m_pEdges[0][pHandle.m_minEdges[0]].m_pos;
			vecInMax[0] = (ushort)(m_pEdges[0][pHandle.m_maxEdges[0]].m_pos + 1);
			vecInMin[1] = m_pEdges[1][pHandle.m_minEdges[1]].m_pos;
			vecInMax[1] = (ushort)( m_pEdges[1][pHandle.m_maxEdges[1]].m_pos + 1);
			vecInMin[2] = m_pEdges[2][pHandle.m_minEdges[2]].m_pos;
			vecInMax[2] = (ushort)( m_pEdges[2][pHandle.m_maxEdges[2]].m_pos + 1);

			aabbMin.setValue( (double)( vecInMin[0] ) / ( m_quantize.x ), (double)( vecInMin[1] ) / ( m_quantize.y ), (double)( vecInMin[2] ) / ( m_quantize.z ) );
			aabbMin += m_worldAabbMin;

			aabbMax.setValue( (double)( vecInMax[0] ) / ( m_quantize.x ), (double)( vecInMax[1] ) / ( m_quantize.y ), (double)( vecInMax[2] ) / ( m_quantize.z ) );
			aabbMax += m_worldAabbMin;
		}




		internal btAxisSweep3( ref btVector3 worldAabbMin, ref btVector3 worldAabbMax
			, ushort handleMask, ushort handleSentinel
			, ushort userMaxHandles = 16384
			, btOverlappingPairCache pairCache = null, bool disableRaycastAccelerator = false )
		{
			m_bpHandleMask = ( handleMask );
			m_handleSentinel = ( handleSentinel );
			m_pairCache = ( pairCache );
			m_userPairCallback = ( null );
			m_ownsPairCache = ( false );
			m_invalidPair = ( 0 );
			m_raycastAccelerator = ( null );
			ushort maxHandles = (ushort)( userMaxHandles + 1 );//need to add one sentinel handle

			if( m_pairCache == null )
			{
				m_pairCache =  new  btHashedOverlappingPairCache();
				m_ownsPairCache = true;
			}

			if( !disableRaycastAccelerator )
			{
				m_nullPairCache = new btNullPairCache();
				m_raycastAccelerator = new  btDbvtBroadphase( m_nullPairCache );//m_pairCache);
				m_raycastAccelerator.m_deferedcollide = true;//don't add/remove pairs
			}

			//Debug.Assert(bounds.HasVolume());

			// init bounds
			m_worldAabbMin = worldAabbMin;
			m_worldAabbMax = worldAabbMax;

			btVector3 aabbSize = m_worldAabbMax - m_worldAabbMin;

			ushort maxInt = m_handleSentinel;

			m_quantize = new btVector3( (double)( maxInt/aabbSize.x ), (double)( maxInt / aabbSize.y ), (double)( maxInt / aabbSize.z ) ) ;

			// allocate handles buffer, using btAlignedAlloc, and put all handles on free list
			m_pHandles = new btList<Handle>( maxHandles );

			m_maxHandles = maxHandles;
			m_numHandles = 0;

			// handle 0 is reserved as the null index, and is also used as the sentinel
			m_firstFreeHandle = 1;
			{
				for( ushort i = m_firstFreeHandle; i < maxHandles; i++ )
					m_pHandles[i].SetNextFree( (ushort)( i + 1 ) );
				m_pHandles[maxHandles - 1].SetNextFree( 0 );
			}

			{
				// allocate edge buffers
				for( int i = 0; i < 3; i++ )
				{
					//m_pEdgesRawPtr[i] = new Edge[ maxHandles * 2];
					m_pEdges[i] = new Edge[maxHandles * 2];
				}
			}
			//removed overlap management

			// make boundary sentinels

			m_pHandles[0].m_clientObject = 0;

			for( int axis = 0; axis < 3; axis++ )
			{
				m_pHandles[0].m_minEdges[axis] = 0;
				m_pHandles[0].m_maxEdges[axis] = 1;

				m_pEdges[axis][0].m_pos = 0;
				m_pEdges[axis][0].m_handle = 0;
				m_pEdges[axis][1].m_pos = m_handleSentinel;
				m_pEdges[axis][1].m_handle = 0;
#if DEBUG_BROADPHASE
		debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE

			}

		}

		~btAxisSweep3()
		{
			if( m_raycastAccelerator != null )
			{
				m_nullPairCache = null;//.~btOverlappingPairCache();
				m_raycastAccelerator = null;//.~btDbvtBroadphase();
			}

			m_pHandles = null;

			if( m_ownsPairCache )
			{
				m_pairCache = null;//.~btOverlappingPairCache();
			}
		}


		void quantize( ushort[] outdata, ref btVector3 point, int isMax )
		{
#if OLD_CLAMPING_METHOD
	///problem with this clamping method is that the floating point during quantization might still go outside the range [(0|isMax) .. (m_handleSentinelm_bpHandleMask]|isMax]
	///see http://code.google.com/p/bullet/issues/detail?id=87
	btVector3 clampedPoint(point);
	clampedPoint.setMax(m_worldAabbMin);
	clampedPoint.setMin(m_worldAabbMax);
	btVector3 v = (clampedPoint - m_worldAabbMin) * m_quantize;
	out = (ushort)(((ushort)v.x & m_bpHandleMask) | isMax);
	out[1] = (ushort)(((ushort)v.y & m_bpHandleMask) | isMax);
	out[2] = (ushort)(((ushort)v.z  m_bpHandleMask) | isMax);
#else
			btVector3 v = ( point - m_worldAabbMin ) * m_quantize;
			outdata[0] = ( v[0] <= 0 ) ? (ushort)isMax : ( v[0] >= m_handleSentinel ) ? (ushort)( ( m_handleSentinel & m_bpHandleMask ) | isMax ) : (ushort)( ( (ushort)v[0] & m_bpHandleMask ) | isMax );
			outdata[1] = ( v[1] <= 0 ) ? (ushort)isMax : ( v[1] >= m_handleSentinel ) ? (ushort)( ( m_handleSentinel & m_bpHandleMask ) | isMax ) : (ushort)( ( (ushort)v[1] & m_bpHandleMask ) | isMax );
			outdata[2] = ( v[2] <= 0 ) ? (ushort)isMax : ( v[2] >= m_handleSentinel ) ? (ushort)( ( m_handleSentinel & m_bpHandleMask ) | isMax ) : (ushort)( ( (ushort)v[2] & m_bpHandleMask ) | isMax );
#endif //OLD_CLAMPING_METHOD
		}



		internal ushort allocHandle()
		{
			Debug.Assert( m_firstFreeHandle != null );

			ushort handle = m_firstFreeHandle;
			m_firstFreeHandle = getHandle( handle ).GetNextFree();
			m_numHandles++;

			return handle;
		}

		internal void freeHandle( ushort handle )
		{
			Debug.Assert( handle > 0 && handle < m_maxHandles );

			getHandle( handle ).SetNextFree( m_firstFreeHandle );
			m_firstFreeHandle = handle;

			m_numHandles--;
		}



		ushort addHandle( ref btVector3 aabbMin, ref btVector3 aabbMax, object pOwner
			, btBroadphaseProxy.CollisionFilterGroups collisionFilterGroup
			, btBroadphaseProxy.CollisionFilterGroups collisionFilterMask, btDispatcher dispatcher, object multiSapProxy )
		{
			// quantize the bounds
			ushort[] min = new ushort[3], max = new ushort[3];
			quantize( min, ref aabbMin, 0 );
			quantize( max, ref aabbMax, 1 );

			// allocate a handle
			ushort handle = allocHandle();


			Handle pHandle = getHandle( handle );

			pHandle.m_uniqueId = handle;
			//pHandle.m_pOverlaps = 0;
			pHandle.m_clientObject = pOwner;
			pHandle.m_collisionFilterGroup = collisionFilterGroup;
			pHandle.m_collisionFilterMask = collisionFilterMask;
			pHandle.m_multiSapParentProxy = multiSapProxy;

			// compute current limit of edge arrays
			ushort limit = (ushort)( m_numHandles * 2 );


			// insert new edges just inside the max boundary edge
			for( ushort axis = 0; axis < 3; axis++ )
			{

				m_pHandles[0].m_maxEdges[axis] += 2;

				m_pEdges[axis][limit + 1] = m_pEdges[axis][limit - 1];

				m_pEdges[axis][limit - 1].m_pos = min[axis];
				m_pEdges[axis][limit - 1].m_handle = handle;

				m_pEdges[axis][limit].m_pos = max[axis];
				m_pEdges[axis][limit].m_handle = handle;

				pHandle.m_minEdges[axis] = (ushort)( limit - 1 );
				pHandle.m_maxEdges[axis] = limit;
			}

			// now sort the new edges to their correct position
			sortMinDown( 0, pHandle.m_minEdges[0], dispatcher, false );
			sortMaxDown( 0, pHandle.m_maxEdges[0], dispatcher, false );
			sortMinDown( 1, pHandle.m_minEdges[1], dispatcher, false );
			sortMaxDown( 1, pHandle.m_maxEdges[1], dispatcher, false );
			sortMinDown( 2, pHandle.m_minEdges[2], dispatcher, true );
			sortMaxDown( 2, pHandle.m_maxEdges[2], dispatcher, true );


			return handle;
		}



		void removeHandle( ushort handle, btDispatcher dispatcher )
		{

			Handle pHandle = getHandle( handle );

			//explicitly remove the pairs containing the proxy
			//we could do it also in the sortMinUp (passing true)
			///@todo: compare performance
			if( !m_pairCache.hasDeferredRemoval() )
			{
				m_pairCache.removeOverlappingPairsContainingProxy( pHandle, dispatcher );
			}

			// compute current limit of edge arrays
			int limit = (int)( m_numHandles * 2 );

			int axis;

			for( axis = 0; axis < 3; axis++ )
			{
				m_pHandles[0].m_maxEdges[axis] -= 2;
			}

			// remove the edges by sorting them up to the end of the list
			for( axis = 0; axis < 3; axis++ )
			{
				Edge[] pEdges = m_pEdges[axis];
				ushort max = pHandle.m_maxEdges[axis];
				pEdges[max].m_pos = m_handleSentinel;

				sortMaxUp( axis, max, dispatcher, false );


				ushort i = pHandle.m_minEdges[axis];
				pEdges[i].m_pos = m_handleSentinel;


				sortMinUp( axis, i, dispatcher, false );

				pEdges[limit - 1].m_handle = 0;
				pEdges[limit - 1].m_pos = m_handleSentinel;

#if DEBUG_BROADPHASE
			debugPrintAxis(axis,false);
#endif //DEBUG_BROADPHASE


			}


			// free the handle
			freeHandle( handle );


		}


		public void resetPool( btDispatcher dispatcher)
		{
			if( m_numHandles == 0 )
			{
				m_firstFreeHandle = 1;
				{
					for( ushort i = m_firstFreeHandle; i < m_maxHandles; i++ )
						m_pHandles[i].SetNextFree( (ushort)( i + 1 ) );
					m_pHandles[m_maxHandles - 1].SetNextFree( 0 );
				}
			}
		}


		public void calculateOverlappingPairs( btDispatcher dispatcher )
		{

			if( m_pairCache.hasDeferredRemoval() )
			{

				btBroadphasePairArray overlappingPairArray = m_pairCache.getOverlappingPairArray();

				//perform a sort, to find duplicates and to sort 'invalid' pairs to the end
				overlappingPairArray.quickSort( btBroadphasePair.qsCompare );

				overlappingPairArray.Count = ( overlappingPairArray.Count - m_invalidPair );
				m_invalidPair = 0;


				int i;

				btBroadphasePair previousPair =  new btBroadphasePair();
				previousPair.m_pProxy0 = null;
				previousPair.m_pProxy1 = null;
				previousPair.m_algorithm = null;


				for( i = 0; i < overlappingPairArray.Count; i++ )
				{

					btBroadphasePair pair = overlappingPairArray[i];

					bool isDuplicate = ( pair == previousPair );

					previousPair = pair;

					bool needsRemoval = false;

					if( !isDuplicate )
					{
						///important to use an AABB test that is consistent with the broadphase
						bool hasOverlap = testAabbOverlap( pair.m_pProxy0, pair.m_pProxy1 );

						if( hasOverlap )
						{
							needsRemoval = false;//callback.processOverlap(pair);
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
						m_pairCache.cleanOverlappingPair( pair, dispatcher );

						//		m_overlappingPairArray.swap(i,m_overlappingPairArray.Count-1);
						//		m_overlappingPairArray.pop_back();
						pair.m_pProxy0 = null;
						pair.m_pProxy1 = null;
						m_invalidPair++;
						btHashedOverlappingPairCache.gOverlappingPairs--;
					}

				}

				///if you don't like to skip the invalid pairs in the array, execute following code:
#if CLEAN_INVALID_PAIRS

				//perform a sort, to sort 'invalid' pairs to the end
				overlappingPairArray.quickSort( btBroadphasePair.qsCompare );

				overlappingPairArray.Count = ( overlappingPairArray.Count - m_invalidPair );
				m_invalidPair = 0;
#endif//CLEAN_INVALID_PAIRS

				//Console.WriteLine("overlappingPairArray.Count=%d\n",overlappingPairArray.Count);
			}

		}



		bool testAabbOverlap( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			Handle pHandleA = (Handle)( proxy0 );
			Handle pHandleB = (Handle)( proxy1 );

			//optimization 1: check the array index (memory address), instead of the m_pos

			for( int axis = 0; axis < 3; axis++ )
			{
				if( pHandleA.m_maxEdges[axis] < pHandleB.m_minEdges[axis] ||
					pHandleB.m_maxEdges[axis] < pHandleA.m_minEdges[axis] )
				{
					return false;
				}
			}
			return true;
		}


		bool testOverlap2D( Handle pHandleA, Handle pHandleB, int axis0, int axis1 )
		{
			//optimization 1: check the array index (memory address), instead of the m_pos

			if( pHandleA.m_maxEdges[axis0] < pHandleB.m_minEdges[axis0] ||
				pHandleB.m_maxEdges[axis0] < pHandleA.m_minEdges[axis0] ||
				pHandleA.m_maxEdges[axis1] < pHandleB.m_minEdges[axis1] ||
				pHandleB.m_maxEdges[axis1] < pHandleA.m_minEdges[axis1] )
			{
				return false;
			}
			return true;
		}


		void updateHandle( ushort handle, ref btVector3 aabbMin, ref btVector3 aabbMax, btDispatcher dispatcher )
		{
			//	Debug.Assert(bounds.IsFinite());
			//Debug.Assert(bounds.HasVolume());

			Handle pHandle = getHandle( handle );

			// quantize the new bounds
			ushort[] min = new ushort[3], max = new ushort[3];
			quantize( min, ref aabbMin, 0 );
			quantize( max, ref aabbMax, 1 );

			// update changed edges
			for( int axis = 0; axis < 3; axis++ )
			{
				ushort emin = pHandle.m_minEdges[axis];
				ushort emax = pHandle.m_maxEdges[axis];

				int dmin = (int)min[axis] - (int)m_pEdges[axis][emin].m_pos;
				int dmax = (int)max[axis] - (int)m_pEdges[axis][emax].m_pos;

				m_pEdges[axis][emin].m_pos = min[axis];
				m_pEdges[axis][emax].m_pos = max[axis];

				// expand (only adds overlaps)
				if( dmin < 0 )
					sortMinDown( axis, emin, dispatcher, true );

				if( dmax > 0 )
					sortMaxUp( axis, emax, dispatcher, true );

				// shrink (only removes overlaps)
				if( dmin > 0 )
					sortMinUp( axis, emin, dispatcher, true );

				if( dmax < 0 )
					sortMaxDown( axis, emax, dispatcher, true );

#if DEBUG_BROADPHASE
	debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE
			}


		}




		// sorting a min edge downwards can only ever *add* overlaps

		void sortMinDown( int axis, ushort edge, btDispatcher  dispatcher , bool updateOverlaps )
		{
			Edge[] edges = m_pEdges[axis];
			Edge pEdge = edges[edge];
			Edge pPrev = edges[edge - 1];
			Handle pHandleEdge = getHandle( pEdge.m_handle );

			while( pEdge.m_pos < pPrev.m_pos )
			{
				Handle pHandlePrev = getHandle( pPrev.m_handle );

				if( pPrev.IsMax() == 0 )
				{
					// if previous edge is a maximum check the bounds and add an overlap if necessary
					int axis1 = ( 1 << axis ) & 3;
					int axis2 = ( 1 << axis1 ) & 3;
					if( updateOverlaps && testOverlap2D( pHandleEdge, pHandlePrev, axis1, axis2 ) )
					{
						m_pairCache.addOverlappingPair( pHandleEdge, pHandlePrev );
						if( m_userPairCallback != null )
							m_userPairCallback.addOverlappingPair( pHandleEdge, pHandlePrev );

						//AddOverlap(pEdge.m_handle, pPrev.m_handle);

					}

					// update edge reference in other handle
					pHandlePrev.m_maxEdges[axis]++;
				}
				else
					pHandlePrev.m_minEdges[axis]++;

				pHandleEdge.m_minEdges[axis]--;

				// swap the edges
				edges[edge] = pPrev;
				edges[edge - 1] = pEdge;

				// decrement
				edge--;
				pEdge = edges[edge];
				pPrev = edges[edge - 1];
			}

#if DEBUG_BROADPHASE
	debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE

		}

		// sorting a min edge upwards can only ever *remove* overlaps

		void sortMinUp( int axis, ushort edge, btDispatcher dispatcher, bool updateOverlaps )
		{
			Edge[] edges = m_pEdges[axis];
			Edge pEdge = edges[edge];
			Edge pNext = edges[edge + 1];
			Handle pHandleEdge = getHandle( pEdge.m_handle );

			while( pNext.m_handle != 0 && ( pEdge.m_pos >= pNext.m_pos ) )
			{
				Handle pHandleNext = getHandle( pNext.m_handle );

				if( pNext.IsMax() == 0 )
				{
					Handle handle0 = getHandle( pEdge.m_handle );
					Handle handle1 = getHandle( pNext.m_handle );
					int axis1 = ( 1 << axis ) & 3;
					int axis2 = ( 1 << axis1 ) & 3;

					// if next edge is maximum remove any overlap between the two handles
					if( updateOverlaps
#if USE_OVERLAP_TEST_ON_REMOVES
				&& testOverlap2D( handle0, handle1, axis1, axis2 )
#endif //USE_OVERLAP_TEST_ON_REMOVES
				)
					{


						m_pairCache.removeOverlappingPair( handle0, handle1, dispatcher );
						if( m_userPairCallback == null )
							m_userPairCallback.removeOverlappingPair( handle0, handle1, dispatcher );

					}


					// update edge reference in other handle
					pHandleNext.m_maxEdges[axis]--;
				}
				else
					pHandleNext.m_minEdges[axis]--;

				pHandleEdge.m_minEdges[axis]++;

				// swap the edges
				edges[edge] = pNext;
				edges[edge + 1] = pEdge;

				// increment
				pEdge = edges[edge];
				pNext = edges[edge + 1];
			}


		}

		// sorting a max edge downwards can only ever *remove* overlaps

		void sortMaxDown( int axis, ushort edge, btDispatcher dispatcher, bool updateOverlaps )
		{
			Edge[] edges = m_pEdges[axis];
            Edge pEdge = edges[ edge];
			Edge pPrev = edges[edge - 1];
			Handle pHandleEdge = getHandle( pEdge.m_handle );

			while( pEdge.m_pos < pPrev.m_pos )
			{
				Handle pHandlePrev = getHandle( pPrev.m_handle );

				if( pPrev.IsMax() == 0 )
				{
					// if previous edge was a minimum remove any overlap between the two handles
					Handle handle0 = getHandle( pEdge.m_handle );
					Handle handle1 = getHandle( pPrev.m_handle );
					int axis1 = ( 1 << axis ) & 3;
					int axis2 = ( 1 << axis1 ) & 3;

					if( updateOverlaps
#if USE_OVERLAP_TEST_ON_REMOVES
				&& testOverlap2D( handle0, handle1, axis1, axis2 )
#endif //USE_OVERLAP_TEST_ON_REMOVES
				)
					{
						//this is done during the overlappingpairarray iteration/narrowphase collision


						m_pairCache.removeOverlappingPair( handle0, handle1, dispatcher );
						if( m_userPairCallback != null )
							m_userPairCallback.removeOverlappingPair( handle0, handle1, dispatcher );



					}

					// update edge reference in other handle
					pHandlePrev.m_minEdges[axis]++; ;
				}
				else
					pHandlePrev.m_maxEdges[axis]++;

				pHandleEdge.m_maxEdges[axis]--;

				edges[edge] = pPrev;
				edges[edge - 1] = pEdge;
				// swap the edges

				// decrement
				edge--;
				pEdge = edges[edge];
				pPrev = edges[edge - 1];
			}


#if DEBUG_BROADPHASE
	debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE

		}

		// sorting a max edge upwards can only ever *add* overlaps

		void sortMaxUp( int axis, ushort edge, btDispatcher  dispatcher , bool updateOverlaps )
		{
			Edge[] edges = m_pEdges[axis];
            Edge pEdge = edges[ edge];
			Edge pNext = edges[edge + 1];
			Handle pHandleEdge = getHandle( pEdge.m_handle );

			while( pNext.m_handle != 0 && ( pEdge.m_pos >= pNext.m_pos ) )
			{
				Handle pHandleNext = getHandle( pNext.m_handle );

				int axis1 = ( 1 << axis ) & 3;
				int axis2 = ( 1 << axis1 ) & 3;

				if( pNext.IsMax() == 0 )
				{
					// if next edge is a minimum check the bounds and add an overlap if necessary
					if( updateOverlaps && testOverlap2D( pHandleEdge, pHandleNext, axis1, axis2 ) )
					{
						Handle handle0 = getHandle( pEdge.m_handle );
						Handle handle1 = getHandle( pNext.m_handle );
						m_pairCache.addOverlappingPair( handle0, handle1 );
						if( m_userPairCallback != null )
							m_userPairCallback.addOverlappingPair( handle0, handle1 );
					}

					// update edge reference in other handle
					pHandleNext.m_minEdges[axis]--;
				}
				else
					pHandleNext.m_maxEdges[axis]--;

				pHandleEdge.m_maxEdges[axis]++;

				// swap the edges
				edges[edge] = pNext;
				edges[edge + 1] = pEdge;

				// increment
				edge++;
				pEdge = edges[edge];
				pNext = edges[edge + 1];

				//next++;
			}


		}



		////////////////////////////////////////////////////////////////////


	}
}