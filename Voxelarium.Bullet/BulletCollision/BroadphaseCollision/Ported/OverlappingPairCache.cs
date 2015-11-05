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
using Bullet.Types;
using System.Diagnostics;

namespace Bullet.Collision.BroadPhase
{


	internal class btBroadphasePairArray : btList<btBroadphasePair>
	{
	}

	internal interface btOverlapCallback
	{
		//return true for deletion of the pair
		bool processOverlap( btBroadphasePair pair );
	};

	internal abstract class btOverlapFilterCallback
	{
		// return true when pairs need collision
		internal abstract bool needBroadphaseCollision( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );
	};



	///The btOverlappingPairCache provides an interface for overlapping pair management (add, remove, storage), used by the btBroadphaseInterface broadphases.
	///The btHashedOverlappingPairCache and btSortedOverlappingPairCache classes are two implementations.
	public abstract class btOverlappingPairCache : btOverlappingPairCallback
	{
		//
		internal abstract btBroadphasePair[] getOverlappingPairArrayPtr();
		internal abstract btBroadphasePairArray getOverlappingPairArray();
		internal abstract void cleanOverlappingPair( btBroadphasePair pair, btDispatcher dispatcher );
		internal abstract int getNumOverlappingPairs();
		internal abstract void cleanProxyFromPairs( btBroadphaseProxy proxy, btDispatcher dispatcher );
		internal abstract void setOverlapFilterCallback( btOverlapFilterCallback callback );
		internal abstract void processAllOverlappingPairs( btOverlapCallback callback, btDispatcher dispatcher );
		internal abstract btBroadphasePair findPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );
		internal abstract bool hasDeferredRemoval();
		internal abstract void setInternalGhostPairCallback( btOverlappingPairCallback ghostPairCallback );
		internal abstract void sortOverlappingPairs( btDispatcher dispatcher );

	};

	/// Hash-space based Pair Cache, thanks to Erin Catto, Box2D, http://www.box2d.org, and Pierre Terdiman, Codercorner, http://codercorner.com
	internal class btHashedOverlappingPairCache : btOverlappingPairCache
	{
		internal static int gRemovePairs;
		internal static int gAddedPairs;
		internal static int gFindPairs;
		internal static int gOverlappingPairs;
		const int BT_NULL_PAIR = -1;

		btBroadphasePairArray m_overlappingPairArray = new btBroadphasePairArray();
		btOverlapFilterCallback m_overlapFilterCallback;


		protected btList<int> m_hashTable = new btList<int>();
		protected btList<int> m_next = new btList<int>();
		protected btOverlappingPairCallback m_ghostPairCallback;


		//internal void removeOverlappingPairsContainingProxy( btBroadphaseProxy proxy, btDispatcher dispatcher );


#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool needsBroadphaseCollision( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			if( m_overlapFilterCallback != null )
				return m_overlapFilterCallback.needBroadphaseCollision( proxy0, proxy1 );

			bool collides = ( proxy0.m_collisionFilterGroup & proxy1.m_collisionFilterMask ) != 0;
			collides = collides && ( proxy1.m_collisionFilterGroup & proxy0.m_collisionFilterMask ) != 0;

			return collides;
		}

		// Add a pair and return the new pair. If the pair already exists,
		// no new pair is created and the old one is returned.
		internal override btBroadphasePair addOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			btHashedOverlappingPairCache.gAddedPairs++;

			if( !needsBroadphaseCollision( proxy0, proxy1 ) )
				return null;

			return internalAddPair( proxy0, proxy1 );
		}



		//internal void cleanProxyFromPairs( btBroadphaseProxy proxy, btDispatcher dispatcher );


		internal override btBroadphasePair[] getOverlappingPairArrayPtr()
		{
			return m_overlappingPairArray.InternalArray;
		}



		internal override btBroadphasePairArray getOverlappingPairArray()
		{
			return m_overlappingPairArray;
		}

		//internal void cleanOverlappingPair( btBroadphasePair pair, btDispatcher dispatcher );


		internal int GetCount() { return m_overlappingPairArray.Count; }
		//	btBroadphasePair GetPairs() { return m_pairs; }

		internal btOverlapFilterCallback getOverlapFilterCallback()
		{
			return m_overlapFilterCallback;
		}

		internal override void setOverlapFilterCallback( btOverlapFilterCallback callback )
		{
			m_overlapFilterCallback = callback;
		}

		internal override int getNumOverlappingPairs()
		{
			return m_overlappingPairArray.Count;
		}

		//btBroadphasePair internalAddPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );

		//void growTables();

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		bool equalsPair( btBroadphasePair pair, btBroadphaseProxy proxyId1, btBroadphaseProxy proxyId2 )
		{
			return ( pair.m_pProxy0 == proxyId1 && pair.m_pProxy1 == proxyId2 )
				|| ( pair.m_pProxy0 == proxyId2 && pair.m_pProxy1 == proxyId1 );
		}

		/*
		// Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
		// This assumes proxyId1 and proxyId2 are 16-bit.
		int getHash(int proxyId1, int proxyId2)
		{
			int key = (proxyId2 << 16) | proxyId1;
			key = ~key + (key << 15);
			key = key ^ (key >> 12);
			key = key + (key << 2);
			key = key ^ (key >> 4);
			key = key * 2057;
			key = key ^ (key >> 16);
			return key;
		}
		*/



#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		uint getHash( uint proxyId1, uint proxyId2 )
		{
			int key = (int)( ( (uint)proxyId1 ) | ( ( (uint)proxyId2 ) << 16 ) );
			// Thomas Wang's hash

			key += ~( key << 15 );
			key ^= ( key >> 10 );
			key += ( key << 3 );
			key ^= ( key >> 6 );
			key += ~( key << 11 );
			key ^= ( key >> 16 );
			return (uint)( key );
		}





#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		btBroadphasePair internalFindPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, int hash )
		{
#if false // wrong, 'equalsPair' use unsorted uids, copy-past devil striked again. Nat.
		if (proxyId1 > proxyId2) 
			btSwap(proxyId1, proxyId2);
#endif

			int index = m_hashTable[hash];

			while( index != BT_NULL_PAIR && equalsPair( m_overlappingPairArray[index], proxy0, proxy1 ) == false )
			{
				index = m_next[index];
			}

			if( index == BT_NULL_PAIR )
			{
				return null;
			}

			Debug.Assert( index < m_overlappingPairArray.Count );

			return m_overlappingPairArray[index];
		}

		internal override bool hasDeferredRemoval()
		{
			return false;
		}

		internal override void setInternalGhostPairCallback( btOverlappingPairCallback ghostPairCallback )
		{
			m_ghostPairCallback = ghostPairCallback;
		}

		//internal override void sortOverlappingPairs( btDispatcher dispatcher );

		internal btHashedOverlappingPairCache()
		{
			m_overlapFilterCallback = ( null );
			m_ghostPairCallback = ( null );
			int initialAllocatedSize = 2;
			m_overlappingPairArray.Capacity = ( initialAllocatedSize );
			growTables();
		}



		internal override void cleanOverlappingPair( btBroadphasePair pair, btDispatcher dispatcher )
		{
			if( pair.m_algorithm != null && dispatcher != null )
			{
				{
					//pair.m_algorithm.~btCollisionAlgorithm();
					dispatcher.freeCollisionAlgorithm( pair.m_algorithm );
					pair.m_algorithm = null;
				}
			}
		}




		struct CleanPairCallback : btOverlapCallback
		{
			btBroadphaseProxy m_cleanProxy;
			btOverlappingPairCache m_pairCache;
			btDispatcher m_dispatcher;

			internal
			CleanPairCallback( btBroadphaseProxy cleanProxy, btOverlappingPairCache pairCache, btDispatcher dispatcher )
			{
				m_cleanProxy = ( cleanProxy );
				m_pairCache = ( pairCache );
				m_dispatcher = ( dispatcher );
			}
			public bool processOverlap( btBroadphasePair pair )
			{
				if( ( pair.m_pProxy0 == m_cleanProxy ) ||
					( pair.m_pProxy1 == m_cleanProxy ) )
				{
					m_pairCache.cleanOverlappingPair( pair, m_dispatcher );
				}
				return false;
			}

		};

		internal override void cleanProxyFromPairs( btBroadphaseProxy proxy, btDispatcher dispatcher )
		{
			CleanPairCallback cleanPairs = new CleanPairCallback( proxy, this, dispatcher );
			processAllOverlappingPairs( cleanPairs, dispatcher );
		}



		struct RemovePairCallback : btOverlapCallback
		{
			btBroadphaseProxy m_obsoleteProxy;

			internal
				RemovePairCallback( btBroadphaseProxy obsoleteProxy )
			{
				m_obsoleteProxy = ( obsoleteProxy );
			}
			public bool processOverlap( btBroadphasePair pair )
			{
				return ( ( pair.m_pProxy0 == m_obsoleteProxy ) ||
					( pair.m_pProxy1 == m_obsoleteProxy ) );
			}

		};


		internal override void removeOverlappingPairsContainingProxy( btBroadphaseProxy proxy, btDispatcher dispatcher )
		{
			RemovePairCallback removeCallback = new RemovePairCallback( proxy );
			processAllOverlappingPairs( removeCallback, dispatcher );
		}





		internal override btBroadphasePair findPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			btHashedOverlappingPairCache.gFindPairs++;
			if( proxy0.GetHashCode() > proxy1.GetHashCode() )
				btScalar.btSwap( ref proxy0, ref proxy1 );
			//int proxyId1 = proxy0.getUid();
			//int proxyId2 = proxy1.getUid();

			/*if (proxyId1 > proxyId2) 
				btSwap(proxyId1, proxyId2);*/

			int hash = (int)( getHash( (uint)( proxy0.GetHashCode() ), (uint)( proxy1.GetHashCode() ) )
				& ( m_overlappingPairArray.Count - 1 ) );

			if( hash >= m_hashTable.Count )
			{
				return null;
			}

			int index = m_hashTable[hash];
			while( index != BT_NULL_PAIR && equalsPair( m_overlappingPairArray[index], proxy0, proxy1 ) == false )
			{
				index = m_next[index];
			}

			if( index == BT_NULL_PAIR )
			{
				return null;
			}

			Debug.Assert( index < m_overlappingPairArray.Count );

			return m_overlappingPairArray[index];
		}

		//#include <stdio.h>

		void growTables()
		{

			int newCapacity = m_overlappingPairArray.Capacity;

			if( m_hashTable.Count < newCapacity )
			{
				//grow hashtable and next table
				int curHashtableSize = m_hashTable.Count;

				m_hashTable.Capacity = ( newCapacity );
				m_next.Capacity = ( newCapacity );


				int i;

				for( i = 0; i < newCapacity; ++i )
				{
					m_hashTable[i] = BT_NULL_PAIR;
				}
				for( i = 0; i < newCapacity; ++i )
				{
					m_next[i] = BT_NULL_PAIR;
				}

				for( i = 0; i < curHashtableSize; i++ )
				{

					btBroadphasePair pair = m_overlappingPairArray[i];
					int proxyId1 = pair.m_pProxy0.GetHashCode();
					int proxyId2 = pair.m_pProxy1.GetHashCode();
					/*if (proxyId1 > proxyId2) 
						btSwap(proxyId1, proxyId2);*/
					int hashValue = (int)( getHash( (uint)( proxyId1 ), (uint)( proxyId2 ) )
									& ( m_overlappingPairArray.Capacity - 1 ) );    // New hash value with new mask
					m_next[i] = m_hashTable[hashValue];
					m_hashTable[hashValue] = i;
				}


			}
		}

		btBroadphasePair internalAddPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			if( proxy0.GetHashCode() > proxy1.GetHashCode() )
				btScalar.btSwap( ref proxy0, ref proxy1 );
			int proxyId1 = proxy0.GetHashCode();
			int proxyId2 = proxy1.GetHashCode();

			/*if (proxyId1 > proxyId2) 
				btSwap(proxyId1, proxyId2);*/

			int hash = (int)( getHash( (uint)( proxyId1 ), (uint)( proxyId2 ) ) & ( m_overlappingPairArray.Capacity - 1 ) ); // New hash value with new mask


			btBroadphasePair pair = internalFindPair( proxy0, proxy1, hash );
			if( pair != null )
			{
				return pair;
			}
			/*for(int i=0;i<m_overlappingPairArray.Count;++i)
				{
				if(	(m_overlappingPairArray[i].m_pProxy0==proxy0)&&
					(m_overlappingPairArray[i].m_pProxy1==proxy1))
					{
					printf("Adding duplicated %u<>%u\r\n",proxyId1,proxyId2);
					internalFindPair(proxy0, proxy1, hash);
					}
				}*/
			int count = m_overlappingPairArray.Count;
			int oldCapacity = m_overlappingPairArray.Capacity;
			//void* mem = &m_overlappingPairArray.expandNonInitializing();
			pair = BulletGlobals.BroadphasePairPool.Get();
			m_overlappingPairArray.Add( pair );

			//this is where we add an actual pair, so also call the 'ghost'
			if( m_ghostPairCallback != null )
				m_ghostPairCallback.addOverlappingPair( proxy0, proxy1 );

			int newCapacity = m_overlappingPairArray.Capacity;

			if( oldCapacity < newCapacity )
			{
				growTables();
				//hash with new capacity
				hash = (int)( getHash( (uint)( proxyId1 ), (uint)( proxyId2 ) ) & ( m_overlappingPairArray.Capacity - 1 ) );
			}

			pair.Initialize( proxy0, proxy1 );
			//	pair.m_pProxy0 = proxy0;
			//	pair.m_pProxy1 = proxy1;
			pair.m_algorithm = null;
			pair.m_internalInfo1 = null;


			m_next[count] = m_hashTable[hash];
			m_hashTable[hash] = count;

			return pair;
		}



		internal override object removeOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, btDispatcher dispatcher )
		{
			btHashedOverlappingPairCache.gRemovePairs++;
			if( proxy0.GetHashCode() > proxy1.GetHashCode() )
				btScalar.btSwap( ref proxy0, ref proxy1 );
			int proxyId1 = proxy0.GetHashCode();
			int proxyId2 = proxy1.GetHashCode();

			/*if (proxyId1 > proxyId2) 
				btSwap(proxyId1, proxyId2);*/

			int hash = (int)( getHash( (uint)( proxyId1 ), (uint)( proxyId2 ) )
										& ( m_overlappingPairArray.Capacity - 1 ) );

			btBroadphasePair pair = internalFindPair( proxy0, proxy1, hash );
			if( pair == null )
			{
				return null;
			}

			cleanOverlappingPair( pair, dispatcher );

			object userData = pair.m_internalInfo1;

			Debug.Assert( pair.m_pProxy0.GetHashCode() == proxyId1 );
			Debug.Assert( pair.m_pProxy1.GetHashCode() == proxyId2 );

			int pairIndex = m_overlappingPairArray.IndexOf( pair );
			Debug.Assert( pairIndex < m_overlappingPairArray.Count );

			// Remove the pair from the hash table.
			int index = m_hashTable[hash];
			Debug.Assert( index != BT_NULL_PAIR );

			int previous = BT_NULL_PAIR;
			while( index != pairIndex )
			{
				previous = index;
				index = m_next[index];
			}

			if( previous != BT_NULL_PAIR )
			{
				Debug.Assert( m_next[previous] == pairIndex );
				m_next[previous] = m_next[pairIndex];
			}
			else
			{
				m_hashTable[hash] = m_next[pairIndex];
			}

			// We now move the last pair into spot of the
			// pair being removed. We need to fix the hash
			// table indices to support the move.

			int lastPairIndex = m_overlappingPairArray.Count - 1;

			if( m_ghostPairCallback == null )
				m_ghostPairCallback.removeOverlappingPair( proxy0, proxy1, dispatcher );

			// If the removed pair is the last pair, we are done.
			if( lastPairIndex == pairIndex )
			{
				m_overlappingPairArray.Count--;
				return userData;
			}

			// Remove the last pair from the hash table.
			btBroadphasePair last = m_overlappingPairArray[lastPairIndex];
			/* missing swap here too, Nat. */
			int lastHash = (int)( getHash( (uint)( last.m_pProxy0.GetHashCode() ), (uint)( last.m_pProxy1.GetHashCode() ) )
										& ( m_overlappingPairArray.Capacity - 1 ) );

			index = m_hashTable[lastHash];
			Debug.Assert( index != BT_NULL_PAIR );

			previous = BT_NULL_PAIR;
			while( index != lastPairIndex )
			{
				previous = index;
				index = m_next[index];
			}

			if( previous != BT_NULL_PAIR )
			{
				Debug.Assert( m_next[previous] == lastPairIndex );
				m_next[previous] = m_next[lastPairIndex];
			}
			else
			{
				m_hashTable[lastHash] = m_next[lastPairIndex];
			}

			// Copy the last pair into the remove pair's spot.
			m_overlappingPairArray[pairIndex] = m_overlappingPairArray[lastPairIndex];

			// Insert the last pair into the hash table
			m_next[pairIndex] = m_hashTable[lastHash];
			m_hashTable[lastHash] = pairIndex;

			m_overlappingPairArray.Count--;

			return userData;
		}
		//#include <stdio.h>

		internal override void processAllOverlappingPairs( btOverlapCallback callback, btDispatcher dispatcher )
		{

			int i;

			//	printf("m_overlappingPairArray.Count=%d\n",m_overlappingPairArray.Count);
			for( i = 0; i < m_overlappingPairArray.Count; )
			{

				btBroadphasePair pair = m_overlappingPairArray[i];
				if( callback.processOverlap( pair ) )
				{
					removeOverlappingPair( pair.m_pProxy0, pair.m_pProxy1, dispatcher );
					btHashedOverlappingPairCache.gOverlappingPairs--;
				}
				else
				{
					i++;
				}
			}
		}

		internal override void sortOverlappingPairs( btDispatcher dispatcher )
		{
			///need to keep hashmap in sync with pair address, so rebuild all
			btBroadphasePairArray tmpPairs = new btBroadphasePairArray();
			int i;
			for( i = 0; i < m_overlappingPairArray.Count; i++ )
			{
				tmpPairs.Add( m_overlappingPairArray[i] );
			}

			for( i = 0; i < tmpPairs.Count; i++ )
			{
				removeOverlappingPair( tmpPairs[i].m_pProxy0, tmpPairs[i].m_pProxy1, dispatcher );
			}

			for( i = 0; i < m_next.Count; i++ )
			{
				m_next[i] = BT_NULL_PAIR;
			}

			tmpPairs.quickSort( btBroadphasePair.qsCompare );

			for( i = 0; i < tmpPairs.Count; i++ )
			{
				addOverlappingPair( tmpPairs[i].m_pProxy0, tmpPairs[i].m_pProxy1 );
			}
		}
	};



	///btSortedOverlappingPairCache maintains the objects with overlapping AABB
	///Typically managed by the Broadphase, Axis3Sweep or btSimpleBroadphase
	class btSortedOverlappingPairCache : btOverlappingPairCache
	{

		//avoid brute-force finding all the time
		protected btBroadphasePairArray m_overlappingPairArray;

		//during the dispatch, check that user doesn't destroy/create proxy
		protected bool m_blockedForChanges;

		///by default, do the removal during the pair traversal
		protected bool m_hasDeferredRemoval;

		//if set, use the callback instead of the built in filter in needBroadphaseCollision
		protected btOverlapFilterCallback m_overlapFilterCallback;

		protected btOverlappingPairCallback m_ghostPairCallback;


		//internal btSortedOverlappingPairCache();
		//virtual ~btSortedOverlappingPairCache();

		//virtual void processAllOverlappingPairs( btOverlapCallback*, btDispatcher dispatcher );
		/*
		void* removeOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, btDispatcher dispatcher );

		void cleanOverlappingPair( btBroadphasePair pair, btDispatcher dispatcher );

		btBroadphasePair addOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );

		btBroadphasePair findPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );


		void cleanProxyFromPairs( btBroadphaseProxy proxy, btDispatcher dispatcher );

		void removeOverlappingPairsContainingProxy( btBroadphaseProxy proxy, btDispatcher dispatcher );
		*/

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool needsBroadphaseCollision( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			if( m_overlapFilterCallback != null )
				return m_overlapFilterCallback.needBroadphaseCollision( proxy0, proxy1 );

			bool collides = ( proxy0.m_collisionFilterGroup & proxy1.m_collisionFilterMask ) != 0;
			collides = collides && ( proxy1.m_collisionFilterGroup & proxy0.m_collisionFilterMask ) != 0;

			return collides;
		}

		internal override btBroadphasePairArray getOverlappingPairArray()
		{
			return m_overlappingPairArray;
		}

		internal override btBroadphasePair[] getOverlappingPairArrayPtr()
		{
			return m_overlappingPairArray.InternalArray;
		}


		internal override int getNumOverlappingPairs()
		{
			return m_overlappingPairArray.Count;
		}

		btOverlapFilterCallback getOverlapFilterCallback()
		{
			return m_overlapFilterCallback;
		}

		internal override void setOverlapFilterCallback( btOverlapFilterCallback callback )
		{
			m_overlapFilterCallback = callback;
		}

		internal override bool hasDeferredRemoval()
		{
			return m_hasDeferredRemoval;
		}

		internal override void setInternalGhostPairCallback( btOverlappingPairCallback ghostPairCallback )
		{
			m_ghostPairCallback = ghostPairCallback;
		}

		//virtual void sortOverlappingPairs( btDispatcher dispatcher );


		internal override object removeOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, btDispatcher dispatcher )
		{
			if( !hasDeferredRemoval() )
			{
				btBroadphasePair findPair = BulletGlobals.BroadphasePairPool.Get();
				findPair.Initialize( proxy0, proxy1 );

				int findIndex = m_overlappingPairArray.LinearSearch( findPair );
				if( findIndex < m_overlappingPairArray.Count )
				{
					btHashedOverlappingPairCache.gOverlappingPairs--;
					btBroadphasePair pair = m_overlappingPairArray[findIndex];
					object userData = pair.m_internalInfo1;
					cleanOverlappingPair( pair, dispatcher );
					if( m_ghostPairCallback != null )
						m_ghostPairCallback.removeOverlappingPair( proxy0, proxy1, dispatcher );

					m_overlappingPairArray.Swap( findIndex, m_overlappingPairArray.Count - 1 );
					m_overlappingPairArray.Count--;
					return userData;
				}
			}

			return 0;
		}


		internal override btBroadphasePair addOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			//don't add overlap with own
			Debug.Assert( proxy0 != proxy1 );

			if( !needsBroadphaseCollision( proxy0, proxy1 ) )
				return null;

			btBroadphasePair pair = BulletGlobals.BroadphasePairPool.Get();
			pair.Initialize( proxy0, proxy1 );
			m_overlappingPairArray.Add( pair );
			btHashedOverlappingPairCache.gOverlappingPairs++;
			btHashedOverlappingPairCache.gAddedPairs++;

			if( m_ghostPairCallback != null )
				m_ghostPairCallback.addOverlappingPair( proxy0, proxy1 );
			return pair;

		}

		///this findPair becomes really slow. Either sort the list to speedup the query, or
		///use a different solution. It is mainly used for Removing overlapping pairs. Removal could be delayed.
		///we could keep a linked list in each proxy, and store pair in one of the proxies (with lowest memory address)
		///Also we can use a 2D bitmap, which can be useful for a future GPU implementation
		internal override btBroadphasePair findPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			if( !needsBroadphaseCollision( proxy0, proxy1 ) )
				return null;

			btBroadphasePair tmpPair = BulletGlobals.BroadphasePairPool.Get();
			tmpPair.Initialize( proxy0, proxy1 );
			int findIndex = m_overlappingPairArray.LinearSearch( tmpPair );
			BulletGlobals.BroadphasePairPool.Free( tmpPair );
			if( findIndex < m_overlappingPairArray.Count )
			{
				//Debug.Assert(it != m_overlappingPairSet.end());
				btBroadphasePair pair = m_overlappingPairArray[findIndex];
				return pair;
			}
			return null;
		}



		internal override void processAllOverlappingPairs( btOverlapCallback callback, btDispatcher dispatcher )
		{

			int i;

			for( i = 0; i < m_overlappingPairArray.Count; )
			{

				btBroadphasePair pair = m_overlappingPairArray[i];
				if( callback.processOverlap( pair ) )
				{
					cleanOverlappingPair( pair, dispatcher );
					pair.m_pProxy0 = null;
					pair.m_pProxy1 = null;
					m_overlappingPairArray.Swap( i, m_overlappingPairArray.Count - 1 );
					m_overlappingPairArray.Count--;
					btHashedOverlappingPairCache.gOverlappingPairs--;
				}
				else
				{
					i++;
				}
			}
		}




		btSortedOverlappingPairCache()
		{
			m_blockedForChanges = ( false );
			m_hasDeferredRemoval = ( true );
			m_overlapFilterCallback = ( null );
			m_ghostPairCallback = ( null );
			int initialAllocatedSize = 2;
			m_overlappingPairArray.Capacity = ( initialAllocatedSize );
		}

		internal override void cleanOverlappingPair( btBroadphasePair pair, btDispatcher dispatcher )
		{
			if( pair.m_algorithm != null )
			{
				{
					dispatcher.freeCollisionAlgorithm( pair.m_algorithm );
					pair.m_algorithm = null;
					btHashedOverlappingPairCache.gRemovePairs--;
				}
			}
		}

		struct CleanPairCallback : btOverlapCallback
		{
			btBroadphaseProxy m_cleanProxy;
			btOverlappingPairCache m_pairCache;
			btDispatcher m_dispatcher;

			internal CleanPairCallback( btBroadphaseProxy cleanProxy, btOverlappingPairCache pairCache, btDispatcher dispatcher )
			{
				m_cleanProxy = ( cleanProxy );
				m_pairCache = ( pairCache );
				m_dispatcher = ( dispatcher );
			}
			public bool processOverlap( btBroadphasePair pair )
			{
				if( ( pair.m_pProxy0 == m_cleanProxy ) ||
					( pair.m_pProxy1 == m_cleanProxy ) )
				{
					m_pairCache.cleanOverlappingPair( pair, m_dispatcher );
				}
				return false;
			}

		};

		internal override void cleanProxyFromPairs( btBroadphaseProxy proxy, btDispatcher dispatcher )
		{
			CleanPairCallback cleanPairs = new CleanPairCallback( proxy, this, dispatcher );
			processAllOverlappingPairs( cleanPairs, dispatcher );
		}


		class RemovePairCallback : btOverlapCallback
		{
			btBroadphaseProxy m_obsoleteProxy;

			internal
			RemovePairCallback( btBroadphaseProxy obsoleteProxy )
			{
				m_obsoleteProxy = ( obsoleteProxy );
			}
			public bool processOverlap( btBroadphasePair pair )
			{
				return ( ( pair.m_pProxy0 == m_obsoleteProxy ) ||
					( pair.m_pProxy1 == m_obsoleteProxy ) );
			}

		};
		internal override void removeOverlappingPairsContainingProxy( btBroadphaseProxy proxy, btDispatcher dispatcher )
		{
			RemovePairCallback removeCallback = new RemovePairCallback( proxy );
			processAllOverlappingPairs( removeCallback, dispatcher );
		}

		internal override void sortOverlappingPairs( btDispatcher dispatcher )
		{
			//should already be sorted
		}
	};



	///btNullPairCache skips add/removal of overlapping pairs. Userful for benchmarking and unit testing.
	internal class btNullPairCache : btOverlappingPairCache
	{

		btBroadphasePairArray m_overlappingPairArray;

		internal override btBroadphasePair[] getOverlappingPairArrayPtr()
		{
			return m_overlappingPairArray.InternalArray;
		}
		internal override btBroadphasePairArray getOverlappingPairArray()
		{
			return m_overlappingPairArray;
		}

		internal override void cleanOverlappingPair( btBroadphasePair pair, btDispatcher dispatcher )
		{

		}

		internal override int getNumOverlappingPairs()
		{
			return 0;
		}

		internal override void cleanProxyFromPairs( btBroadphaseProxy proxy, btDispatcher dispatcher )
		{

		}

		internal override void setOverlapFilterCallback( btOverlapFilterCallback callback )
		{
		}

		internal override void processAllOverlappingPairs( btOverlapCallback callback, btDispatcher dispatcher )
		{
		}

		internal override btBroadphasePair findPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			return null;
		}

		internal override bool hasDeferredRemoval()
		{
			return true;
		}

		internal override void setInternalGhostPairCallback( btOverlappingPairCallback ghostPairCallback )
		{

		}

		internal override btBroadphasePair addOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			return null;
		}

		internal override object removeOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, btDispatcher dispatcher )
		{
			return null;
		}

		internal override void removeOverlappingPairsContainingProxy( btBroadphaseProxy proxy0, btDispatcher dispatcher )
		{
		}

		internal override void sortOverlappingPairs( btDispatcher dispatcher )
		{
			//(void)dispatcher;
		}



	};

}