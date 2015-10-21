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

using Bullet.Types;

namespace Bullet.Collision.BroadPhase
{

	public class btBroadphasePairArray : btList<btBroadphasePair>
	{
	}

	public interface btOverlapCallback
	{
		//return true for deletion of the pair
		bool processOverlap( ref btBroadphasePair pair );

	};

	public interface btOverlapFilterCallback
	{
		// return true when pairs need collision
		bool needBroadphaseCollision( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );
	};


	///The btOverlappingPairCache provides an interface for overlapping pair management (add, remove, storage), used by the btBroadphaseInterface broadphases.
	///The btHashedOverlappingPairCache and btSortedOverlappingPairCache classes are two implementations.
	public interface btOverlappingPairCache : btOverlappingPairCallback
	{

		btBroadphasePair[] getOverlappingPairArrayPtr();

		btBroadphasePairArray getOverlappingPairArray();

		void cleanOverlappingPair( ref btBroadphasePair pair, btDispatcher dispatcher );

		int getNumOverlappingPairs();

		void cleanProxyFromPairs( btBroadphaseProxy proxy, btDispatcher dispatcher );

		void setOverlapFilterCallback( btOverlapFilterCallback callback );

		void processAllOverlappingPairs( btOverlapCallback callback, btDispatcher dispatcher );

		btBroadphasePair findPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );

		bool hasDeferredRemoval();

		void setInternalGhostPairCallback( btOverlappingPairCallback ghostPairCallback );

		void sortOverlappingPairs( btDispatcher dispatcher );


	};

	/// Hash-space based Pair Cache, thanks to Erin Catto, Box2D, http://www.box2d.org, and Pierre Terdiman, Codercorner, http://codercorner.com
	class btHashedOverlappingPairCache : btOverlappingPairCache
	{
		btBroadphasePairArray m_overlappingPairArray;
		btOverlapFilterCallback m_overlapFilterCallback;


		protected List<int> m_hashTable;
		protected List<int> m_next;
		protected btOverlappingPairCallback* m_ghostPairCallback;


		public void removeOverlappingPairsContainingProxy( btBroadphaseProxy proxy, btDispatcher dispatcher );

		public virtual object removeOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, btDispatcher dispatcher );

		public bool needsBroadphaseCollision( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			if( m_overlapFilterCallback )
				return m_overlapFilterCallback.needBroadphaseCollision( proxy0, proxy1 );

			bool collides = ( proxy0.m_collisionFilterGroup & proxy1.m_collisionFilterMask ) != 0;
			collides = collides && ( proxy1.m_collisionFilterGroup & proxy0.m_collisionFilterMask );

			return collides;
		}

		// Add a pair and return the new pair. If the pair already exists,
		// no new pair is created and the old one is returned.
		public virtual btBroadphasePair* addOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			btHashedOverlappingPairCache.gAddedPairs++;

			if( !needsBroadphaseCollision( proxy0, proxy1 ) )
				return 0;

			return internalAddPair( proxy0, proxy1 );
		}



		public void cleanProxyFromPairs( btBroadphaseProxy proxy, btDispatcher* dispatcher );


		public virtual void processAllOverlappingPairs( btOverlapCallback*, btDispatcher* dispatcher );

		public virtual btBroadphasePair[] getOverlappingPairArrayPtr()
		{
			return m_overlappingPairArray.InternalArray;
		}


	btBroadphasePairArray&	getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	btBroadphasePairArray&	getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

void cleanOverlappingPair( btBroadphasePair& pair, btDispatcher* dispatcher );



btBroadphasePair* findPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );

int GetCount() string  return m_overlappingPairArray.Count; }
//	btBroadphasePair* GetPairs() { return m_pairs; }

	btOverlapFilterCallback* getOverlapFilterCallback()
{
	return m_overlapFilterCallback;
}

void setOverlapFilterCallback( btOverlapFilterCallback* callback )
{
	m_overlapFilterCallback = callback;
}

int getNumOverlappingPairs()
	{
		return m_overlappingPairArray.Count;
	}
private:
	
//	btBroadphasePair* internalAddPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );

//void growTables();

public bool equalsPair( btBroadphasePair& pair, int proxyId1, int proxyId2 )
{
	return pair.m_pProxy0.getUid() == proxyId1 && pair.m_pProxy1.getUid() == proxyId2;
}


/*
// Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
// This assumes proxyId1 and proxyId2 are 16-bit.
public int getHash(int proxyId1, int proxyId2)
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



public uint getHash( uint proxyId1, uint proxyId2 )
{
	int key = (int)( ( proxyId1 ) | ( ( proxyId2 ) << 16 ) );
	// Thomas Wang's hash

	key += ~( key << 15 );
	key ^= ( key >> 10 );
	key += ( key << 3 );
	key ^= ( key >> 6 );
	key += ~( key << 11 );
	key ^= ( key >> 16 );
	return (uint)( key );
}





public btBroadphasePair* internalFindPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, int hash )
{
	int proxyId1 = proxy0.getUid();
	int proxyId2 = proxy1.getUid();
#if 0 // wrong, 'equalsPair' use unsorted uids, copy-past devil striked again. Nat.
		if (proxyId1 > proxyId2) 
			btSwap(proxyId1, proxyId2);
#endif

	int index = m_hashTable[hash];

	while( index != BT_NULL_PAIR && equalsPair( m_overlappingPairArray[index], proxyId1, proxyId2 ) == false )
	{
		index = m_next[index];
	}

	if( index == BT_NULL_PAIR )
	{
		return NULL;
	}

	Debug.Assert( index < m_overlappingPairArray.Count );

	return &m_overlappingPairArray[index];
}

virtual bool hasDeferredRemoval()
{
	return false;
}

virtual void setInternalGhostPairCallback( btOverlappingPairCallback* ghostPairCallback )
{
	m_ghostPairCallback = ghostPairCallback;
}

virtual void sortOverlappingPairs( btDispatcher* dispatcher );
	

	
};




///btSortedOverlappingPairCache maintains the objects with overlapping AABB
///Typically managed by the Broadphase, Axis3Sweep or btSimpleBroadphase
class btSortedOverlappingPairCache : btOverlappingPairCache
{
	protected:
		//avoid brute-force finding all the time
		btBroadphasePairArray m_overlappingPairArray;

//during the dispatch, check that user doesn't destroy/create proxy
bool m_blockedForChanges;

///by default, do the removal during the pair traversal
bool m_hasDeferredRemoval;

//if set, use the callback instead of the built in filter in needBroadphaseCollision
btOverlapFilterCallback* m_overlapFilterCallback;

btOverlappingPairCallback* m_ghostPairCallback;

public:

		btSortedOverlappingPairCache();
virtual ~btSortedOverlappingPairCache();

virtual void processAllOverlappingPairs( btOverlapCallback*, btDispatcher* dispatcher );

object removeOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, btDispatcher* dispatcher );

void cleanOverlappingPair( btBroadphasePair& pair, btDispatcher* dispatcher );

btBroadphasePair* addOverlappingPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );

btBroadphasePair* findPair( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 );


void cleanProxyFromPairs( btBroadphaseProxy proxy, btDispatcher* dispatcher );

void removeOverlappingPairsContainingProxy( btBroadphaseProxy proxy, btDispatcher* dispatcher );


inline bool needsBroadphaseCollision( btBroadphaseProxy proxy0, btBroadphaseProxy proxy1 )
		{
			if (m_overlapFilterCallback)
				return m_overlapFilterCallback.needBroadphaseCollision( proxy0, proxy1);

bool collides = ( proxy0.m_collisionFilterGroup & proxy1.m_collisionFilterMask ) != 0;
collides = collides && (proxy1.m_collisionFilterGroup & proxy0.m_collisionFilterMask);
			
			return collides;
		}
		
		btBroadphasePairArray&	getOverlappingPairArray()
{
	return m_overlappingPairArray;
}

btBroadphasePairArray&	getOverlappingPairArray()
		{
			return m_overlappingPairArray;
		}

		


		btBroadphasePair[] getOverlappingPairArrayPtr()
{
	return m_overlappingPairArray;
}

btBroadphasePair* getOverlappingPairArrayPtr()
		{
			return m_overlappingPairArray;
		}

		int getNumOverlappingPairs()
		{
			return m_overlappingPairArray.Count;
		}
		
		btOverlapFilterCallback* getOverlapFilterCallback()
{
	return m_overlapFilterCallback;
}

void setOverlapFilterCallback( btOverlapFilterCallback* callback )
{
	m_overlapFilterCallback = callback;
}

virtual bool hasDeferredRemoval()
{
	return m_hasDeferredRemoval;
}

virtual void setInternalGhostPairCallback( btOverlappingPairCallback* ghostPairCallback )
{
	m_ghostPairCallback = ghostPairCallback;
}

virtual void sortOverlappingPairs( btDispatcher* dispatcher );
		

};



///btNullPairCache skips add/removal of overlapping pairs. Userful for benchmarking and unit testing.
internal class btNullPairCache : btOverlappingPairCache
{

	btBroadphasePairArray m_overlappingPairArray;

	public:

	virtual btBroadphasePair* getOverlappingPairArrayPtr()
	{
		return m_overlappingPairArray;
	}
	btBroadphasePair* getOverlappingPairArrayPtr()
		{
		return m_overlappingPairArray;
	}
btBroadphasePairArray&	getOverlappingPairArray()
{
	return m_overlappingPairArray;
}

virtual void cleanOverlappingPair( btBroadphasePair& /*pair*/, btDispatcher* /*dispatcher*/)
{

}

virtual int getNumOverlappingPairs()
	{
		return 0;
	}

	virtual void cleanProxyFromPairs( btBroadphaseProxy /*proxy*/, btDispatcher* /*dispatcher*/)
{

}

virtual void setOverlapFilterCallback( btOverlapFilterCallback* /*callback*/)
{
}

virtual void processAllOverlappingPairs( btOverlapCallback*, btDispatcher* /*dispatcher*/)
{
}

virtual btBroadphasePair* findPair( btBroadphaseProxy /*proxy0*/, btBroadphaseProxy /*proxy1*/)
{
	return 0;
}

virtual bool hasDeferredRemoval()
{
	return true;
}

virtual void setInternalGhostPairCallback( btOverlappingPairCallback* /* ghostPairCallback */)
{

}

virtual btBroadphasePair* addOverlappingPair( btBroadphaseProxy /*proxy0*/, btBroadphaseProxy /*proxy1*/)
{
	return 0;
}

virtual object removeOverlappingPair( btBroadphaseProxy /*proxy0*/, btBroadphaseProxy /*proxy1*/, btDispatcher* /*dispatcher*/)
{
	return 0;
}

virtual void removeOverlappingPairsContainingProxy( btBroadphaseProxy /*proxy0*/, btDispatcher* /*dispatcher*/)
{
}

virtual void sortOverlappingPairs( btDispatcher* dispatcher )
{
	(void)dispatcher;
}


};



}