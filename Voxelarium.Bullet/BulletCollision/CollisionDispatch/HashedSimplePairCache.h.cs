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

#if ! BT_HASHED_SIMPLE_PAIR_CACHE_H
#define BT_HASHED_SIMPLE_PAIR_CACHE_H



#include "LinearMath/List.h"

int BT_SIMPLE_NULL_PAIR=0xffffffff;

struct btSimplePair
{
	btSimplePair(int indexA,int indexB)
		:m_indexA(indexA),
		m_indexB(indexB),
		m_userPointer(0)
	{
	}

	int m_indexA;
	int m_indexB;
	union
	{
		object	m_userPointer;
		int		m_userValue;
	};
};

typedef btList<btSimplePair>	btSimplePairArray;



extern int gOverlappingSimplePairs;
extern int gRemoveSimplePairs;
extern int gAddedSimplePairs;
extern int gFindSimplePairs;




class btHashedSimplePairCache
{
	btSimplePairArray	m_overlappingPairArray;
		

protected:
	
	List<int>	m_hashTable;
	List<int>	m_next;
	

public:
	btHashedSimplePairCache();
	virtual ~btHashedSimplePairCache();
	
	void removeAllPairs();

	virtual object	removeOverlappingPair(int indexA,int indexB);
	
	// Add a pair and return the new pair. If the pair already exists,
	// no new pair is created and the old one is returned.
	virtual btSimplePair* 	addOverlappingPair(int indexA,int indexB)
	{
		gAddedSimplePairs++;

		return internalAddPair(indexA,indexB);
	}

	
	virtual btSimplePair*	getOverlappingPairArrayPtr()
	{
		return m_overlappingPairArray;
	}

	btSimplePair*	getOverlappingPairArrayPtr()
	{
		return m_overlappingPairArray;
	}

	btSimplePairArray&	getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	btSimplePairArray&	getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	
	btSimplePair* findPair(int indexA,int indexB);

	int GetCount() string  return m_overlappingPairArray.Count; }

	int	getNumOverlappingPairs()
	{
		return m_overlappingPairArray.Count;
	}
private:
	
	btSimplePair* 	internalAddPair(int indexA, int indexB);

	void	growTables();

	public bool equalsPair(btSimplePair& pair, int indexA, int indexB)
	{	
		return pair.m_indexA == indexA && pair.m_indexB == indexB;
	}

	
	
	public	uint getHash(uint indexA, uint indexB)
	{
		int key = static_cast<int>(((uint)indexA) | (((uint)indexB) <<16));
		// Thomas Wang's hash

		key += ~(key << 15);
		key ^=  (key >> 10);
		key +=  (key << 3);
		key ^=  (key >> 6);
		key += ~(key << 11);
		key ^=  (key >> 16);
		return static_cast<uint>(key);
	}
	




	public btSimplePair* internalFindPair(int proxyIdA , int proxyIdB, int hash)
	{
		
		int index = m_hashTable[hash];
		
		while( index != BT_SIMPLE_NULL_PAIR && equalsPair(m_overlappingPairArray[index], proxyIdA, proxyIdB) == false)
		{
			index = m_next[index];
		}

		if ( index == BT_SIMPLE_NULL_PAIR )
		{
			return NULL;
		}

		Debug.Assert(index < m_overlappingPairArray.Count);

		return &m_overlappingPairArray[index];
	}

	
};




#endif //BT_HASHED_SIMPLE_PAIR_CACHE_H


