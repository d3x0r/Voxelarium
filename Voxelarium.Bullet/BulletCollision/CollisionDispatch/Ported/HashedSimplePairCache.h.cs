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
using System.Diagnostics;

namespace Bullet.Collision.Dispatch
{


	internal class btSimplePair
	{
		internal btSimplePair( int indexA, int indexB )
		{
			m_indexA = ( indexA );
			m_indexB = ( indexB );
			m_userPointer = null;
		}

		internal int m_indexA;
		internal int m_indexB;
		internal object m_userPointer;
	};

	internal class btSimplePairArray : btList<btSimplePair> { }






	class btHashedSimplePairCache
	{
		const int BT_SIMPLE_NULL_PAIR = -1;
		btSimplePairArray m_overlappingPairArray = new btSimplePairArray();



		btList<int> m_hashTable = new btList<int>();
		btList<int> m_next = new btList<int>();


		// Add a pair and return the new pair. If the pair already exists,
		// no new pair is created and the old one is returned.
		internal virtual btSimplePair addOverlappingPair( int indexA, int indexB )
		{
			gAddedSimplePairs++;

			return internalAddPair( indexA, indexB );
		}



		internal btSimplePair[] getOverlappingPairArrayPtr()
		{
			return m_overlappingPairArray.InternalArray;
		}

		internal btSimplePairArray getOverlappingPairArray()
		{
			return m_overlappingPairArray;
		}


		//btSimplePair findPair( int indexA, int indexB );

		int GetCount() { return m_overlappingPairArray.Count; }

		int getNumOverlappingPairs()
		{
			return m_overlappingPairArray.Count;
		}
		
		//btSimplePair internalAddPair( int indexA, int indexB );

		//void growTables();

		public bool equalsPair( btSimplePair pair, int indexA, int indexB )
		{
			return pair.m_indexA == indexA && pair.m_indexB == indexB;
		}



		public uint getHash( uint indexA, uint indexB )
		{
			int key = (int)( ( (uint)indexA ) | ( ( (uint)indexB ) << 16 ) );
			// Thomas Wang's hash

			key += ~( key << 15 );
			key ^= ( key >> 10 );
			key += ( key << 3 );
			key ^= ( key >> 6 );
			key += ~( key << 11 );
			key ^= ( key >> 16 );
			return (uint)( key );
		}





		public btSimplePair internalFindPair( int proxyIdA, int proxyIdB, int hash )
		{

			int index = m_hashTable[hash];

			while( index != BT_SIMPLE_NULL_PAIR && equalsPair( m_overlappingPairArray[index], proxyIdA, proxyIdB ) == false )
			{
				index = m_next[index];
			}

			if( index == BT_SIMPLE_NULL_PAIR )
			{
				return null;
			}

			Debug.Assert( index < m_overlappingPairArray.Count );

			return m_overlappingPairArray[index];
		}


		static int gOverlappingSimplePairs = 0;
		static int gRemoveSimplePairs = 0;
		static int gAddedSimplePairs = 0;
		static int gFindSimplePairs = 0;




		internal btHashedSimplePairCache()
		{
			int initialAllocatedSize = 2;
			m_overlappingPairArray.Capacity = ( initialAllocatedSize );
			growTables();
		}



		internal void removeAllPairs()
		{
			m_overlappingPairArray.Clear();
			m_hashTable.Clear();
			m_next.Clear();

			int initialAllocatedSize = 2;
			m_overlappingPairArray.Capacity = initialAllocatedSize;
			growTables();
		}



		internal btSimplePair findPair( int indexA, int indexB )
		{
			gFindSimplePairs++;


			/*if (indexA > indexB) 
				btSwap(indexA, indexB);*/

			int hash = (int)( getHash( (uint)( indexA ), (uint)( indexB ) ) & ( m_overlappingPairArray.Capacity - 1 ) );

			if( hash >= m_hashTable.Count )
			{
				return null;
			}

			int index = m_hashTable[hash];
			while( index != BT_SIMPLE_NULL_PAIR && equalsPair( m_overlappingPairArray[index], indexA, indexB ) == false )
			{
				index = m_next[index];
			}

			if( index == BT_SIMPLE_NULL_PAIR )
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

				m_hashTable.Capacity = newCapacity;
				m_next.Capacity = ( newCapacity );


				int i;

				for( i = 0; i < newCapacity; ++i )
				{
					m_hashTable[i] = BT_SIMPLE_NULL_PAIR;
				}
				for( i = 0; i < newCapacity; ++i )
				{
					m_next[i] = BT_SIMPLE_NULL_PAIR;
				}

				for( i = 0; i < curHashtableSize; i++ )
				{

					btSimplePair  pair = m_overlappingPairArray[i];
					int indexA = pair.m_indexA;
					int indexB = pair.m_indexB;

					int hashValue = (int)( getHash( (uint)( indexA ), (uint)( indexB ) ) & ( m_overlappingPairArray.Capacity - 1 ) );    // New hash value with new mask
					m_next[i] = m_hashTable[hashValue];
					m_hashTable[hashValue] = i;
				}


			}
		}

		btSimplePair internalAddPair( int indexA, int indexB )
		{

			int hash = (int)( getHash( (uint)( indexA ), (uint)( indexB ) ) 
					& ( m_overlappingPairArray.Capacity - 1 ) ); // New hash value with new mask


			btSimplePair pair = internalFindPair( indexA, indexB, hash );
			if( pair != null )
			{
				return pair;
			}

			int count = m_overlappingPairArray.Count;
			int oldCapacity = m_overlappingPairArray.Capacity;
			
			int newCapacity = m_overlappingPairArray.Capacity;

			if( oldCapacity < newCapacity )
			{
				growTables();
				//hash with new capacity
				hash = (int)( getHash( (uint)( indexA ), (uint)( indexB ) ) & ( m_overlappingPairArray.Capacity - 1 ) );
			}

			pair = new btSimplePair( indexA, indexB );

			pair.m_userPointer = 0;

			m_next[count] = m_hashTable[hash];
			m_hashTable[hash] = count;

			return pair;
		}



		internal object removeOverlappingPair( int indexA, int indexB )
		{
			gRemoveSimplePairs++;


			/*if (indexA > indexB) 
				btSwap(indexA, indexB);*/

			int hash = (int)( getHash( (uint)( indexA ), (uint)( indexB ) ) & ( m_overlappingPairArray.Capacity - 1 ) );

			btSimplePair pair = internalFindPair( indexA, indexB, hash );
			if( pair == null )
			{
				return 0;
			}


			object userData = pair.m_userPointer;


			int pairIndex = m_overlappingPairArray.IndexOf( pair );
			Debug.Assert( pairIndex < m_overlappingPairArray.Count );

			// Remove the pair from the hash table.
			int index = m_hashTable[hash];
			Debug.Assert( index != BT_SIMPLE_NULL_PAIR );

			int previous = BT_SIMPLE_NULL_PAIR;
			while( index != pairIndex )
			{
				previous = index;
				index = m_next[index];
			}

			if( previous != BT_SIMPLE_NULL_PAIR )
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

			// If the removed pair is the last pair, we are done.
			if( lastPairIndex == pairIndex )
			{
				m_overlappingPairArray.Count--;
				return userData;
			}

			// Remove the last pair from the hash table.
			btSimplePair last = m_overlappingPairArray[lastPairIndex];
			/* missing swap here too, Nat. */
			int lastHash = (int)( getHash( (uint)( last.m_indexA ), (uint)( last.m_indexB ) ) & ( m_overlappingPairArray.Capacity - 1 ) );

			index = m_hashTable[lastHash];
			Debug.Assert( index != BT_SIMPLE_NULL_PAIR );

			previous = BT_SIMPLE_NULL_PAIR;
			while( index != lastPairIndex )
			{
				previous = index;
				index = m_next[index];
			}

			if( previous != BT_SIMPLE_NULL_PAIR )
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







	};




}
