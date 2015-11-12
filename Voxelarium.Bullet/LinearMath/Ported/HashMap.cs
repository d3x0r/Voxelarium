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

using System;
using System.Diagnostics;
using Bullet.Types;

namespace Bullet.LinearMath
{

	public interface btHashInterface
	{
		uint getHash();
	}

	///very basic hashable string implementation, compatible with btHashMap
	public struct btHashString : btHashInterface
	{
		static uint InitialFNV = 2166136261u;
		static uint FNVMultiple = 16777619u;

		public string m_string;
		public uint m_hash;

		public uint getHash()
		{
			return m_hash;
		}

		public btHashString( string name )
		{
			m_string = name;
			/* magic numbers from http://www.isthe.com/chongo/tech/comp/fnv/ */

			/* Fowler / Noll / Vo (FNV) Hash */
			uint hash = InitialFNV;

			for( int i = 0; i < m_string.Length; i++ )
			{
				hash = hash ^ ( m_string[i] );       /* xor  the low 8 bits */
				hash = hash * FNVMultiple;  /* multiply by the magic number */
			}
			m_hash = hash;
		}

		public static int portableStringCompare( string src, string dst )
		{
			return string.Compare( src, dst );
		}

		bool Equals( btHashString other )
		{
			return ( m_string == other.m_string ||
				( 0 == string.Compare( m_string, other.m_string ) ) );
		}

	};



	class btHashInt : btHashInterface
	{

		uint m_uid;
		public btHashInt( uint uid )
		{
			m_uid = ( uid );
		}

		public uint getUid1()
		{
			return m_uid;
		}

		public void setUid1( uint uid )
		{
			m_uid = uid;
		}

		public bool Equals( btHashInt other )
		{
			return getUid1() == other.getUid1();
		}
		//to our success
		public uint getHash()
		{
			uint key = m_uid;
			// Thomas Wang's hash
			key += ~( key << 15 ); key ^= ( key >> 10 ); key += ( key << 3 ); key ^= ( key >> 6 ); key += ~( key << 11 ); key ^= ( key >> 16 );
			return key;
		}
	};



	class btHashPtr : btHashInterface
	{

		//union
		//{
		object m_pointer;
		//int m_hashValues[2];
		//};

		public btHashPtr( object ptr )
		{
			m_pointer = ptr;
		}

		public object getPointer()
		{
			return m_pointer;
		}

		public bool Equals( btHashPtr other )
		{
			return getPointer() == other.getPointer();
		}

		//to our success
		public uint getHash()
		{
			uint key = (uint)( (UIntPtr)m_pointer );
			// Thomas Wang's hash
			key += ~( key << 15 ); key ^= ( key >> 10 ); key += ( key << 3 ); key ^= ( key >> 6 ); key += ~( key << 11 ); key ^= ( key >> 16 );
			return (uint)key;
		}

	};


	public class btHashKeyPtr<Value> : btHashInterface
	{
		int m_uid;

		public btHashKeyPtr( int uid )
		{
			m_uid = uid;
		}

		public int getUid1()
		{
			return m_uid;
		}

		public bool Equals( btHashKeyPtr<Value> other )
		{
			return getUid1() == other.getUid1();
		}

		//to our success
		public uint getHash()
		{
			int key = m_uid;
			// Thomas Wang's hash
			key += ~( key << 15 ); key ^= ( key >> 10 ); key += ( key << 3 ); key ^= ( key >> 6 ); key += ~( key << 11 ); key ^= ( key >> 16 );
			return (uint)key;
		}


	};


	class btHashKey<Value> : btHashInterface
	{
		int m_uid;


		public btHashKey( int uid )
		{
			m_uid = uid;
		}

		public int getUid1()
		{
			return m_uid;
		}

		public bool Equals( btHashKey<Value> other )
		{
			return getUid1() == other.getUid1();
		}
		//to our success
		public uint getHash()
		{
			uint key = (uint)m_uid;
			// Thomas Wang's hash
			key += ~( key << 15 ); key ^= ( key >> 10 ); key += ( key << 3 ); key ^= ( key >> 6 ); key += ~( key << 11 ); key ^= ( key >> 16 );
			return key;
		}
	};


	///The btHashMap template class implements a generic and lightweight hashmap.
	///A basic sample of how to use btHashMap is located in Demos\BasicDemo\main.cpp
	class btHashMap<Key, Value> where Key : btHashInterface
	{
		const int BT_HASH_NULL = -1;

		protected btList<int> m_hashTable = new btList<int>();
		protected btList<int> m_next = new btList<int>();

		protected btList<Value> m_valueArray = new btList<Value>();
		protected btList<Key> m_keyArray = new btList<Key>();

		protected void growTables( Key key )
		{
			int newCapacity = m_valueArray.Capacity;

			if( m_hashTable.Count < newCapacity )
			{
				//grow hashtable and next table
				int curHashtableSize = m_hashTable.Count;

				m_hashTable.Count = m_hashTable.Capacity = newCapacity;
				m_next.Count = m_next.Capacity = ( newCapacity );

				int[] arr_hashTable = m_hashTable.InternalArray;
				int[] arr_next = m_next.InternalArray;
				int i;

				for( i = 0; i < newCapacity; ++i )
				{
					arr_hashTable[i] = BT_HASH_NULL;
				}
				for( i = 0; i < newCapacity; ++i )
				{
					arr_next[i] = BT_HASH_NULL;
				}

				for( i = 0; i < curHashtableSize; i++ )
				{
					//string alue& value = m_valueArray[i];
					//string ey& key = m_keyArray[i];

					uint hashValue = m_keyArray[i].getHash() & (uint)( m_valueArray.Capacity - 1 );  // New hash value with new mask
					m_next[i] = arr_hashTable[hashValue];
					arr_hashTable[hashValue] = i;
				}


			}
		}


		public void insert( ref Key key, ref Value value )
		{
			uint hash = key.getHash() & (uint)( m_valueArray.Capacity - 1 );

			//replace value if the key is already there
			int index = findIndex( ref key );
			if( index != BT_HASH_NULL )
			{
				m_valueArray[index] = value;
				return;
			}

			int count = m_valueArray.Count;
			int oldCapacity = m_valueArray.Capacity;
			m_valueArray.Add( value );
			m_keyArray.Add( key );

			int newCapacity = m_valueArray.Capacity;
			if( oldCapacity < newCapacity )
			{
				growTables( key );
				//hash with new capacity
				hash = key.getHash() & (uint)( m_valueArray.Capacity - 1 );
			}
			m_next[count] = m_hashTable[(int)hash];
			m_hashTable[(int)hash] = count;
		}

		public void remove( ref Key key )
		{

			uint hash = key.getHash() & (uint)( m_valueArray.Capacity - 1 );

			int pairIndex = findIndex( ref key );

			if( pairIndex == BT_HASH_NULL )
			{
				return;
			}

			// Remove the pair from the hash table.
			int index = m_hashTable[(int)hash];
			Debug.Assert( index != BT_HASH_NULL );

			int previous = BT_HASH_NULL;
			while( index != pairIndex )
			{
				previous = index;
				index = m_next[index];
			}

			if( previous != BT_HASH_NULL )
			{
				Debug.Assert( m_next[previous] == pairIndex );
				m_next[previous] = m_next[pairIndex];
			}
			else
			{
				m_hashTable[(int)hash] = m_next[pairIndex];
			}

			// We now move the last pair into spot of the
			// pair being removed. We need to fix the hash
			// table indices to support the move.

			int lastPairIndex = m_valueArray.Count - 1;

			// If the removed pair is the last pair, we are done.
			if( lastPairIndex == pairIndex )
			{
				m_valueArray.RemoveAt( lastPairIndex );
				m_keyArray.RemoveAt( lastPairIndex );
				return;
			}

			// Remove the last pair from the hash table.
			uint lastHash = m_keyArray[lastPairIndex].getHash() & (uint)( m_valueArray.Capacity - 1 );

			index = m_hashTable[(int)lastHash];
			Debug.Assert( index != BT_HASH_NULL );

			previous = BT_HASH_NULL;
			while( index != lastPairIndex )
			{
				previous = index;
				index = m_next[index];
			}

			if( previous != BT_HASH_NULL )
			{
				Debug.Assert( m_next[previous] == lastPairIndex );
				m_next[previous] = m_next[lastPairIndex];
			}
			else
			{
				m_hashTable[(int)lastHash] = m_next[lastPairIndex];
			}

			// Copy the last pair into the remove pair's spot.
			m_valueArray[pairIndex] = m_valueArray[lastPairIndex];
			m_keyArray[pairIndex] = m_keyArray[lastPairIndex];

			// Insert the last pair into the hash table
			m_next[pairIndex] = m_hashTable[(int)lastHash];
			m_hashTable[(int)lastHash] = pairIndex;

			m_valueArray.RemoveAt( lastPairIndex );
			m_keyArray.RemoveAt( lastPairIndex );

		}


		public int size()
		{
			return m_valueArray.Count;
		}

		public Value getAtIndex( int index )
		{
			Debug.Assert( index < m_valueArray.Count );

			return m_valueArray[index];
		}

		public Value this[Key key]
		{
			get
			{
				return find( ref key );
			}
		}

		public Value find( ref Key key )
		{
			int index = findIndex( ref key );
			if( index == BT_HASH_NULL )
			{
				return default( Value );
			}
			return m_valueArray[index];
		}


		public int findIndex( ref Key key )
		{
			uint hash = key.getHash() & (uint)( m_valueArray.Capacity - 1 );

			if( hash >= (uint)m_hashTable.Count )
			{
				return BT_HASH_NULL;
			}

			int index = m_hashTable[(int)hash];
			while( ( index != BT_HASH_NULL ) && key.Equals( m_keyArray[index] ) == false )
			{
				index = m_next[index];
			}
			return index;
		}

		public void clear()
		{
			m_hashTable.Clear();
			m_next.Clear();
			m_valueArray.Clear();
			m_keyArray.Clear();
		}

	};

}
