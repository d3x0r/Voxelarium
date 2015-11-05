#define USE_PATH_COMPRESSION

#define STATIC_SIMULATION_ISLAND_OPTIMIZATION

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
using System.Collections.Generic;
using Bullet.Types;
///see for discussion of static island optimizations by Vroonsh here: http://code.google.com/p/bullet/issues/detail?id=406
namespace Bullet.Collision.Dispatch
{
	///UnionFind calculates connected subsets
	// Implements weighted Quick Union with path compression
	// optimization: could use short ints instead of ints (halving memory, would limit the number of rigid bodies to 64k, sounds reasonable)
	public class btUnionFind
	{
		internal struct btElement
		{
			public int m_id;
			public int m_sz;
		};

		btList<btElement> m_elements = new btList<btElement>();

		public int getNumElements()
		{
			return m_elements.Count;
		}
		public bool isRoot( int x )
		{
			return ( x == m_elements[x].m_id );
		}

		btElement getElement( int index )
		{
			return m_elements[index];
		}
		internal int getElementId( int index )
		{
			return m_elements.InternalArray[index].m_id;
		}
		internal int getElementSz( int index )
		{
			return m_elements.InternalArray[index].m_sz;
		}

		internal void setElementSz( int index, int sz )
		{
			m_elements.InternalArray[index].m_sz = sz;
		}

		bool find( int p, int q )
		{
			return ( find( p ) == find( q ) );
		}

		public void unite( int p, int q )
		{
			btElement[] _elements = m_elements.InternalArray;
			int i = find( p ), j = find( q );
			if( i == j )
				return;

#if USE_PATH_COMPRESSION
			//weighted quick union, this keeps the 'trees' balanced, and keeps performance of unite O( log(n) )
			if( m_elements[i].m_sz < m_elements[j].m_sz )
			{
				_elements[i].m_id = j; _elements[j].m_sz += m_elements[i].m_sz;
			}
			else
			{
				_elements[j].m_id = i; _elements[i].m_sz += m_elements[j].m_sz;
			}
#else
			m_elements[i].m_id = j; m_elements[j].m_sz += m_elements[i].m_sz; 
#endif //USE_PATH_COMPRESSION
		}

		public int find( int x )
		{
			//Debug.Assert(x < m_N);
			//Debug.Assert(x >= 0);
			btElement[] _elements = m_elements.InternalArray;

			while( x != _elements[x].m_id )
			{
				//not really a reason not to use path compression, and it flattens the trees/improves find performance dramatically

#if USE_PATH_COMPRESSION
				int n;
				n = m_elements[x].m_id;
				_elements[x].m_id = _elements[n].m_id;
				x = _elements[n].m_id;
#else//
				x = m_elements[x].m_id;
#endif
				//Debug.Assert(x < m_N);
				//Debug.Assert(x >= 0);

			}
			return x;
		}

		~btUnionFind()
		{
			Free();
		}

		public btUnionFind()
		{

		}

		public void allocate( int N )
		{
			m_elements.Capacity = N;
			m_elements.Count = N;
			m_elements[N - 1] = new btElement();
		}
		public void Free()
		{
			m_elements.Clear();
		}


		public void reset( int N )
		{
			allocate( N );
			btElement[] _elements = m_elements.InternalArray;

			for( int i = 0; i < N; i++ )
			{
				_elements[i].m_id = i; _elements[i].m_sz = 1;
			}
		}

		class btUnionFindElementSortPredicate : IComparer<btElement>
		{
			int IComparer<btElement>.Compare( btElement lhs, btElement rhs )
			{
				return ( lhs.m_id < rhs.m_id ) ? -1 :
					( lhs.m_id > rhs.m_id ) ? 1 : 0;
			}
		}

		///this is a special operation, destroying the content of btUnionFind.
		///it sorts the elements, based on island id, in order to make it easy to iterate over islands
		public void sortIslands()
		{

			//first store the original body index, and islandId
			int numElements = m_elements.Count;
			btElement[] _elements = m_elements.InternalArray;

			for( int i = 0; i < numElements; i++ )
			{
				_elements[i].m_id = find( i );
# if !STATIC_SIMULATION_ISLAND_OPTIMIZATION
				_elements[i].m_sz = i;
#endif //STATIC_SIMULATION_ISLAND_OPTIMIZATION
			}

			// Sort the vector using predicate and std::sort
			//std::sort(m_elements.begin(), m_elements.end(), btUnionFindElementSortPredicate);
			m_elements.Sort( new btUnionFindElementSortPredicate() );

		}
	};
}
