/*
 * This file is part of Voxelarium.
 *
 * Copyright 2015-2016 James Buckeyne  *** Added 11/22/2015
 *
 * Voxelarium is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Voxelarium is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//#define USE_REFS_ROTATE
//#define DEBUG_BALANCING
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

namespace Voxelarium.Core.Voxels.Types
{
	internal class SortingTree : IEnumerable
	{
		/// <summary>
		/// storage type for Sorting Tree
		/// </summary>
		internal struct SortingTreeNode
		{
			internal int key;
			internal ushort value;
			internal int lesser, greater, parent, children;
		};
		static SortingTreeNode null_node;

		SortingTreeNode[] storage;
		int root;
		int used;
#if DEBUG_BALANCING
		int left_swaps;
		int right_swaps;
		int left_shallow_swaps;
		int right_shallow_swaps;
#endif
		internal SortingTree( int Capacity )
		{
			storage = new SortingTreeNode[Capacity];
			root = -1;
			used = 0;
		}

		internal void Clear()
		{
			root = -1;
			used = 0;
#if DEBUG_BALANCING
			left_swaps = right_swaps = left_shallow_swaps = right_shallow_swaps = 0;
#endif
		}

		void DumpTree( int current, int level )
		{
			if( current == -1 )
			{
				if( root == -1 )
					return;
				Console.WriteLine( "Start Tree Dump" );
				DumpTree( root, level + 1 );
			}
			else
			{
				if( storage[current].lesser != -1 )
				{
					DumpTree( storage[current].lesser, level + 1 );
				}
				Console.WriteLine( "Cur: " + current + " Par: " + storage[current].parent 
					+ " Les: " + storage[current].lesser
					+ " Grt: " + storage[current].greater
					+  " Lev: " + level + " cld: " + storage[current].children + "  Key: " + storage[current].key + " val: " + storage[current].value );
				if( storage[current].greater != -1 )
				{
					DumpTree( storage[current].greater, level + 1 );
				}
			}

		}

		void Add( ref SortingTreeNode node, int parent_id, int K, ushort V )
		{
			node.key = K;
			node.value = V;
			node.lesser = -1;
			node.greater = -1;
			node.parent = parent_id;
			node.children = 0;
		}

#if USE_REFS_ROTATE
		void RotateLeft( ref SortingTreeNode node, int node_id, ref SortingTreeNode parent, ref SortingTreeNode greater )
		{
			if( node.parent == -1 )
			{
				root = node.greater;
				storage[root].parent = -1;
			}
			else
			{
				if( parent.lesser == node_id )
					parent.lesser = node.greater;
				else
					parent.greater = node.greater;
				greater.parent = node.parent;
			}
			node.children -= ( greater.children + 1 );
			node.parent = node.greater;
			int temp = greater.lesser;
			node.greater = temp;
			greater.lesser = node_id;
			if( temp >= 0 )
			{
				storage[temp].parent = node_id;
				greater.children -= ( storage[temp].children + 1 );
				node.children += ( storage[temp].children + 1 );
			}
			greater.children += ( node.children + 1 );
		}

		void RotateRight( ref SortingTreeNode node, int node_id, ref SortingTreeNode parent, ref SortingTreeNode lesser )
		{
			if( node.parent == -1 )
			{
				root = node.lesser;
				storage[root].parent = -1;
			}
			else
			{
				if( parent.lesser == node_id )
					parent.lesser = node.lesser;
				else
					parent.greater = node.lesser;
				lesser.parent = node.parent;
			}
			node.children -= ( lesser.children + 1 );
			node.parent = node.lesser;
			int temp = lesser.greater;
			node.lesser = temp;
			lesser.greater = node_id;  // actually oldnode.lesser
			if( temp >= 0 )
			{
				storage[temp].parent = node_id;
				lesser.children -= ( storage[temp].children + 1 );
				node.children += ( storage[temp].children + 1 );
			}
			lesser.children += ( node.children + 1 );
		}
#else
		void RotateLeft( ref SortingTreeNode node, int node_id )
		{
			if( node.parent == -1 )
			{
				root = node.greater;
				storage[root].parent = -1;
			}
			else
			{
				if( storage[node.parent].lesser == node_id )
					storage[node.parent].lesser = node.greater;
				else
					storage[node.parent].greater = node.greater;
				storage[node.greater].parent = node.parent;
			}
			node.children -= ( storage[node.greater].children + 1 );
			node.parent = node.greater;
			int temp = storage[node.greater].lesser;
			node.greater = temp;
			storage[node.parent].lesser = node_id;
			if( temp >= 0 )
			{
				storage[temp].parent = node_id;
				storage[node.parent].children -= ( storage[temp].children + 1 );
				node.children += ( storage[temp].children + 1 );
			}
			storage[node.parent].children += ( node.children + 1 );
		}

		void RotateRight( ref SortingTreeNode node, int node_id )
		{
			if( node.parent == -1 )
			{
				root = node.lesser;
				storage[root].parent = -1;
			}
			else
			{
				if( storage[node.parent].lesser == node_id )
					storage[node.parent].lesser = node.lesser;
				else
					storage[node.parent].greater = node.lesser;
				storage[node.lesser].parent = node.parent;
			}
			node.children -= ( storage[node.lesser].children + 1 );
			node.parent = node.lesser;
			int temp = storage[node.lesser].greater;
			node.lesser = temp;
			storage[node.parent].greater = node_id;  // actually oldnode.lesser
			if( temp >= 0 )
			{
				storage[temp].parent = node_id;
				storage[node.parent].children -= ( storage[temp].children + 1 );
				node.children += ( storage[temp].children + 1 );
			}
			storage[node.parent].children += ( node.children + 1 );
		}
#endif

		void Balance( ref SortingTreeNode node, int node_id )
		{
			int left_children = node.lesser == -1 ? 0 : (storage[node.lesser].children+1);
			int right_children = node.greater == -1 ? 0 : (storage[node.greater].children+1);
			int parent = node.parent;
			if( left_children == 0 )
			{
				if( right_children > 1 )
				{
#if DEBUG_BALANCING
					left_shallow_swaps++;
#endif
#if !USE_REFS_ROTATE
					RotateLeft( ref node, node_id );
#else
					if( parent != -1 )
						RotateLeft( ref node, node_id, ref storage[parent], ref storage[node.greater] );
					else
						RotateLeft( ref node, node_id, ref null_node, ref storage[node.greater] );
#endif

				}
			}
			else if( right_children == 0 )
			{
				if( left_children > 1 )
				{
#if DEBUG_BALANCING
					right_shallow_swaps++;
#endif
#if !USE_REFS_ROTATE
					RotateRight( ref node, node_id );
#else
					if( parent != -1 )
						RotateRight( ref node, node_id, ref storage[parent], ref storage[node.lesser] );
					else
						RotateRight( ref node, node_id, ref null_node, ref storage[node.lesser] );
#endif
				}
			}
			else if( right_children > ( left_children * 3 ) )
			{
#if DEBUG_BALANCING
				left_swaps++;
#endif
#if !USE_REFS_ROTATE
				RotateLeft( ref node, node_id );
#else
				if( parent == -1 )
					RotateLeft( ref node, node_id, ref null_node, ref storage[node.greater] );
				else
					RotateLeft( ref node, node_id, ref storage[parent], ref storage[node.greater] );
#endif
				Balance( ref storage[node.lesser], node.lesser );
			}
			else if( left_children > ( right_children * 3 ) )
			{
#if DEBUG_BALANCING
				right_swaps++;
#endif
#if !USE_REFS_ROTATE
				RotateRight( ref node, node_id );
#else
				if( parent == -1 )
					RotateRight( ref node, node_id, ref null_node, ref storage[node.lesser] );
				else 
					RotateRight( ref node, node_id, ref storage[parent], ref storage[node.lesser] );
#endif
				Balance( ref storage[node.greater], node.greater );
			}
		}

		void ScanAdd( ref SortingTreeNode node, int node_id, int K, ushort V )
		{
			if( K < node.key )
			{
				if( node.lesser == -1 )
				{
					node.children++;
					Add( ref storage[used], node_id, K, V );
					node.lesser = used;
					used++;
				}
				else
				{
					node.children++;
					ScanAdd( ref storage[node.lesser], node.lesser, K, V );
					Balance( ref node, node_id );
				}
			}
			else if( K >= node.key )
			{
				if( node.greater == -1 )
				{
					node.children++;
					Add( ref storage[used], node_id, K, V );
					node.greater = used;
					used++;
				}
				else
				{
					node.children++;
					ScanAdd( ref storage[node.greater], node.greater, K, V );
					Balance( ref node, node_id );
				}
			}
		}

		internal void Add( int K, ushort V )
		{
			if( root == -1 )
			{
				root = 0;
				Add( ref storage[root], -1, K, V );
				used++;
			}
			else
				ScanAdd( ref storage[root], root, K, V );
			//DumpTree( -1, -1 );
		}

		internal class SortingTreeEnumerator : IEnumerator
		{
			int current;
			SortingTree Tree;
			internal SortingTreeEnumerator( SortingTree tree )
			{
				Tree = tree;
				current = -1;
			}

			public object Current
			{
				get
				{
					if( current == -1 )
						return null;
					return Tree.storage[current].value;
				}
			}

			public bool MoveNext()
			{
				int next;
				if( current == -1 )
				{
					current = Tree.root;
					if( current == -1 )
						return false;
					while( ( next = Tree.storage[current].lesser ) != -1 )
						current = next;
					return true;
				}
				if( ( next = Tree.storage[current].greater ) != -1 )
				{
					current = next;
					while( ( next = Tree.storage[current].lesser ) != -1 )
						current = next;
					return true;
				}

				while( ( next = Tree.storage[current].parent ) != -1 )
				{
					// came from lesser, so this is next larger. 
					if( Tree.storage[next].lesser == current )
					{
						current = next;
						return true;
					}
					else
						current = next;
				}
				return false;
			}

			public void Reset()
			{
				current = -1;
			}
		}

#if DEBUG_BALANCING
		static int depth;
		void GetDepth( int node_id, int level )
		{
			if( level > depth )
				depth = level;
            if( node_id == -1 )
				{
					depth = 0;
				if( root == -1 )
					return;
					node_id = root;
				}
			if( storage[node_id].lesser != -1 )
				GetDepth( storage[node_id].lesser, level + 1 );
			if( storage[node_id].greater != -1 )
				GetDepth( storage[node_id].greater, level + 1 );
		}
#endif

		public IEnumerator GetEnumerator()
		{
#if DEBUG_BALANCING
			GetDepth( -1, 0 );
			Console.WriteLine( "depth is " + depth  + " left shallow " + left_shallow_swaps + " left " + left_swaps + " right shallow " + right_shallow_swaps + " right " + right_swaps + " total " + ( left_swaps+left_shallow_swaps+right_swaps+right_shallow_swaps) );
#endif
			//DumpTree( -1, -1 );
			return new SortingTreeEnumerator( this );
		}
	}
}
