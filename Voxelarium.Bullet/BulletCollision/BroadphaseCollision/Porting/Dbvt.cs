/*
* C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
*
* Bullet Continuous Collision Detection and Physics Library
* Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
*
* This software is provided 'as-is', without any express or implied warranty.
* In no event will the authors be held liable for any damages arising from
* the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose, 
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
*	claim that you wrote the original software. If you use this software
*	in a product, an acknowledgment in the product documentation would be
*	appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
*	misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using Bullet.LinearMath;
using System.Diagnostics;
using Bullet.Types;

namespace Bullet.Collision
{
	public class btDbvt
	{
		public interface ICollide
		{
			void Process( btDbvtNode n, btDbvtNode n2 );
			void Process( btDbvtNode n );
			void Process( btDbvtNode n, double f );
			//{
			//	Process(n);
			//}
			bool Descent( btDbvtNode n );
			//{
			//	return true;
			//}
			bool AllLeaves( btDbvtNode n );
			//{
			//	return true;
			//}
		}

		// Helpers	
		public static int Nearest( int[] i, sStkNPS[] a, double v, int l, int h )
		{
			int m = 0;
			while( l < h )
			{
				m = ( l + h ) >> 1;
				if( a[i[m]].value >= v ) l = m + 1; else h = m;
			}
			return ( h );
		}

		public static int Allocate( btList<int> ifree,
			btList<sStkNPS> stock,
			sStkNPS value )
		{
			//int	i;
			//if(ifree.Count>0)
			//{
			//	i = ifree[ifree.Count - 1]; 
			//	ifree.pop_back(); 
			//	stock[i] = value; 
			//}
			//else
			//{ 
			//	i=stock.Count;
			//	stock.push_back(value); 
			//}
			//return(i); 
			return 0;
		}


		public btDbvt()
		{
			m_lkhd = -1;
		}



		public btDbvtNode Root
		{
			get { return m_root; }
			set { m_root = value; }
		}

		public void Clear()
		{
			if( m_root != null )
			{
				RecurseDeleteNode( this, m_root );
			}
			//btAlignedFree(m_free);
			//m_free = 0;
			m_lkhd = -1;
			m_stkStack.Clear();
			m_opath = 0;
		}



		public bool Empty()
		{
			return Root == null;
		}



		public void OptimizeBottomUp()
		{
			if( Root != null )
			{
				btList<btDbvtNode> leafs = new btList<btDbvtNode>( m_leaves );
				FetchLeafs( this, Root, leafs );
				BottomUp( this, leafs );
				Root = leafs[0];
			}
		}


		public void OptimizeTopDown()
		{
			OptimizeTopDown( 128 );
		}

		public void OptimizeTopDown( int bu_threshold )
		{
			// threshhold defaults to 128
			if( Root != null )
			{
				btList<btDbvtNode> leafs = new btList<btDbvtNode>( m_leaves );
				FetchLeafs( this, Root, leafs );
				Root = TopDown( this, leafs, bu_threshold );
			}
		}


		public virtual void Cleanup()
		{
			Clear();
		}

		public void OptimizeIncremental( int passes )
		{
			if( passes < 0 )
			{
				passes = m_leaves;
			}

			if( Root != null && ( passes > 0 ) )
			{
				int sizeOfUnsigned = 4;
				int computedValue = ( sizeOfUnsigned * 8 - 1 );
				do
				{
					btDbvtNode node = Root;
					int bit = 0;
					while( node.IsInternal() )
					{
						node = Sort( node, m_root )._children[( m_opath >> bit ) & 1];
						bit = ( bit + 1 ) & ( sizeof( UInt32 ) * 8 - 1 );
					}
					update( node );
					++m_opath;
				} while( --passes > 0 );
			}
		}

		public btDbvtNode insert( ref btDbvtVolume box, int data )
		{
			btDbvtNode leaf = CreateNode( this, null, ref box, data );
			InsertLeaf( this, Root, leaf );
			++m_leaves;
			return leaf;
		}


		public btDbvtNode insert( ref btDbvtVolume box, Object data )
		{
			btDbvtNode leaf = CreateNode( this, null, ref box, data );
			InsertLeaf( this, Root, leaf );
			++m_leaves;
			return leaf;
		}


		public void update( btDbvtNode leaf )
		{
			update( leaf, -1 );
		}

		public void update( btDbvtNode leaf, int lookahead )
		{
			btDbvtNode root = RemoveLeaf( this, leaf );
			if( root != null )
			{
				if( lookahead >= 0 )
				{
					for( int i = 0; ( i < lookahead ) && ( root.parent != null ); ++i )
					{
						root = root.parent;
					}
				}
				else
				{
					root = Root;
				}
			}
			InsertLeaf( this, root, leaf );
		}



		public void update( btDbvtNode leaf, ref btDbvtVolume volume )
		{
			btDbvtNode root = RemoveLeaf( this, leaf );
			if( root != null )
			{
				if( m_lkhd >= 0 )
				{
					for( int i = 0; ( i < m_lkhd ) && ( root.parent != null ); ++i )
					{
						root = root.parent;
					}
				}
				else
				{
					root = Root;
				}
			}
			leaf.volume = volume;
			InsertLeaf( this, root, leaf );
		}



		public bool update( btDbvtNode leaf, ref btDbvtVolume volume, ref btVector3 velocity, double margin )
		{
			if( leaf.volume.Contain( ref volume ) )
			{
				return ( false );
			}
			volume.Expand( new btVector3( margin ) );
			volume.SignedExpand( velocity );
			update( leaf, ref volume );
			return ( true );
		}



		public bool update( btDbvtNode leaf, ref btDbvtVolume volume, ref btVector3 velocity )
		{
			if( leaf.volume.Contain( ref volume ) )
			{
				return ( false );
			}
			volume.SignedExpand( velocity );
			update( leaf, ref volume );
			return ( true );
		}



		public bool update( btDbvtNode leaf, ref btDbvtVolume volume, double margin )
		{
			if( leaf.volume.Contain( ref volume ) )
			{
				return ( false );
			}
			volume.Expand( new btVector3( margin ) );
			update( leaf, ref volume );
			return ( true );
		}



		public void remove( btDbvtNode leaf )
		{
			RemoveLeaf( this, leaf );
			DeleteNode( this, leaf );
			--m_leaves;
		}



		public static btDbvtNode Sort( btDbvtNode n, btDbvtNode r )
		{
			btDbvtNode p = n.parent;
			Debug.Assert( n.IsInternal() );
			if( p != null && ( p.id > n.id ) )
			{
				int i = IndexOf( n );
				int j = 1 - i;
				btDbvtNode s = p._children[j];
				btDbvtNode q = p.parent;
				Debug.Assert( n == p._children[i] );
				if( q != null )
				{
					q._children[IndexOf( p )] = n;
				}
				else
				{
					r = n;
				}
				s.parent = n;
				p.parent = n;
				n.parent = q;
				p._children[0] = n._children[0];
				p._children[1] = n._children[1];
				n._children[0].parent = p;
				n._children[1].parent = p;
				n._children[i] = p;
				n._children[j] = s;

				// swap id's? as well - probably not.

				swap( ref p.volume, ref n.volume );
				return ( p );
			}
			return ( n );
		}

		public static void swap( ref btDbvtVolume a, ref btDbvtVolume b )
		{
			btDbvtVolume temp = b;
			b = a;
			a = temp;
		}

		//
		// depth is defaulted to -1
		public static void FetchLeafs( btDbvt pdbvt, btDbvtNode root, btList<btDbvtNode> leafs )
		{
			FetchLeafs( pdbvt, root, leafs, -1 );
		}
		public static void FetchLeafs( btDbvt pdbvt, btDbvtNode root, btList<btDbvtNode> leafs, int depth )
		{
			if( root.IsInternal() && depth != 0 )
			{
				FetchLeafs( pdbvt, root._children[0], leafs, depth - 1 );
				FetchLeafs( pdbvt, root._children[1], leafs, depth - 1 );
				DeleteNode( pdbvt, root );
			}
			else
			{
				leafs.Add( root );
			}
		}



		public static void Split( btList<btDbvtNode> leaves, btList<btDbvtNode> left, btList<btDbvtNode> right, ref btVector3 org, ref btVector3 axis )
		{
			left.Count = ( 0 );
			right.Count = ( 0 );
			for( int i = 0, ni = leaves.Count; i < ni; ++i )
			{
				btVector3 tmp;
				leaves[i].volume.Center().Sub( ref org, out tmp );
				if( btVector3.dot( ref axis, ref tmp ) < 0 )
				{
					left.Add( leaves[i] );
				}
				else
				{
					right.Add( leaves[i] );
				}
			}
		}

		public static void GetMaxDepth( btDbvtNode node, int depth, ref int maxDepth )
		{
			if( node.IsInternal() )
			{
				GetMaxDepth( node._children[0], depth + 1, ref maxDepth );
				GetMaxDepth( node._children[1], depth + 1, ref maxDepth );
			}
			else
			{
				maxDepth = Math.Max( depth, maxDepth );
			}
		}



		public static void EnumNodes( btDbvtNode root, ICollide collideable )
		{
			collideable.Process( root );
			if( root.IsInternal() )
			{
				EnumNodes( root._children[0], collideable );
				EnumNodes( root._children[1], collideable );
			}
		}



		public static void EnumLeaves( btDbvtNode root, ICollide collideable )
		{
			if( root.IsInternal() )
			{
				EnumLeaves( root._children[0], collideable );
				EnumLeaves( root._children[1], collideable );
			}
			else
			{
				collideable.Process( root );
			}
		}

		private static btList<sStkNN> CollideTTStack = new btList<sStkNN>( DOUBLE_STACKSIZE );
		private static int CollideTTCount = 0;

		public static void CollideTT( btDbvtNode root0, btDbvtNode root1, ICollide collideable )
		{
			CollideTTCount++;
			Debug.Assert( CollideTTCount < 2 );
			CollideTTStack.Clear();

			if( root0 != null && root1 != null )
			{
				int depth = 1;
				int treshold = DOUBLE_STACKSIZE - 4;
				CollideTTStack[0] = new sStkNN( root0, root1 );

				do
				{
					sStkNN p = CollideTTStack[--depth];

					if( depth > treshold )
					{
						CollideTTStack.Count = ( CollideTTStack.Count * 2 );
						treshold = CollideTTStack.Count - 4;
					}

					if( p.a == p.b )
					{
						if( p.a.IsInternal() )
						{
							CollideTTStack[depth++] = new sStkNN( p.a._children[0], p.a._children[0] );
							CollideTTStack[depth++] = new sStkNN( p.a._children[1], p.a._children[1] );
							CollideTTStack[depth++] = new sStkNN( p.a._children[0], p.a._children[1] );
						}
					}
					else if( btDbvtVolume.Intersect( ref p.a.volume, ref p.b.volume ) )
					{
						if( p.a.IsInternal() )
						{
							if( p.b.IsInternal() )
							{
								CollideTTStack[depth++] = new sStkNN( p.a._children[0], p.b._children[0] );
								CollideTTStack[depth++] = new sStkNN( p.a._children[1], p.b._children[0] );
								CollideTTStack[depth++] = new sStkNN( p.a._children[0], p.b._children[1] );
								CollideTTStack[depth++] = new sStkNN( p.a._children[1], p.b._children[1] );
							}
							else
							{
								CollideTTStack[depth++] = new sStkNN( p.a._children[0], p.b );
								CollideTTStack[depth++] = new sStkNN( p.a._children[1], p.b );
							}
						}
						else
						{
							if( p.b.IsInternal() )
							{
								CollideTTStack[depth++] = new sStkNN( p.a, p.b._children[0] );
								CollideTTStack[depth++] = new sStkNN( p.a, p.b._children[1] );
							}
							else
							{
								collideable.Process( p.a, p.b );
							}
						}
					}
				} while( depth > 0 );
			}
			CollideTTCount--;
		}


		static btList<sStkNN> m_stkStack = new btList<sStkNN>();

		public static void CollideTTpersistentStack( btDbvtNode root0,
								  btDbvtNode root1,
								  ICollide collideable )
		{
			//CollideTT(root0, root1, collideable);
			//return;
			if( root0 != null && root1 != null )
			{
				int depth = 1;
				int treshold = DOUBLE_STACKSIZE - 4;

				m_stkStack.Count = DOUBLE_STACKSIZE;
				m_stkStack[0] = new sStkNN( root0, root1 );
				do
				{
					sStkNN p = m_stkStack[--depth];
					if( depth > treshold )
					{
						m_stkStack.Count = ( m_stkStack.Count * 2 );
						treshold = m_stkStack.Count - 4;
					}
					if( p.a == p.b )
					{
						if( p.a.IsInternal() )
						{
							m_stkStack[depth++] = new sStkNN( p.a._children[0], p.a._children[0] );
							m_stkStack[depth++] = new sStkNN( p.a._children[1], p.a._children[1] );
							m_stkStack[depth++] = new sStkNN( p.a._children[0], p.a._children[1] );
						}
					}
					else if( btDbvtVolume.Intersect( ref p.a.volume, ref p.b.volume ) )
					{
						if( p.a.IsInternal() )
						{
							if( p.b.IsInternal() )
							{
								m_stkStack[depth++] = new sStkNN( p.a._children[0], p.b._children[0] );
								m_stkStack[depth++] = new sStkNN( p.a._children[1], p.b._children[0] );
								m_stkStack[depth++] = new sStkNN( p.a._children[0], p.b._children[1] );
								m_stkStack[depth++] = new sStkNN( p.a._children[1], p.b._children[1] );
							}
							else
							{
								m_stkStack[depth++] = new sStkNN( p.a._children[0], p.b );
								m_stkStack[depth++] = new sStkNN( p.a._children[1], p.b );
							}
						}
						else
						{
							if( p.b.IsInternal() )
							{
								m_stkStack[depth++] = new sStkNN( p.a, p.b._children[0] );
								m_stkStack[depth++] = new sStkNN( p.a, p.b._children[1] );
							}
							else
							{
								collideable.Process( p.a, p.b );
							}
						}
					}
				} while( depth > 0 );
			}
		}




		public static void rayTest( btDbvtNode root,
								ref btVector3 rayFrom,
								ref btVector3 rayTo,
								ICollide policy )
		{

			using( btDbvtStackDataBlock stackDataBlock = new btDbvtStackDataBlock() )
			{
				if( root != null )
				{
					btVector3 rayDir = ( rayTo - rayFrom );
					rayDir.normalize();

					///what about division by zero? -. just set rayDirection[i] to INF/BT_LARGE_FLOAT
					btVector3 rayDirectionInverse = new btVector3(
						rayDir.x == 0.0f ? btScalar.BT_LARGE_FLOAT : 1.0f / rayDir.x,
						rayDir.y == 0.0f ? btScalar.BT_LARGE_FLOAT : 1.0f / rayDir.y,
						rayDir.z == 0.0f ? btScalar.BT_LARGE_FLOAT : 1.0f / rayDir.z );

					stackDataBlock.signs[0] = rayDirectionInverse.x < 0.0f ? (uint)1 :0;
					stackDataBlock.signs[1] = rayDirectionInverse.y < 0.0f ? (uint)1 :0;
					stackDataBlock.signs[2] = rayDirectionInverse.z < 0.0f ? (uint)1 :0;


					btVector3 tmp; rayTo.Sub( ref rayFrom, out tmp );
					double lambda_max = btVector3.dot( ref rayDir, ref tmp );


					int depth = 1;
					int treshold = DOUBLE_STACKSIZE - 2;

					stackDataBlock.stack.Count = ( DOUBLE_STACKSIZE );
					stackDataBlock.stack[0] = root;
					do
					{
						btDbvtNode node = stackDataBlock.stack[--depth];

						stackDataBlock.bounds[0] = node.volume.Mins();
						stackDataBlock.bounds[1] = node.volume.Maxs();

						double tmin = 1.0f, lambda_min = 0.0f;
						bool result1 = btAabbUtil.btRayAabb2( ref rayFrom, ref rayDirectionInverse, stackDataBlock.signs, stackDataBlock.bounds, out tmin, lambda_min, lambda_max );

#if COMPARE_BTRAY_AABB2
				double param=1.0f;
				bool result2 = AabbUtil.RayAabb(ref rayFrom,ref rayTo,node.volume.Mins(),node.volume.Maxs(),param,resultNormal);
				Debug.Assert(result1 == result2);
#endif //TEST_BTRAY_AABB2

						if( result1 )
						{
							if( node.IsInternal() )
							{
								if( depth > treshold )
								{
									stackDataBlock.stack.Count = ( stackDataBlock.stack.Count * 2 );
									treshold = stackDataBlock.stack.Count - 2;
								}
								stackDataBlock.stack[depth++] = node._children[0];
								stackDataBlock.stack[depth++] = node._children[1];
							}
							else
							{
								policy.Process( node );
							}
						}
					} while( depth != 0 );

				}
			}
		}


		public void rayTestInternal( btDbvtNode root,
									ref btVector3 rayFrom,
									ref btVector3 rayTo,
									ref btVector3 rayDirectionInverse,
									uint[] signs,
									double lambda_max,
									ref btVector3 aabbMin,
									ref btVector3 aabbMax,
									ICollide policy )
		{
			using( btDbvtStackDataBlock stackDataBlock = BulletGlobals.DbvtStackDataBlockPool.Get() )
			{
				//	(void) rayTo;
				//DBVT_CHECKTYPE
				if( root != null )
				{
					btVector3 resultNormal = new btVector3( 0, 1, 0 );

					int depth = 1;
					int treshold = DOUBLE_STACKSIZE - 2;
					stackDataBlock.stack[0] = root;
					do
					{
						btDbvtNode node = stackDataBlock.stack[--depth];
						stackDataBlock.bounds[0] = node.volume.Mins() - aabbMax;
						stackDataBlock.bounds[1] = node.volume.Maxs() - aabbMin;
						double tmin = 1.0f, lambda_min = 0.0f;
						bool result1 = btAabbUtil.btRayAabb2( ref rayFrom, ref rayDirectionInverse, signs, stackDataBlock.bounds, out tmin, lambda_min, lambda_max );
						if( result1 )
						{
							if( node.IsInternal() )
							{
								if( depth > treshold )
								{
									stackDataBlock.stack.Count = ( stackDataBlock.stack.Count * 2 );
									treshold = stackDataBlock.stack.Count - 2;
								}
								stackDataBlock.stack[depth++] = node._children[0];
								stackDataBlock.stack[depth++] = node._children[1];
							}
							else
							{
								policy.Process( node );
							}
						}
					} while( depth != 0 );
				}

			}
		}



		private static Stack<btDbvtNode> CollideTVStack = new Stack<btDbvtNode>( SIMPLE_STACKSIZE );
		private static int CollideTVCount = 0;

		public static void CollideTV( btDbvtNode root, ref btDbvtVolume volume, ICollide collideable )
		{
			CollideTVCount++;
			Debug.Assert( CollideTVCount < 2 );
			CollideTVStack.Clear();
			if( root != null )
			{
				CollideTVStack.Push( root );
				do
				{
					btDbvtNode n = CollideTVStack.Pop();
					if( btDbvtVolume.Intersect( ref n.volume, ref volume ) )
					{
						if( n.IsInternal() )
						{
							CollideTVStack.Push( n._children[0] );
							CollideTVStack.Push( n._children[1] );
						}
						else
						{
							collideable.Process( n );
						}
					}
				} while( CollideTVStack.Count > 0 );
			}
			CollideTVCount--;
		}

		//
		public static btDbvtVolume Bounds( btList<btDbvtNode> leafs )
		{
			btDbvtVolume volume = leafs[0].volume;
			for( int i = 1, ni = leafs.Count; i < ni; ++i )
			{
				btDbvtVolume.Merge( ref volume, ref leafs[i].volume, ref volume );
			}
			return ( volume );
		}


		//
		public static void BottomUp( btDbvt pdbvt, btList<btDbvtNode> leaves )
		{
			while( leaves.Count > 1 )
			{
				double minsize = double.MaxValue;
				int[] minidx = { -1, -1 };
				for( int i = 0; i < leaves.Count; ++i )
				{
					for( int j = i + 1; j < leaves.Count; ++j )
					{
						btDbvtVolume mergeResults = btDbvtVolume.Merge( ref leaves[i].volume, ref leaves[j].volume );
						double sz = Size( ref mergeResults );
						if( sz < minsize )
						{
							minsize = sz;
							minidx[0] = i;
							minidx[1] = j;
						}
					}
				}
				btDbvtNode[] n = { leaves[minidx[0]], leaves[minidx[1]] };
				btDbvtNode p = CreateNode( pdbvt, null, ref n[0].volume, ref n[1].volume, null );
				p._children[0] = n[0];
				p._children[1] = n[1];
				n[0].parent = p;
				n[1].parent = p;
				leaves[minidx[0]] = p;
				leaves.Swap( minidx[1], leaves.Count - 1 );
				leaves.Count--;// PopBack();
			}
		}


		//

		public static btVector3[] axis = { new btVector3( 1, 0, 0 ), new btVector3( 0, 1, 0 ), new btVector3( 0, 0, 1 ) };

		public static btDbvtNode TopDown( btDbvt pdbvt, btList<btDbvtNode> leaves, int bu_treshold )
		{
			if( leaves.Count > 1 )
			{
				if( leaves.Count > bu_treshold )
				{
					btDbvtVolume vol = Bounds( leaves );
					btVector3 org = vol.Center();
					btList<btDbvtNode>[] sets = { new btList<btDbvtNode>(), new btList<btDbvtNode>() };
					int bestaxis = -1;
					int bestmidp = leaves.Count;
					int[] a1 = new int[] { 0, 0 };
					int[] a2 = new int[] { 0, 0 };
					int[] a3 = new int[] { 0, 0 };

					int[][] splitcount = new int[][] { a1, a2, a3 };
					int i;
					for( i = 0; i < leaves.Count; ++i )
					{
						btVector3 x = leaves[i].volume.Center() - org;
						for( int j = 0; j < 3; ++j )
						{
							++splitcount[j][btVector3.dot( ref x, ref axis[j] ) > 0 ? 1 : 0];
						}
					}
					for( i = 0; i < 3; ++i )
					{
						if( ( splitcount[i][0] > 0 ) && ( splitcount[i][1] > 0 ) )
						{
							int midp = (int)Math.Abs( ( splitcount[i][0] - splitcount[i][1] ) );
							if( midp < bestmidp )
							{
								bestaxis = i;
								bestmidp = midp;
							}
						}
					}
					if( bestaxis >= 0 )
					{
						sets[0].Capacity = ( splitcount[bestaxis][0] );
						sets[1].Capacity = ( splitcount[bestaxis][1] );
						Split( leaves, sets[0], sets[1], ref org, ref axis[bestaxis] );
					}
					else
					{
						sets[0].Capacity = ( leaves.Count / 2 + 1 );
						sets[1].Capacity = ( leaves.Count / 2 );
						for( int i2 = 0, ni = leaves.Count; i2 < ni; ++i2 )
						{
							sets[i2 & 1].Add( leaves[i2] );
						}
					}
					btDbvtNode node = CreateNode( pdbvt, null, ref vol, null );
					node._children[0] = TopDown( pdbvt, sets[0], bu_treshold );
					node._children[1] = TopDown( pdbvt, sets[1], bu_treshold );
					node._children[0].parent = node;
					node._children[1].parent = node;
					return ( node );
				}
				else
				{
					BottomUp( pdbvt, leaves );
					return ( leaves[0] );
				}
			}
			return ( leaves[0] );
		}

		public static btDbvtNode CreateNode( btDbvt pdbvt, btDbvtNode parent, int data )
		{
			btDbvtNode node = BulletGlobals.DbvtNodePool.Get();
			node.parent = parent;
			node.data = null;
			node.dataAsInt = data;
			node._children[0] = null;
			node._children[1] = null;
			return ( node );
		}


		public static btDbvtNode CreateNode( btDbvt pdbvt, btDbvtNode parent, Object data )
		{
			btDbvtNode node = BulletGlobals.DbvtNodePool.Get();
			node.parent = parent;
			node.data = data;
			if( node.data is int )
			{
				//Debug.Assert(false);
				node.dataAsInt = (int)node.data;
			}
			node._children[0] = null;
			node._children[1] = null;
			return ( node );
		}


		public static btDbvtNode CreateNode2( btDbvt tree, btDbvtNode aparent, ref btDbvtVolume avolume, Object adata )
		{
			btDbvtNode node = BulletGlobals.DbvtNodePool.Get();
			node.volume = avolume;
			node.parent = aparent;
			node.data = adata;
			node._children[0] = null;
			node._children[1] = null;

			if( node.data is int )
			{
				Debug.Assert( false );
				node.dataAsInt = (int)node.data;
			}

			return node;
		}


		public static btDbvtNode CreateNode( btDbvt pdbvt,
									   btDbvtNode parent,
									   ref btDbvtVolume volume,
									   int data )
		{
			btDbvtNode node = CreateNode( pdbvt, parent, data );
			node.volume = volume;
			return ( node );
		}


		//
		public static btDbvtNode CreateNode( btDbvt pdbvt,
											   btDbvtNode parent,
											   ref btDbvtVolume volume,
											   Object data )
		{
			btDbvtNode node = CreateNode( pdbvt, parent, data );
			node.volume = volume;
			return ( node );
		}

		//
		public static btDbvtNode CreateNode( btDbvt pdbvt,
									btDbvtNode parent,
									ref btDbvtVolume volume0,
									ref btDbvtVolume volume1,
									Object data )
		{
			btDbvtNode node = CreateNode( pdbvt, parent, data );
			btDbvtVolume.Merge( ref volume0, ref volume1, ref node.volume );
			return ( node );
		}

		public static void DeleteNode( btDbvt pdbvt, btDbvtNode node )
		{
			//btAlignedFree(pdbvt.m_free);
			//pdbvt.m_free = node;
			node.Reset();
			BulletGlobals.DbvtNodePool.Free( node );
		}

		public static void RecurseDeleteNode( btDbvt pdbvt, btDbvtNode node )
		{
			if( !node.IsLeaf() )
			{
				RecurseDeleteNode( pdbvt, node._children[0] );
				RecurseDeleteNode( pdbvt, node._children[1] );
			}
			if( node == pdbvt.m_root )
			{
				pdbvt.m_root = null;
			}
			DeleteNode( pdbvt, node );
		}



		public static void InsertLeaf( btDbvt pdbvt, btDbvtNode root, btDbvtNode leaf )
		{
			if( pdbvt.Root == null )
			{
				pdbvt.Root = leaf;
				leaf.parent = null;
			}
			else
			{
				if( !root.IsLeaf() )
				{
					do
					{
						root = root._children[btDbvtVolume.Select( ref leaf.volume,
						ref root._children[0].volume,
						ref root._children[1].volume )];

					} while( !root.IsLeaf() );
				}
				btDbvtNode prev = root.parent;
				btDbvtVolume mergeResults = btDbvtVolume.Merge( ref leaf.volume, ref root.volume );

				btDbvtNode node = CreateNode2( pdbvt, prev, ref mergeResults, null );
				if( prev != null )
				{
					prev._children[IndexOf( root )] = node;
					node._children[0] = root;
					root.parent = node;
					node._children[1] = leaf;
					leaf.parent = node;
					do
					{
						if( !prev.volume.Contain( ref node.volume ) )
						{
							btDbvtVolume.Merge( ref prev._children[0].volume, ref prev._children[1].volume, ref prev.volume );
						}
						else
						{
							break;
						}
						node = prev;
					} while( null != ( prev = node.parent ) );
				}
				else
				{
					node._children[0] = root;
					root.parent = node;
					node._children[1] = leaf;
					leaf.parent = node;
					pdbvt.Root = node;
				}
			}
		}



		public static btDbvtNode RemoveLeaf( btDbvt pdbvt, btDbvtNode leaf )
		{
			if( leaf == pdbvt.Root )
			{
				pdbvt.Root = null;
				return null;
			}
			else
			{
				btDbvtNode parent = leaf.parent;
				btDbvtNode prev = parent.parent;
				btDbvtNode sibling = parent._children[1 - IndexOf( leaf )];
				if( prev != null )
				{
					prev._children[IndexOf( parent )] = sibling;
					sibling.parent = prev;
					DeleteNode( pdbvt, parent );
					while( prev != null )
					{
						btDbvtVolume pb = prev.volume;
						btDbvtVolume.Merge( ref prev._children[0].volume, ref prev._children[1].volume, ref prev.volume );
						if( btDbvtVolume.NotEqual( ref pb, ref prev.volume ) )
						{
							sibling = prev;
							prev = prev.parent;
						}
						else
						{
							break;
						}
					}
					return ( prev != null ? prev : pdbvt.Root );
				}
				else
				{
					pdbvt.Root = sibling;
					sibling.parent = null;
					DeleteNode( pdbvt, parent );
					return ( pdbvt.Root );
				}
			}
		}






		public static int IndexOf( btDbvtNode node )
		{
			return ( node.parent._children[1] == node ) ? 1 : 0;
		}



		// volume+edge lengths
		public static double Size( ref btDbvtVolume a )
		{
			btVector3 edges = a.Lengths();
			return ( edges.x * edges.y * edges.z +
				edges.x + edges.y + edges.z );
		}



		public static int SIMPLE_STACKSIZE = 64;
		public static int DOUBLE_STACKSIZE = SIMPLE_STACKSIZE * 2;

		public btDbvtNode m_root;
		//public btDbvtNode m_free;

		public int m_lkhd;
		public int m_leaves;
		public uint m_opath;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	public class btDbvtNode
	{
		public btDbvtNode()
		{
			id = counter++;
		}
		public btDbvtNode( btDbvt tree, btDbvtNode aparent, ref btDbvtVolume avolume, Object adata )
			: this()
		{
			volume = avolume;
			parent = aparent;
			data = adata;
			if( data is int )
			{
				dataAsInt = (int)data;
			}
		}

		public void Reset()
		{
			parent = null;

			data = null;
			dataAsInt = 0;
			// bump id as well? we're effectively a new node..
			id = counter++;
		}

		public btDbvtVolume volume;
		public btDbvtNode parent;
		public btDbvtNode[] _children = new btDbvtNode[2];
		public Object data;
		public int dataAsInt;
		public int id;

		public bool IsLeaf() { return ( _children[1] == null ); }
		public bool IsInternal() { return ( !IsLeaf() ); }

		public static int counter = 0;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	public struct sStkNN
	{
		public btDbvtNode a;
		public btDbvtNode b;

		public sStkNN( btDbvtNode na, btDbvtNode nb ) { a = na; b = nb; }
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	public struct sStkNP
	{
		public btDbvtNode node;
		public uint mask;
		public sStkNP( btDbvtNode n, uint m ) { node = n; mask = m; }
	};

	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	public struct sStkNPS
	{
		public btDbvtNode node;
		public uint mask;
		public double value;
		public sStkNPS( btDbvtNode n, uint m, double v ) { node = n; mask = m; value = v; }
	};

	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	public struct btDbvtVolume
	{
		public btVector3 _min;
		public btVector3 _max;

		public btVector3 Center() { return ( _max + _min ) * 0.5; }
		public btVector3 Extent() { return ( _max - _min ) * 0.5f; }
		public btVector3 Mins() { return _min; }    // should be ref?
		public btVector3 Maxs() { return _max; }    // should be ref?
		public btVector3 Lengths() { return new btVector3(); }

		public static double Proximity( ref btDbvtVolume a, ref btDbvtVolume b )
		{
			btVector3 d = ( a._min + a._max ) - ( b._min + b._max );
			return ( Math.Abs( d.x ) + Math.Abs( d.y ) + Math.Abs( d.z ) );
		}


		public static btDbvtVolume Merge( ref btDbvtVolume a, ref btDbvtVolume b )
		{
			btDbvtVolume res = new btDbvtVolume();
			Merge( ref a, ref b, ref res );
			return ( res );
		}

		public static void Merge( ref btDbvtVolume a, ref btDbvtVolume b, ref btDbvtVolume r )
		{
			//r = a;
			//SetMin(ref r._min, ref b._min);
			//SetMax(ref r._max, ref b._max);
			btVector3.setMin( ref a._min, ref b._min, out r._min );
			btVector3.setMax( ref a._max, ref b._max, out r._max );

		}

		public static int Select( ref btDbvtVolume o,
							   ref btDbvtVolume a,
							   ref btDbvtVolume b )
		{
			return ( Proximity( ref o, ref a ) < Proximity( ref o, ref b ) ? 0 : 1 );
		}

		public static bool NotEqual( ref btDbvtVolume a, ref btDbvtVolume b )
		{
			return ( ( a._min.x != b._min.x ) ||
					( a._min.y != b._min.y ) ||
					( a._min.z != b._min.z ) ||
					( a._max.x != b._max.x ) ||
					( a._max.y != b._max.y ) ||
					( a._max.z != b._max.z ) );

		}


		public static btDbvtVolume FromCE( ref btVector3 c, ref btVector3 e )
		{
			btDbvtVolume box;
			box._min = c - e; box._max = c + e;
			return ( box );
		}
		public static btDbvtVolume FromCR( ref btVector3 c, double r )
		{
			btVector3 temp = new btVector3( r );
			return ( FromCE( ref c, ref temp ) );
		}
		public static btDbvtVolume FromMM( ref btVector3 mi, ref btVector3 mx )
		{
			btDbvtVolume box;
			box._min = mi; box._max = mx;
			return ( box );
		}

		public static btDbvtVolume FromPoints( btList<btVector3> points )
		{
			btDbvtVolume box;
			box._min = box._max = points[0];
			for( int i = 1; i < points.Count; ++i )
			{
				btVector3 temp = points[i];
				//SetMin(ref box._min, ref temp);
				//SetMax(ref box._max, ref temp);
				box._min.setMin( ref temp );
				box._max.setMin( ref temp );
			}
			return ( box );
		}

		public static btDbvtVolume FromPoints( btList<btList<btVector3>> points )
		{
			return new btDbvtVolume();
		}

		public void Expand( btVector3 e )
		{
			_min -= e; _max += e;
		}

		public void SignedExpand( btVector3 e )
		{
			if( e.x > 0 ) _max.x = _max.x + e.x; else _min.x = _min.x + e.x;
			if( e.y > 0 ) _max.y = _max.y + e.y; else _min.y = _min.y + e.y;
			if( e.z > 0 ) _max.z = _max.z + e.z; else _min.z = _min.z + e.z;
		}
		public bool Contain( ref btDbvtVolume a )
		{
			return ( ( _min.x <= a._min.x ) &&
				( _min.y <= a._min.y ) &&
				( _min.z <= a._min.z ) &&
				( _max.x >= a._max.x ) &&
				( _max.y >= a._max.y ) &&
				( _max.z >= a._max.z ) );
		}

		public static bool Intersect( ref btDbvtVolume a, ref btDbvtVolume b )
		{
			return ( ( a._min.x <= b._max.x ) &&
				( a._max.x >= b._min.x ) &&
				( a._min.y <= b._max.y ) &&
				( a._max.y >= b._min.y ) &&
				( a._min.z <= b._max.z ) &&
				( a._max.z >= b._min.z ) );
		}

		public static bool Intersect( btDbvtVolume a, ref btVector3 b )
		{
			return ( ( b.x >= a._min.x ) &&
				( b.y >= a._min.y ) &&
				( b.z >= a._min.z ) &&
				( b.x <= a._max.x ) &&
				( b.y <= a._max.y ) &&
				( b.z <= a._max.z ) );
		}



		public int Classify( ref btVector3 n, double o, int s )
		{
			btVector3 pi, px;
			switch( s )
			{
				case ( 0 + 0 + 0 ):
					{
						px = new btVector3( _min.x, _min.y, _min.z );
						pi = new btVector3( _max.x, _max.y, _max.z );
						break;
					}
				case ( 1 + 0 + 0 ):
					{
						px = new btVector3( _max.x, _min.y, _min.z );
						pi = new btVector3( _min.x, _max.y, _max.z ); break;
					}
				case ( 0 + 2 + 0 ):
					{
						px = new btVector3( _min.x, _max.y, _min.z );
						pi = new btVector3( _max.x, _min.y, _max.z ); break;
					}
				case ( 1 + 2 + 0 ):
					{
						px = new btVector3( _max.x, _max.y, _min.z );
						pi = new btVector3( _min.x, _min.y, _max.z ); break;
					}
				case ( 0 + 0 + 4 ):
					{
						px = new btVector3( _min.x, _min.y, _max.z );
						pi = new btVector3( _max.x, _max.y, _min.z ); break;
					}
				case ( 1 + 0 + 4 ):
					{
						px = new btVector3( _max.x, _min.y, _max.z );
						pi = new btVector3( _min.x, _max.y, _min.z ); break;
					}
				case ( 0 + 2 + 4 ):
					{
						px = new btVector3( _min.x, _max.y, _max.z );
						pi = new btVector3( _max.x, _min.y, _min.z ); break;
					}
				case ( 1 + 2 + 4 ):
					{
						px = new btVector3( _max.x, _max.y, _max.z );
						pi = new btVector3( _min.x, _min.y, _min.z ); break;
					}
				default:
					{
						px = new btVector3();
						pi = new btVector3();
						break;
					}
			}
			if( ( btVector3.dot( ref n, ref px ) + o ) < 0 ) return ( -1 );
			if( ( btVector3.dot( ref n, ref pi ) + o ) >= 0 ) return ( +1 );
			return ( 0 );
		}

		public double ProjectMinimum( ref btVector3 v, uint signs )
		{
			btVector3[] b = { _max, _min };
			btVector3 p = new btVector3( b[( signs >> 0 ) & 1].x,
									b[( signs >> 1 ) & 1].y,
									b[( signs >> 2 ) & 1].z );
			return ( btVector3.dot( ref p, ref v ) );
		}

		public void AddSpan( ref btVector3 d, ref double smi, ref double smx )
		{
			for( int i = 0; i < 3; ++i )
			{
				if( d[i] < 0 )
				{ smi += _max[i] * d[i]; smx += _min[i] * d[i]; }
				else
				{ smi += _min[i] * d[i]; smx += _max[i] * d[i]; }
			}
		}

	}


	public abstract class DefaultCollide : btDbvt.ICollide
	{
		public virtual void Process( btDbvtNode n, btDbvtNode n2 ) { }
		public virtual void Process( btDbvtNode n ) { }

		public virtual void Process( btDbvtNode n, double f )
		{
			Process( n );
		}
		public virtual bool Descent( btDbvtNode n )
		{
			return true;
		}
		public virtual bool AllLeaves( btDbvtNode n )
		{
			return true;
		}
	}

	public class DbvtDraw : btDbvt.ICollide
	{
		public virtual void Process( btDbvtNode n, btDbvtNode n2 ) { }
		public virtual void Process( btDbvtNode n )
		{
			btVector3 color = new btVector3( 1, 1, 1 );
			BulletGlobals.gDebugDraw.drawBox( ref n.volume._min, ref n.volume._max, ref btTransform.Identity, ref color );

		}

		public virtual void Process( btDbvtNode n, double f )
		{
			//IndexedMatrix im = IndexedMatrix.Identity;
			btVector3 color = new btVector3( 1, 1, 1 );
			BulletGlobals.gDebugDraw.drawBox( ref n.volume._min, ref n.volume._max, ref btTransform.Identity, ref color );

		}
		public virtual bool Descent( btDbvtNode n )
		{
			return true;
		}
		public virtual bool AllLeaves( btDbvtNode n )
		{
			return true;
		}
	}



	public class btDbvtStackDataBlock : IDisposable
	{
		public btList<btDbvtNode> stack = new btList<btDbvtNode>();
		public uint[] signs = new uint[3];
		public btVector3[] bounds = new btVector3[2];

		public void Dispose()
		{
			BulletGlobals.DbvtStackDataBlockPool.Free( this );
		}

	}


}
