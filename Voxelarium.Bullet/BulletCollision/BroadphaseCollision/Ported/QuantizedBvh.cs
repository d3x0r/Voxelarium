//#define DEBUG_CHECK_DEQUANTIZATION 1
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

using System.Diagnostics;
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.Collision.BroadPhase
{


	//http://msdn.microsoft.com/library/default.asp?url=/library/en-us/vclang/html/vclrf__m128.asp

	///btQuantizedBvhNode is a compressed aabb node, 16 bytes.
	///Node can be used for leafnode or internal node. Leafnodes can point to 32-bit triangle index (non-negative range).
	internal class btQuantizedBvhNode
	{
		//Note: currently we have 16 bytes per quantized node
		internal const int MAX_SUBTREE_SIZE_IN_BYTES = 2048;

		// 10 gives the potential for 1024 parts, with at most 2^21 (2097152) (minus one
		// actually) triangles each (since the sign bit is reserved
		internal const int MAX_NUM_PARTS_IN_BITS = 10;


		//12 bytes
		internal ushort[] m_quantizedAabbMin = new ushort[3];
		internal ushort[] m_quantizedAabbMax = new ushort[3];
		//4 bytes
		int m_escapeIndexOrTriangleIndex;

		bool isLeafNode()
		{
			//skipindex is negative (internal node), triangleindex >=0 (leafnode)
			return ( m_escapeIndexOrTriangleIndex >= 0 );
		}
		int getEscapeIndex()
		{
			Debug.Assert( !isLeafNode() );
			return -m_escapeIndexOrTriangleIndex;
		}
		int getTriangleIndex()
		{
			Debug.Assert( isLeafNode() );
			uint x = 0;
			uint y = ( ~( x & 0 ) ) << ( 31 - MAX_NUM_PARTS_IN_BITS );
			// Get only the lower bits where the triangle index is stored
			return (int)( m_escapeIndexOrTriangleIndex & ~( y ) );
		}
		int getPartId()
		{
			Debug.Assert( isLeafNode() );
			// Get only the highest bits where the part index is stored
			return ( m_escapeIndexOrTriangleIndex >> ( 31 - MAX_NUM_PARTS_IN_BITS ) );
		}
	}


	/// btOptimizedBvhNode contains both internal and leaf node information.
	/// Total node size is 44 bytes / node. You can use the compressed version of 16 bytes.
	internal class btOptimizedBvhNode
	{


		//32 bytes
		btVector3 m_aabbMinOrg;
		btVector3 m_aabbMaxOrg;

		//4
		int m_escapeIndex;

		//8
		//for child nodes
		int m_subPart;
		int m_triangleIndex;

		//pad the size to 64 bytes
		//char m_padding[20];
	};


	///btBvhSubtreeInfo provides info to gather a subtree of limited size
	internal class btBvhSubtreeInfo
	{
		//12 bytes
		public ushort[] m_quantizedAabbMin = new ushort[3];
		public ushort[] m_quantizedAabbMax = new ushort[3];
		//4 bytes, points to the root of the subtree
		int m_rootNodeIndex;
		//4 bytes
		int m_subtreeSize;
		//int m_padding[3];

		btBvhSubtreeInfo()
		{
			//memset(m_padding, 0, sizeof(m_padding));
		}


		void setAabbFromQuantizeNode( btQuantizedBvhNode quantizedNode )
		{
			m_quantizedAabbMin[0] = quantizedNode.m_quantizedAabbMin[0];
			m_quantizedAabbMin[1] = quantizedNode.m_quantizedAabbMin[1];
			m_quantizedAabbMin[2] = quantizedNode.m_quantizedAabbMin[2];
			m_quantizedAabbMax[0] = quantizedNode.m_quantizedAabbMax[0];
			m_quantizedAabbMax[1] = quantizedNode.m_quantizedAabbMax[1];
			m_quantizedAabbMax[2] = quantizedNode.m_quantizedAabbMax[2];
		}
	}
	;


	internal abstract class btNodeOverlapCallback
	{
		public abstract void processNode( int subPart, int triangleIndex );
	};


	///for code readability:
	internal class NodeArray : btList<btOptimizedBvhNode> { }
	internal class QuantizedNodeArray : btList<btQuantizedBvhNode> { }
	internal class BvhSubtreeInfoArray : btList<btBvhSubtreeInfo> { }


	///The btQuantizedBvh class stores an AABB tree that can be quickly traversed on CPU and Cell SPU.
	///It is used by the btBvhTriangleMeshShape as midphase, and by the btMultiSapBroadphase.
	///It is recommended to use quantization for better performance and lower memory requirements.
	internal class btQuantizedBvh
	{
		public enum btTraversalMode
		{
			TRAVERSAL_STACKLESS = 0,
			TRAVERSAL_STACKLESS_CACHE_FRIENDLY,
			TRAVERSAL_RECURSIVE
		};


		protected btVector3 m_bvhAabbMin;
		protected btVector3 m_bvhAabbMax;
		protected btVector3 m_bvhQuantization;

		protected int m_bulletVersion;    //for serialization versioning. It could also be used to detect endianess.

		protected int m_curNodeIndex;
		//quantization data
		protected bool m_useQuantization;



		protected NodeArray m_leafNodes;
		protected NodeArray m_contiguousNodes;
		protected QuantizedNodeArray m_quantizedLeafNodes;
		protected QuantizedNodeArray m_quantizedContiguousNodes;

		protected btTraversalMode m_traversalMode;
		protected BvhSubtreeInfoArray m_SubtreeHeaders;

		//This is only used for serialization so we don't have to add serialization directly to List
		protected int m_subtreeHeaderCount;





		///two versions, one for quantized and normal nodes. This allows code-reuse while maintaining readability (no template/macro!)
		///this might be refactored into a virtual, it is usually not calculated at run-time
		protected void setInternalNodeAabbMin( int nodeIndex, ref btVector3 aabbMin )
		{
			if( m_useQuantization )
			{
				quantize( &m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0], aabbMin, 0 );
			}
			else
			{
				m_contiguousNodes[nodeIndex].m_aabbMinOrg = aabbMin;

			}
		}
		protected void setInternalNodeAabbMax( int nodeIndex, ref btVector3 aabbMax )
		{
			if( m_useQuantization )
			{
				quantize( &m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0], aabbMax, 1 );
			}
			else
			{
				m_contiguousNodes[nodeIndex].m_aabbMaxOrg = aabbMax;
			}
		}

		protected btVector3 getAabbMin( int nodeIndex )
		{
			if( m_useQuantization )
			{
				return unQuantize( &m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMin[0] );
			}
			//non-quantized
			return m_leafNodes[nodeIndex].m_aabbMinOrg;

		}
		protected btVector3 getAabbMax( int nodeIndex )
		{
			if( m_useQuantization )
			{
				return unQuantize( &m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMax[0] );
			}
			//non-quantized
			return m_leafNodes[nodeIndex].m_aabbMaxOrg;

		}


		protected void setInternalNodeEscapeIndex( int nodeIndex, int escapeIndex )
		{
			if( m_useQuantization )
			{
				m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = -escapeIndex;
			}
			else
			{
				m_contiguousNodes[nodeIndex].m_escapeIndex = escapeIndex;
			}

		}

		void mergeInternalNodeAabb( int nodeIndex, ref btVector3 newAabbMin, ref btVector3 newAabbMax )
		{
			if( m_useQuantization )
			{
				ushort[] quantizedAabbMin = new ushort[3];
				ushort[] quantizedAabbMax = new ushort[3];
				quantize( quantizedAabbMin, ref newAabbMin, 0 );
				quantize( quantizedAabbMax, ref newAabbMax, 1 );
				for( int i = 0; i < 3; i++ )
				{
					if( m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[i] > quantizedAabbMin[i] )
						m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[i] = quantizedAabbMin[i];

					if( m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[i] < quantizedAabbMax[i] )
						m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[i] = quantizedAabbMax[i];

				}
			}
			else
			{
				//non-quantized
				m_contiguousNodes[nodeIndex].m_aabbMinOrg.setMin( newAabbMin );
				m_contiguousNodes[nodeIndex].m_aabbMaxOrg.setMax( newAabbMax );
			}
		}

#if false
		protected void swapLeafNodes( int firstIndex, int secondIndex );

		protected void assignInternalNodeFromLeafNode( int internalNode, int leafNodeIndex );



		protected void buildTree( int startIndex, int endIndex );

		protected int calcSplittingAxis( int startIndex, int endIndex );

		protected int sortAndCalcSplittingIndex( int startIndex, int endIndex, int splitAxis );

		protected void walkStacklessTree( btNodeOverlapCallback* nodeCallback, ref btVector3 aabbMin, ref btVector3 aabbMax );

		protected void walkStacklessQuantizedTreeAgainstRay( btNodeOverlapCallback* nodeCallback, ref btVector3 raySource, ref btVector3 rayTarget, ref btVector3 aabbMin, ref btVector3 aabbMax, int startNodeIndex, int endNodeIndex );
		protected void walkStacklessTreeAgainstRay( btNodeOverlapCallback* nodeCallback, ref btVector3 raySource, ref btVector3 rayTarget, ref btVector3 aabbMin, ref btVector3 aabbMax, int startNodeIndex, int endNodeIndex );


		///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
		protected void walkRecursiveQuantizedTreeAgainstQueryAabb( btQuantizedBvhNode* currentNode, btNodeOverlapCallback* nodeCallback, ushort int* quantizedQueryAabbMin, ushort int* quantizedQueryAabbMax );

		///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
		protected void walkRecursiveQuantizedTreeAgainstQuantizedTree( btQuantizedBvhNode* treeNodeA, btQuantizedBvhNode* treeNodeB, btNodeOverlapCallback* nodeCallback );




		protected void updateSubtreeHeaders( int leftChildNodexIndex, int rightChildNodexIndex );

	

		public btQuantizedBvh();

		///**************************** expert/internal use only *****************
		///buildInternal is expert use only: assumes that setQuantizationValues and LeafNodeArray are initialized
		public void buildInternal();
		///**************************** expert/internal use only *****************

		public void reportAabbOverlappingNodex( btNodeOverlapCallback* nodeCallback, ref btVector3 aabbMin, ref btVector3 aabbMax );
		public void reportRayOverlappingNodex( btNodeOverlapCallback* nodeCallback, ref btVector3 raySource, ref btVector3 rayTarget );
		public void reportBoxCastOverlappingNodex( btNodeOverlapCallback* nodeCallback, ref btVector3 raySource, ref btVector3 rayTarget, ref btVector3 aabbMin, ref btVector3 aabbMax );
#endif


		public QuantizedNodeArray getLeafNodeArray() { return m_quantizedLeafNodes; }

		public void quantize( ushort[] outdata, ref btVector3 point, int isMax )
		{

			Debug.Assert( m_useQuantization );

			Debug.Assert( point.x <= m_bvhAabbMax.x );
			Debug.Assert( point.y <= m_bvhAabbMax.y );
			Debug.Assert( point.z <= m_bvhAabbMax.z );

			Debug.Assert( point.x >= m_bvhAabbMin.x );
			Debug.Assert( point.y >= m_bvhAabbMin.y );
			Debug.Assert( point.z >= m_bvhAabbMin.z );

			btVector3 v = ( point - m_bvhAabbMin ) * m_bvhQuantization;
			///Make sure rounding is done in a way that unQuantize(quantizeWithClamp(...)) is conservative
			///end-points always set the first bit, so that they are sorted properly (so that neighbouring AABBs overlap properly)
			///@todo: double-check this
			if( isMax )
			{
				outdata[0] = (ushort)( ( (ushort)( v.x + btScalar.BT_ONE ) | 1 ) );
				outdata[1] = (ushort)( ( (ushort)( v.y + btScalar.BT_ONE ) | 1 ) );
				outdata[2] = (ushort)( ( (ushort)( v.z + btScalar.BT_ONE ) | 1 ) );
			}
			else
			{
				outdata[0] = (ushort)( ( (ushort)( v.x ) & 0xfffe ) );
				outdata[1] = (ushort)( ( (ushort)( v.y ) & 0xfffe ) );
				outdata[2] = (ushort)( ( (ushort)(v.z)0xfffe ) );
			}


#if DEBUG_CHECK_DEQUANTIZATION
		btVector3 newPoint = unQuantize(out);
		if (isMax)
		{
			if (newPoint.x < point.x)
			{
				Console.WriteLine("unconservative X, diffX = %f, oldX=%f,newX=%f\n",newPoint.x-point.x, newPoint.x,point.x);
			}
			if (newPoint.y < point.y)
			{
				Console.WriteLine("unconservative Y, diffY = %f, oldY=%f,newY=%f\n",newPoint.y-point.y, newPoint.y,point.y);
			}
			if (newPoint.z < point.z)
			{

				Console.WriteLine("unconservative Z, diffZ = %f, oldZ=%f,newZ=%f\n",newPoint.z-point.z, newPoint.z,point.z);
			}
		} else
		{
			if (newPoint.x > point.x)
			{
				Console.WriteLine("unconservative X, diffX = %f, oldX=%f,newX=%f\n",newPoint.x-point.x, newPoint.x,point.x);
			}
			if (newPoint.y > point.y)
			{
				Console.WriteLine("unconservative Y, diffY = %f, oldY=%f,newY=%f\n",newPoint.y-point.y, newPoint.y,point.y);
			}
			if (newPoint.z > point.z)
			{
				Console.WriteLine("unconservative Z, diffZ = %f, oldZ=%f,newZ=%f\n",newPoint.z-point.z, newPoint.z,point.z);
			}
		}
#endif //DEBUG_CHECK_DEQUANTIZATION

		}


		public void quantizeWithClamp( ushort[] outdata, ref btVector3 point2, int isMax )
		{

			Debug.Assert( m_useQuantization );

			btVector3 clampedPoint = point2;
			clampedPoint.setMax( m_bvhAabbMin );
			clampedPoint.setMin( m_bvhAabbMax );
			quantize( outdata, clampedPoint, isMax );
		}

		public btVector3 unQuantize( ushort[] vecIn )
		{
			btVector3 vecOut;
			vecOut.setValue(
			(double)( vecIn ) / ( m_bvhQuantization.x ),
			(double)( vecIn[1] ) / ( m_bvhQuantization.y ),
			(double)( vecIn[2] ) / ( m_bvhQuantization.z ) );
			vecOut += m_bvhAabbMin;
			return vecOut;
		}

		///setTraversalMode let's you choose between stackless, recursive or stackless cache friendly tree traversal. Note this is only implemented for quantized trees.
		void setTraversalMode( btTraversalMode traversalMode )
		{
			m_traversalMode = traversalMode;
		}


		public QuantizedNodeArray getQuantizedNodeArray()
		{
			return m_quantizedContiguousNodes;
		}


		public BvhSubtreeInfoArray getSubtreeInfoArray()
		{
			return m_SubtreeHeaders;
		}

		////////////////////////////////////////////////////////////////////

		/////Calculate space needed to store BVH for serialization

		////////////////////////////////////////////////////////////////////

		public bool isQuantized()
		{
			return m_useQuantization;
		}



		internal btQuantizedBvh()
		{
			m_bulletVersion = ( BulletGlobals.BT_BULLET_VERSION );
			m_useQuantization = ( false );
			//m_traversalMode(TRAVERSAL_STACKLESS_CACHE_FRIENDLY)
			m_traversalMode = ( btTraversalMode.TRAVERSAL_STACKLESS );
			//m_traversalMode(TRAVERSAL_RECURSIVE)
			m_subtreeHeaderCount = ( 0 ); //PCK: add this line
			m_bvhAabbMin = btVector3.Min;
			m_bvhAabbMax = btVector3.Max;
		}



		void buildInternal()
		{
			///assumes that caller filled in the m_quantizedLeafNodes
			m_useQuantization = true;
			int numLeafNodes = 0;

			if( m_useQuantization )
			{
				//now we have an array of leafnodes in m_leafNodes
				numLeafNodes = m_quantizedLeafNodes.Count;

				m_quantizedContiguousNodes.resize( 2 * numLeafNodes );

			}

			m_curNodeIndex = 0;

			buildTree( 0, numLeafNodes );

			///if the entire tree is small then subtree size, we need to create a header info for the tree
			if( m_useQuantization & !m_SubtreeHeaders.Count )
			{
				btBvhSubtreeInfo & subtree = m_SubtreeHeaders.expand();
				subtree.setAabbFromQuantizeNode( m_quantizedContiguousNodes );
				subtree.m_rootNodeIndex = 0;
				subtree.m_subtreeSize = m_quantizedContiguousNodes[0].isLeafNode() ? 1 : m_quantizedContiguousNodes[0].getEscapeIndex();
			}

			//PCK: update the copy of the size
			m_subtreeHeaderCount = m_SubtreeHeaders.Count;

			//PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
			m_quantizedLeafNodes.clear();
			m_leafNodes.clear();
		}



		///just for debugging, to visualize the individual patches/subtrees
#if DEBUG_PATCH_COLORS
btVector3 color[4]=
{
	btVector3(1,0,0),
	btVector3(0,1,0),
	btVector3(0,0,1),
	btVector3(0,1,1)
};
#endif //DEBUG_PATCH_COLORS



		void setQuantizationValues( ref btVector3 bvhAabbMin, ref btVector3 bvhAabbMax, double quantizationMargin = 1.0 )
		{
			//enlarge the AABB to avoid division by zero when initializing the quantization values
			btVector3 clampValue = new btVector3( quantizationMargin, quantizationMargin, quantizationMargin );
			m_bvhAabbMin = bvhAabbMin - clampValue;
			m_bvhAabbMax = bvhAabbMax + clampValue;
			btVector3 aabbSize = m_bvhAabbMax - m_bvhAabbMin;
			m_bvhQuantization = btVector3( (double)( 65533.0 ), (double)( 65533.0 ), (double)( 65533.0 ) ) / aabbSize;

			m_useQuantization = true;

			{
				ushort[] vecIn = new ushort[3];
				btVector3 v;
				{
					quantize( vecIn, m_bvhAabbMin, false );
					v = unQuantize( vecIn );
					m_bvhAabbMin.setMin( v - clampValue );
				}
				{
					quantize( vecIn, m_bvhAabbMax, true );
					v = unQuantize( vecIn );
					m_bvhAabbMax.setMax( v + clampValue );
				}
				aabbSize = m_bvhAabbMax - m_bvhAabbMin;
				m_bvhQuantization = btVector3( (double)( 65533.0 ), (double)( 65533.0 ), (double)( 65533.0 ) ) / aabbSize;
			}
		}




		~btQuantizedBvh()
		{
		}

#if DEBUG_TREE_BUILDING
int gStackDepth = 0;
int gMaxStackDepth = 0;
#endif //DEBUG_TREE_BUILDING

		void buildTree( int startIndex, int endIndex )
		{
#if DEBUG_TREE_BUILDING
	gStackDepth++;
	if (gStackDepth > gMaxStackDepth)
		gMaxStackDepth = gStackDepth;
#endif //DEBUG_TREE_BUILDING


			int splitAxis, splitIndex, i;
			int numIndices = endIndex - startIndex;
			int curIndex = m_curNodeIndex;

			Debug.Assert( numIndices > 0 );

			if( numIndices == 1 )
			{
#if DEBUG_TREE_BUILDING
		gStackDepth--;
#endif //DEBUG_TREE_BUILDING

				assignInternalNodeFromLeafNode( m_curNodeIndex, startIndex );

				m_curNodeIndex++;
				return;
			}
			//calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.

			splitAxis = calcSplittingAxis( startIndex, endIndex );

			splitIndex = sortAndCalcSplittingIndex( startIndex, endIndex, splitAxis );

			int internalNodeIndex = m_curNodeIndex;

			//set the min aabb to 'inf' or a max value, and set the max aabb to a -inf/minimum value.
			//the aabb will be expanded during buildTree/mergeInternalNodeAabb with actual node values
			setInternalNodeAabbMin( m_curNodeIndex, m_bvhAabbMax );//can't use btVector3(SIMD_INFINITY,SIMD_INFINITY,SIMD_INFINITY)) because of quantization
			setInternalNodeAabbMax( m_curNodeIndex, m_bvhAabbMin );//can't use btVector3(-SIMD_INFINITY,-SIMD_INFINITY,-SIMD_INFINITY)) because of quantization


			for( i = startIndex; i < endIndex; i++ )
			{
				mergeInternalNodeAabb( m_curNodeIndex, getAabbMin( i ), getAabbMax( i ) );
			}

			m_curNodeIndex++;


			//internalNode.m_escapeIndex;

			int leftChildNodexIndex = m_curNodeIndex;

			//build left child tree
			buildTree( startIndex, splitIndex );

			int rightChildNodexIndex = m_curNodeIndex;
			//build right child tree
			buildTree( splitIndex, endIndex );

#if DEBUG_TREE_BUILDING
	gStackDepth--;
#endif //DEBUG_TREE_BUILDING

			int escapeIndex = m_curNodeIndex - curIndex;

			if( m_useQuantization )
			{
				//escapeIndex is the number of nodes of this subtree
				int sizeQuantizedNode = sizeof( btQuantizedBvhNode );
				int treeSizeInBytes = escapeIndex * sizeQuantizedNode;
				if( treeSizeInBytes > MAX_SUBTREE_SIZE_IN_BYTES )
				{
					updateSubtreeHeaders( leftChildNodexIndex, rightChildNodexIndex );
				}
			}
			else
			{

			}

			setInternalNodeEscapeIndex( internalNodeIndex, escapeIndex );

		}

		void updateSubtreeHeaders( int leftChildNodexIndex, int rightChildNodexIndex )
		{
			Debug.Assert( m_useQuantization );

			btQuantizedBvhNode & leftChildNode = m_quantizedContiguousNodes[leftChildNodexIndex];
			int leftSubTreeSize = leftChildNode.isLeafNode() ? 1 : leftChildNode.getEscapeIndex();
			int leftSubTreeSizeInBytes = leftSubTreeSize * static_cast<int>( sizeof( btQuantizedBvhNode ) );

			btQuantizedBvhNode & rightChildNode = m_quantizedContiguousNodes[rightChildNodexIndex];
			int rightSubTreeSize = rightChildNode.isLeafNode() ? 1 : rightChildNode.getEscapeIndex();
			int rightSubTreeSizeInBytes = rightSubTreeSize * static_cast<int>( sizeof( btQuantizedBvhNode ) );

			if( leftSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES )
			{
				btBvhSubtreeInfo & subtree = m_SubtreeHeaders.expand();
				subtree.setAabbFromQuantizeNode( leftChildNode );
				subtree.m_rootNodeIndex = leftChildNodexIndex;
				subtree.m_subtreeSize = leftSubTreeSize;
			}

			if( rightSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES )
			{
				btBvhSubtreeInfo & subtree = m_SubtreeHeaders.expand();
				subtree.setAabbFromQuantizeNode( rightChildNode );
				subtree.m_rootNodeIndex = rightChildNodexIndex;
				subtree.m_subtreeSize = rightSubTreeSize;
			}

			//PCK: update the copy of the size
			m_subtreeHeaderCount = m_SubtreeHeaders.Count;
		}


		int sortAndCalcSplittingIndex( int startIndex, int endIndex, int splitAxis )
		{
			int i;
			int splitIndex = startIndex;
			int numIndices = endIndex - startIndex;
			double splitValue;

			btVector3 means= btVector3.Zero;
			for( i = startIndex; i < endIndex; i++ )
			{
				btVector3 center = (double)( 0.5 ) * ( getAabbMax( i ) + getAabbMin( i ) );
				means += center;
			}
			means *= ( btScalar.BT_ONE / (double)numIndices );

			splitValue = means[splitAxis];

			//sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
			for( i = startIndex; i < endIndex; i++ )
			{
				btVector3 center = (double)( 0.5 ) * ( getAabbMax( i ) + getAabbMin( i ) );
				if( center[splitAxis] > splitValue )
				{
					//swap
					swapLeafNodes( i, splitIndex );
					splitIndex++;
				}
			}

			//if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
			//otherwise the tree-building might fail due to stack-overflows in certain cases.
			//unbalanced1 is unsafe: it can cause stack overflows
			//bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

			//unbalanced2 should work too: always use center (perfect balanced trees)	
			//bool unbalanced2 = true;

			//this should be safe too:
			int rangeBalancedIndices = numIndices / 3;
			bool unbalanced = ( ( splitIndex <= ( startIndex + rangeBalancedIndices ) ) || ( splitIndex >= ( endIndex - 1 - rangeBalancedIndices ) ) );

			if( unbalanced )
			{
				splitIndex = startIndex + ( numIndices >> 1 );
			}

			bool unbal = ( splitIndex == startIndex ) || ( splitIndex == ( endIndex ) );
			//(void)unbal;
			Debug.Assert( !unbal );

			return splitIndex;
		}


		int calcSplittingAxis( int startIndex, int endIndex )
		{
			int i;

			btVector3 means = btVector3.Zero;
			btVector3 variance = btVector3.Zero;
			int numIndices = endIndex - startIndex;

			for( i = startIndex; i < endIndex; i++ )
			{
				btVector3 center = (double)( 0.5 ) * ( getAabbMax( i ) + getAabbMin( i ) );
				means += center;
			}
			means *= ( btScalar.BT_ONE / (double)numIndices );

			for( i = startIndex; i < endIndex; i++ )
			{
				btVector3 center = (double)( 0.5 ) * ( getAabbMax( i ) + getAabbMin( i ) );
				btVector3 diff2 = center - means;
				diff2 = diff2 * diff2;
				variance += diff2;
			}
			variance *= ( btScalar.BT_ONE / ( (double)numIndices - 1 ) );

			return variance.maxAxis();
		}



		void reportAabbOverlappingNodex( btNodeOverlapCallback* nodeCallback, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			//either choose recursive traversal (walkTree) or stackless (walkStacklessTree)

			if( m_useQuantization )
			{
				///quantize query AABB
				ushort[] quantizedQueryAabbMin = new ushort[3];
				ushort[] quantizedQueryAabbMax = new ushort[3];
				quantizeWithClamp( quantizedQueryAabbMin, aabbMin, 0 );
				quantizeWithClamp( quantizedQueryAabbMax, aabbMax, 1 );

				switch( m_traversalMode )
				{
					case TRAVERSAL_STACKLESS:
						walkStacklessQuantizedTree( nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax, 0, m_curNodeIndex );
						break;
					case TRAVERSAL_STACKLESS_CACHE_FRIENDLY:
						walkStacklessQuantizedTreeCacheFriendly( nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax );
						break;
					case TRAVERSAL_RECURSIVE:
						{
							btQuantizedBvhNode* rootNode = m_quantizedContiguousNodes;
							walkRecursiveQuantizedTreeAgainstQueryAabb( rootNode, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax );
						}
						break;
					default:
						//unsupported
						Debug.Assert( false );
				}
			}
			else
			{
				walkStacklessTree( nodeCallback, aabbMin, aabbMax );
			}
		}


		int maxIterations = 0;


		void walkStacklessTree( btNodeOverlapCallback* nodeCallback, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			Debug.Assert( !m_useQuantization );

			btOptimizedBvhNode* rootNode = m_contiguousNodes;
			int escapeIndex, curIndex = 0;
			int walkIterations = 0;
			bool isLeafNode;
			//PCK: unsigned instead of bool
			unsigned aabbOverlap;

			while( curIndex < m_curNodeIndex )
			{
				//catch bugs in tree data
				Debug.Assert( walkIterations < m_curNodeIndex );

				walkIterations++;
				aabbOverlap = TestAabbAgainstAabb2( aabbMin, aabbMax, rootNode.m_aabbMinOrg, rootNode.m_aabbMaxOrg );
				isLeafNode = rootNode.m_escapeIndex == -1;

				//PCK: unsigned instead of bool
				if( isLeafNode & ( aabbOverlap != 0 ) )
				{
					nodeCallback.processNode( rootNode.m_subPart, rootNode.m_triangleIndex );
				}

				//PCK: unsigned instead of bool
				if( ( aabbOverlap != 0 ) || isLeafNode )
				{
					rootNode++;
					curIndex++;
				}
				else
				{
					escapeIndex = rootNode.m_escapeIndex;
					rootNode += escapeIndex;
					curIndex += escapeIndex;
				}
			}
			if( maxIterations < walkIterations )
				maxIterations = walkIterations;

		}

		/*
		///this was the original recursive traversal, before we optimized towards stackless traversal
		void	walkTree(btOptimizedBvhNode* rootNode,btNodeOverlapCallback* nodeCallback,ref btVector3 aabbMin,ref btVector3 aabbMax)
		{
			bool isLeafNode, aabbOverlap = TestAabbAgainstAabb2(aabbMin,aabbMax,rootNode.m_aabbMin,rootNode.m_aabbMax);
			if (aabbOverlap)
			{
				isLeafNode = (!rootNode.m_leftChild && !rootNode.m_rightChild);
				if (isLeafNode)
				{
					nodeCallback.processNode(rootNode);
				} else
				{
					walkTree(rootNode.m_leftChild,nodeCallback,aabbMin,aabbMax);
					walkTree(rootNode.m_rightChild,nodeCallback,aabbMin,aabbMax);
				}
			}

		}
		*/

		void walkRecursiveQuantizedTreeAgainstQueryAabb( btQuantizedBvhNode currentNode, btNodeOverlapCallback nodeCallback, ushort[] quantizedQueryAabbMin, ushort[] quantizedQueryAabbMax )
		{
			Debug.Assert( m_useQuantization );

			bool isLeafNode;
			//PCK: unsigned instead of bool
			unsigned aabbOverlap;

			//PCK: unsigned instead of bool
			aabbOverlap = testQuantizedAabbAgainstQuantizedAabb( quantizedQueryAabbMin, quantizedQueryAabbMax, currentNode.m_quantizedAabbMin, currentNode.m_quantizedAabbMax );
			isLeafNode = currentNode.isLeafNode();

			//PCK: unsigned instead of bool
			if( aabbOverlap != 0 )
			{
				if( isLeafNode )
				{
					nodeCallback.processNode( currentNode.getPartId(), currentNode.getTriangleIndex() );
				}
				else
				{
					//process left and right children
					btQuantizedBvhNode leftChildNode = currentNode + 1;
					walkRecursiveQuantizedTreeAgainstQueryAabb( leftChildNode, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax );

					btQuantizedBvhNode rightChildNode = leftChildNode.isLeafNode() ? leftChildNode + 1 : leftChildNode + leftChildNode.getEscapeIndex();
					walkRecursiveQuantizedTreeAgainstQueryAabb( rightChildNode, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax );
				}
			}
		}



		void walkStacklessTreeAgainstRay( btNodeOverlapCallback* nodeCallback, ref btVector3 raySource, ref btVector3 rayTarget, ref btVector3 aabbMin, ref btVector3 aabbMax, int startNodeIndex, int endNodeIndex )
		{
			Debug.Assert( !m_useQuantization );

			btOptimizedBvhNode* rootNode = &m_contiguousNodes;
			int escapeIndex, curIndex = 0;
			int walkIterations = 0;
			bool isLeafNode;
			//PCK: unsigned instead of bool
			unsigned aabbOverlap = 0;
			unsigned rayBoxOverlap = 0;
			double lambda_max = 1.0;

			/* Quick pruning by quantized box */
			btVector3 rayAabbMin = raySource;
			btVector3 rayAabbMax = raySource;
			rayAabbMin.setMin( rayTarget );
			rayAabbMax.setMax( rayTarget );

			/* Add box cast extents to bounding box */
			rayAabbMin += aabbMin;
			rayAabbMax += aabbMax;

#if RAYAABB2
	btVector3 rayDir = (rayTarget-raySource);
	rayDir.normalize ();
	lambda_max = rayDir.dot(rayTarget-raySource);
	///what about division by zero? -. just set rayDirection[i] to 1.0
	btVector3 rayDirectionInverse;
	rayDirectionInverse[0] = rayDir[0] == (double)(0.0) ? (double)(BT_LARGE_FLOAT) : (double)(1.0) / rayDir[0];
	rayDirectionInverse[1] = rayDir[1] == (double)(0.0) ? (double)(BT_LARGE_FLOAT) : (double)(1.0) / rayDir[1];
	rayDirectionInverse[2] = rayDir[2] == (double)(0.0) ? (double)(BT_LARGE_FLOAT) : (double)(1.0) / rayDir[2];
	uint sign[3] = { rayDirectionInverse[0] < 0.0, rayDirectionInverse[1] < 0.0, rayDirectionInverse[2] < 0.0};
#endif

			btVector3[] bounds = new btVector3[2];

			while( curIndex < m_curNodeIndex )
			{
				double param = 1.0;
				//catch bugs in tree data
				Debug.Assert( walkIterations < m_curNodeIndex );

				walkIterations++;

				bounds[0] = rootNode.m_aabbMinOrg;
				bounds[1] = rootNode.m_aabbMaxOrg;
				/* Add box cast extents */
				bounds[0] -= aabbMax;
				bounds[1] -= aabbMin;

				aabbOverlap = TestAabbAgainstAabb2( rayAabbMin, rayAabbMax, rootNode.m_aabbMinOrg, rootNode.m_aabbMaxOrg );
				//perhaps profile if it is worth doing the aabbOverlap test first

#if RAYAABB2
			///careful with this check: need to check division by zero (above) and fix the unQuantize method
			///thanks Joerg/hiker for the reproduction case!
			///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9t=1858
		rayBoxOverlap = aabbOverlap ? btRayAabb2 (raySource, rayDirectionInverse, sign, bounds, param, 0.0f, lambda_max) : false;

#else
				btVector3 normal;
				rayBoxOverlap = btRayAabb( raySource, rayTarget, bounds, bounds[1], param, normal );
#endif

				isLeafNode = rootNode.m_escapeIndex == -1;

				//PCK: unsigned instead of bool
				if( isLeafNode && ( rayBoxOverlap != 0 ) )
				{
					nodeCallback.processNode( rootNode.m_subPart, rootNode.m_triangleIndex );
				}

				//PCK: unsigned instead of bool
				if( ( rayBoxOverlap != 0 ) || isLeafNode )
				{
					rootNode++;
					curIndex++;
				}
				else
				{
					escapeIndex = rootNode.m_escapeIndex;
					rootNode += escapeIndex;
					curIndex += escapeIndex;
				}
			}
			if( maxIterations < walkIterations )
				maxIterations = walkIterations;

		}



		void walkStacklessQuantizedTreeAgainstRay( btNodeOverlapCallback* nodeCallback, ref btVector3 raySource, ref btVector3 rayTarget, ref btVector3 aabbMin, ref btVector3 aabbMax, int startNodeIndex, int endNodeIndex )
		{
			Debug.Assert( m_useQuantization );

			int curIndex = startNodeIndex;
			int walkIterations = 0;
			int subTreeSize = endNodeIndex - startNodeIndex;
			//(void)subTreeSize;

			btQuantizedBvhNode* rootNode = &m_quantizedContiguousNodes[startNodeIndex];
			int escapeIndex;

			bool isLeafNode;
			//PCK: unsigned instead of bool
			unsigned boxBoxOverlap = 0;
			unsigned rayBoxOverlap = 0;

			double lambda_max = 1.0;

#if RAYAABB2
	btVector3 rayDirection = (rayTarget-raySource);
	rayDirection.normalize ();
	lambda_max = rayDirection.dot(rayTarget-raySource);
	///what about division by zero? -. just set rayDirection[i] to 1.0
	rayDirection[0] = rayDirection[0] == (double)(0.0) ? (double)(BT_LARGE_FLOAT) : (double)(1.0) / rayDirection[0];
	rayDirection[1] = rayDirection[1] == (double)(0.0) ? (double)(BT_LARGE_FLOAT) : (double)(1.0) / rayDirection[1];
	rayDirection[2] = rayDirection[2] == (double)(0.0) ? (double)(BT_LARGE_FLOAT) : (double)(1.0) / rayDirection[2];
	uint sign[3] = { rayDirection[0] < 0.0, rayDirection[1] < 0.0, rayDirection[2] < 0.0};
#endif

			/* Quick pruning by quantized box */
			btVector3 rayAabbMin = raySource;
			btVector3 rayAabbMax = raySource;
			rayAabbMin.setMin( rayTarget );
			rayAabbMax.setMax( rayTarget );

			/* Add box cast extents to bounding box */
			rayAabbMin += aabbMin;
			rayAabbMax += aabbMax;

			ushort[] quantizedQueryAabbMin = new ushort[3];
			ushort[] quantizedQueryAabbMax = new ushort[3];
			quantizeWithClamp( quantizedQueryAabbMin, rayAabbMin, 0 );
			quantizeWithClamp( quantizedQueryAabbMax, rayAabbMax, 1 );

			while( curIndex < endNodeIndex )
			{

				//#define VISUALLY_ANALYZE_BVH 1
#if VISUALLY_ANALYZE_BVH
		//some code snippet to debugDraw aabb, to visually analyze bvh structure
		static int drawPatch = 0;
		//need some global access to a debugDrawer
		extern btIDebugDraw* debugDrawerPtr;
		if (curIndex==drawPatch)
		{
			btVector3 aabbMin,aabbMax;
			aabbMin = unQuantize(rootNode.m_quantizedAabbMin);
			aabbMax = unQuantize(rootNode.m_quantizedAabbMax);
			btVector3	color(1,0,0);
			debugDrawerPtr.drawAabb(aabbMin,aabbMax,color);
		}
#endif//VISUALLY_ANALYZE_BVH

				//catch bugs in tree data
				Debug.Assert( walkIterations < subTreeSize );

				walkIterations++;
				//PCK: unsigned instead of bool
				// only interested if this is closer than any previous hit
				double param = 1.0;
				rayBoxOverlap = 0;
				boxBoxOverlap = testQuantizedAabbAgainstQuantizedAabb( quantizedQueryAabbMin, quantizedQueryAabbMax, rootNode.m_quantizedAabbMin, rootNode.m_quantizedAabbMax );
				isLeafNode = rootNode.isLeafNode();
				if( boxBoxOverlap )
				{
					btVector3[] bounds = btVector3[2];
					bounds[0] = unQuantize( rootNode.m_quantizedAabbMin );
					bounds[1] = unQuantize( rootNode.m_quantizedAabbMax );
					/* Add box cast extents */
					bounds[0] -= aabbMax;
					bounds[1] -= aabbMin;
					btVector3 normal;
#if false
			bool ra2 = btRayAabb2 (raySource, rayDirection, sign, bounds, param, 0.0, lambda_max);
			bool ra = btRayAabb (raySource, rayTarget, bounds[0], bounds[1], param, normal);
			if (ra2 != ra)
			{
				Console.WriteLine("functions don't match\n");
			}
#endif
#if RAYAABB2
			///careful with this check: need to check division by zero (above) and fix the unQuantize method
			///thanks Joerg/hiker for the reproduction case!
			///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9t=1858

			//CProfileSample sample = new CProfileSample("btRayAabb2");
			rayBoxOverlap = btRayAabb2 (raySource, rayDirection, sign, bounds, param, 0.0f, lambda_max);
			
#else
					rayBoxOverlap = true;//btRayAabb(raySource, rayTarget, bounds, bounds[1], param, normal);
#endif
				}

				if( isLeafNode && rayBoxOverlap )
				{
					nodeCallback.processNode( rootNode.getPartId(), rootNode.getTriangleIndex() );
				}

				//PCK: unsigned instead of bool
				if( ( rayBoxOverlap != 0 ) || isLeafNode )
				{
					rootNode++;
					curIndex++;
				}
				else
				{
					escapeIndex = rootNode.getEscapeIndex();
					rootNode += escapeIndex;
					curIndex += escapeIndex;
				}
			}
			if( maxIterations < walkIterations )
				maxIterations = walkIterations;

		}

		void walkStacklessQuantizedTree( btNodeOverlapCallback nodeCallback
			, ushort[] quantizedQueryAabbMin, ushort[] quantizedQueryAabbMax
			, int startNodeIndex, int endNodeIndex )
		{
			Debug.Assert( m_useQuantization );

			int curIndex = startNodeIndex;
			int walkIterations = 0;
			int subTreeSize = endNodeIndex - startNodeIndex;
			//(void)subTreeSize;

			btQuantizedBvhNode rootNode = m_quantizedContiguousNodes[startNodeIndex];
			int escapeIndex;

			bool isLeafNode;
			//PCK: unsigned instead of bool
			unsigned aabbOverlap;

			while( curIndex < endNodeIndex )
			{

				//#define VISUALLY_ANALYZE_BVH 1
#if VISUALLY_ANALYZE_BVH
		//some code snippet to debugDraw aabb, to visually analyze bvh structure
		static int drawPatch = 0;
		//need some global access to a debugDrawer
		extern btIDebugDraw* debugDrawerPtr;
		if (curIndex==drawPatch)
		{
			btVector3 aabbMin,aabbMax;
			aabbMin = unQuantize(rootNode.m_quantizedAabbMin);
			aabbMax = unQuantize(rootNode.m_quantizedAabbMax);
			btVector3	color(1,0,0);
			debugDrawerPtr.drawAabb(aabbMin,aabbMax,color);
		}
#endif//VISUALLY_ANALYZE_BVH

				//catch bugs in tree data
				Debug.Assert( walkIterations < subTreeSize );

				walkIterations++;
				//PCK: unsigned instead of bool
				aabbOverlap = testQuantizedAabbAgainstQuantizedAabb( quantizedQueryAabbMin, quantizedQueryAabbMax, rootNode.m_quantizedAabbMin, rootNode.m_quantizedAabbMax );
				isLeafNode = rootNode.isLeafNode();

				if( isLeafNode && aabbOverlap )
				{
					nodeCallback.processNode( rootNode.getPartId(), rootNode.getTriangleIndex() );
				}

				//PCK: unsigned instead of bool
				if( ( aabbOverlap != 0 ) || isLeafNode )
				{
					rootNode++;
					curIndex++;
				}
				else
				{
					escapeIndex = rootNode.getEscapeIndex();
					rootNode += escapeIndex;
					curIndex += escapeIndex;
				}
			}
			if( maxIterations < walkIterations )
				maxIterations = walkIterations;

		}

		//This traversal can be called from Playstation 3 SPU
		void walkStacklessQuantizedTreeCacheFriendly( btNodeOverlapCallback nodeCallback, ushort[] quantizedQueryAabbMin, ushort[] quantizedQueryAabbMax )
		{
			Debug.Assert( m_useQuantization );

			int i;


			for( i = 0; i < this.m_SubtreeHeaders.Count; i++ )
			{
				btBvhSubtreeInfo & subtree = m_SubtreeHeaders[i];

				//PCK: unsigned instead of bool
				unsigned overlap = testQuantizedAabbAgainstQuantizedAabb( quantizedQueryAabbMin, quantizedQueryAabbMax, subtree.m_quantizedAabbMin, subtree.m_quantizedAabbMax );
				if( overlap != 0 )
				{
					walkStacklessQuantizedTree( nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax,
						subtree.m_rootNodeIndex,
						subtree.m_rootNodeIndex + subtree.m_subtreeSize );
				}
			}
		}


		void reportRayOverlappingNodex( btNodeOverlapCallback* nodeCallback, ref btVector3 raySource, ref btVector3 rayTarget )
		{
			reportBoxCastOverlappingNodex( nodeCallback, raySource, rayTarget, btVector3( 0, 0, 0 ), btVector3( 0, 0, 0 ) );
		}


		void reportBoxCastOverlappingNodex( btNodeOverlapCallback* nodeCallback, ref btVector3 raySource, ref btVector3 rayTarget, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			//always use stackless

			if( m_useQuantization )
			{
				walkStacklessQuantizedTreeAgainstRay( nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, m_curNodeIndex );
			}
			else
			{
				walkStacklessTreeAgainstRay( nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, m_curNodeIndex );
			}
			/*
			{
				//recursive traversal
				btVector3 qaabbMin = raySource;
				btVector3 qaabbMax = raySource;
				qaabbMin.setMin(rayTarget);
				qaabbMax.setMax(rayTarget);
				qaabbMin += aabbMin;
				qaabbMax += aabbMax;
				reportAabbOverlappingNodex(nodeCallback,qaabbMin,qaabbMax);
			}
			*/

		}


		void swapLeafNodes( int i, int splitIndex )
		{
			if( m_useQuantization )
			{
				btQuantizedBvhNode tmp = m_quantizedLeafNodes[i];
				m_quantizedLeafNodes[i] = m_quantizedLeafNodes[splitIndex];
				m_quantizedLeafNodes[splitIndex] = tmp;
			}
			else
			{
				btOptimizedBvhNode tmp = m_leafNodes[i];
				m_leafNodes[i] = m_leafNodes[splitIndex];
				m_leafNodes[splitIndex] = tmp;
			}
		}

		void assignInternalNodeFromLeafNode( int internalNode, int leafNodeIndex )
		{
			if( m_useQuantization )
			{
				m_quantizedContiguousNodes[internalNode] = m_quantizedLeafNodes[leafNodeIndex];
			}
			else
			{
				m_contiguousNodes[internalNode] = m_leafNodes[leafNodeIndex];
			}
		}
		// Constructor that prevents btVector3's default constructor from being called
		// Special "copy" constructor that allows for in-place deserialization
		// Prevents btVector3's default constructor from being called, but doesn't inialize much else
		// ownsMemory should most likely be false if deserializing, and if you are not, don't call this (it also changes the function signature, which we need)
		internal btQuantizedBvh( btQuantizedBvh self, bool ownsMemory )
		{
			m_bvhAabbMin = ( self.m_bvhAabbMin );
			m_bvhAabbMax = ( self.m_bvhAabbMax );
			m_bvhQuantization = ( self.m_bvhQuantization );
			m_bulletVersion( BT_BULLET_VERSION );

		}

		//PCK: include

#if false
//# include <new>
//PCK: consts
static string nsigned BVH_ALIGNMENT = 16;
static string nsigned BVH_ALIGNMENT_MASK = BVH_ALIGNMENT-1;

static string nsigned BVH_ALIGNMENT_BLOCKS = 2;
#endif

#if SERLIALIZE_DONE
#if BT_USE_DOUBLE_PRECISION
//#define btQuantizedBvhData btQuantizedBvhDoubleData
//#define btOptimizedBvhNodeData btOptimizedBvhNodeDoubleData
//#define btQuantizedBvhDataName "btQuantizedBvhDoubleData"
#else
//#define btQuantizedBvhData btQuantizedBvhFloatData
//#define btOptimizedBvhNodeData btOptimizedBvhNodeFloatData
//#define btQuantizedBvhDataName "btQuantizedBvhFloatData"
#endif

		unsigned calculateSerializeBufferSize();

		/// Data buffer MUST be 16 byte aligned
		virtual bool serialize( object o_alignedDataBuffer, unsigned i_dataBufferSize, bool i_swapEndian );

		///deSerializeInPlace loads and initializes a BVH from a buffer in memory 'in place'
		static btQuantizedBvh* deSerializeInPlace( object i_alignedDataBuffer, uint i_dataBufferSize, bool i_swapEndian );

		static uint getAlignmentSerializationPadding();
		//////////////////////////////////////////////////////////////////////


		virtual int calculateSerializeBufferSizeNew();

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual string serialize( object dataBuffer, btSerializer* serializer );

		virtual void deSerializeFloat(struct btQuantizedBvhFloatData& quantizedBvhFloatData);

	virtual void deSerializeDouble(struct btQuantizedBvhDoubleData& quantizedBvhDoubleData);


uint getAlignmentSerializationPadding()
{
	// I changed this to 0 since the extra padding is not needed or used.
	return 0;//BVH_ALIGNMENT_BLOCKS * BVH_ALIGNMENT;
}

unsigned calculateSerializeBufferSize()
{
	unsigned baseSize = sizeof( btQuantizedBvh ) + getAlignmentSerializationPadding();
	baseSize += sizeof( btBvhSubtreeInfo ) * m_subtreeHeaderCount;
	if( m_useQuantization )
	{
		return baseSize + m_curNodeIndex * sizeof( btQuantizedBvhNode );
	}
	return baseSize + m_curNodeIndex * sizeof( btOptimizedBvhNode );
}

bool serialize( object o_alignedDataBuffer, unsigned /*i_dataBufferSize */, bool i_swapEndian )
{
	Debug.Assert( m_subtreeHeaderCount == m_SubtreeHeaders.Count );
	m_subtreeHeaderCount = m_SubtreeHeaders.Count;

	/*	if (i_dataBufferSize < calculateSerializeBufferSize() || o_alignedDataBuffer == NULL || (((unsigned)o_alignedDataBuffer & BVH_ALIGNMENT_MASK) != 0))
		{
			///check alignedment for buffer?
			Debug.Assert(false);
			return false;
		}
	*/

	btQuantizedBvh* targetBvh = (btQuantizedBvh*)o_alignedDataBuffer;

	// construct the class so the virtual function table, etc will be set up
	// Also, m_leafNodes and m_quantizedLeafNodes will be initialized to default values by the constructor
	new ( targetBvh ) btQuantizedBvh;

	if( i_swapEndian )
	{
		targetBvh.m_curNodeIndex = static_cast<int>( btSwapEndian( m_curNodeIndex ) );


		btSwapVector3Endian( m_bvhAabbMin, targetBvh.m_bvhAabbMin );
		btSwapVector3Endian( m_bvhAabbMax, targetBvh.m_bvhAabbMax );
		btSwapVector3Endian( m_bvhQuantization, targetBvh.m_bvhQuantization );

		targetBvh.m_traversalMode = (btTraversalMode)btSwapEndian( m_traversalMode );
		targetBvh.m_subtreeHeaderCount = static_cast<int>( btSwapEndian( m_subtreeHeaderCount ) );
	}
	else
	{
		targetBvh.m_curNodeIndex = m_curNodeIndex;
		targetBvh.m_bvhAabbMin = m_bvhAabbMin;
		targetBvh.m_bvhAabbMax = m_bvhAabbMax;
		targetBvh.m_bvhQuantization = m_bvhQuantization;
		targetBvh.m_traversalMode = m_traversalMode;
		targetBvh.m_subtreeHeaderCount = m_subtreeHeaderCount;
	}

	targetBvh.m_useQuantization = m_useQuantization;

	byte* nodeData = (byte*)targetBvh;
	nodeData += sizeof( btQuantizedBvh );

	unsigned sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;

	int nodeCount = m_curNodeIndex;

	if( m_useQuantization )
	{
		targetBvh.m_quantizedContiguousNodes.initializeFromBuffer( nodeData, nodeCount, nodeCount );

		if( i_swapEndian )
		{
			for( int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++ )
			{
				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] = btSwapEndian( m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] );
				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] = btSwapEndian( m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] );
				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] = btSwapEndian( m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] );

				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] = btSwapEndian( m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] );
				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] = btSwapEndian( m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] );
				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] = btSwapEndian( m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] );

				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = static_cast<int>( btSwapEndian( m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex ) );
			}
		}
		else
		{
			for( int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++ )
			{

				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0];
				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1];
				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2];

				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0];
				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1];
				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2];

				targetBvh.m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex;


			}
		}
		nodeData += sizeof( btQuantizedBvhNode ) * nodeCount;

		// this clears the pointer in the member variable it doesn't really do anything to the data
		// it does call the destructor on the contained objects, but they are all classes with no destructor defined
		// so the memory (which is not freed) is left alone
		targetBvh.m_quantizedContiguousNodes.initializeFromBuffer( NULL, 0, 0 );
	}
	else
	{
		targetBvh.m_contiguousNodes.initializeFromBuffer( nodeData, nodeCount, nodeCount );

		if( i_swapEndian )
		{
			for( int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++ )
			{
				btSwapVector3Endian( m_contiguousNodes[nodeIndex].m_aabbMinOrg, targetBvh.m_contiguousNodes[nodeIndex].m_aabbMinOrg );
				btSwapVector3Endian( m_contiguousNodes[nodeIndex].m_aabbMaxOrg, targetBvh.m_contiguousNodes[nodeIndex].m_aabbMaxOrg );

				targetBvh.m_contiguousNodes[nodeIndex].m_escapeIndex = static_cast<int>( btSwapEndian( m_contiguousNodes[nodeIndex].m_escapeIndex ) );
				targetBvh.m_contiguousNodes[nodeIndex].m_subPart = static_cast<int>( btSwapEndian( m_contiguousNodes[nodeIndex].m_subPart ) );
				targetBvh.m_contiguousNodes[nodeIndex].m_triangleIndex = static_cast<int>( btSwapEndian( m_contiguousNodes[nodeIndex].m_triangleIndex ) );
			}
		}
		else
		{
			for( int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++ )
			{
				targetBvh.m_contiguousNodes[nodeIndex].m_aabbMinOrg = m_contiguousNodes[nodeIndex].m_aabbMinOrg;
				targetBvh.m_contiguousNodes[nodeIndex].m_aabbMaxOrg = m_contiguousNodes[nodeIndex].m_aabbMaxOrg;

				targetBvh.m_contiguousNodes[nodeIndex].m_escapeIndex = m_contiguousNodes[nodeIndex].m_escapeIndex;
				targetBvh.m_contiguousNodes[nodeIndex].m_subPart = m_contiguousNodes[nodeIndex].m_subPart;
				targetBvh.m_contiguousNodes[nodeIndex].m_triangleIndex = m_contiguousNodes[nodeIndex].m_triangleIndex;
			}
		}
		nodeData += sizeof( btOptimizedBvhNode ) * nodeCount;

		// this clears the pointer in the member variable it doesn't really do anything to the data
		// it does call the destructor on the contained objects, but they are all classes with no destructor defined
		// so the memory (which is not freed) is left alone
		targetBvh.m_contiguousNodes.initializeFromBuffer( NULL, 0, 0 );
	}

	sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;

	// Now serialize the subtree headers
	targetBvh.m_SubtreeHeaders.initializeFromBuffer( nodeData, m_subtreeHeaderCount, m_subtreeHeaderCount );
	if( i_swapEndian )
	{
		for( int i = 0; i < m_subtreeHeaderCount; i++ )
		{
			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMin[0] = btSwapEndian( m_SubtreeHeaders[i].m_quantizedAabbMin[0] );
			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMin[1] = btSwapEndian( m_SubtreeHeaders[i].m_quantizedAabbMin[1] );
			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMin[2] = btSwapEndian( m_SubtreeHeaders[i].m_quantizedAabbMin[2] );

			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMax[0] = btSwapEndian( m_SubtreeHeaders[i].m_quantizedAabbMax[0] );
			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMax[1] = btSwapEndian( m_SubtreeHeaders[i].m_quantizedAabbMax[1] );
			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMax[2] = btSwapEndian( m_SubtreeHeaders[i].m_quantizedAabbMax[2] );

			targetBvh.m_SubtreeHeaders[i].m_rootNodeIndex = static_cast<int>( btSwapEndian( m_SubtreeHeaders[i].m_rootNodeIndex ) );
			targetBvh.m_SubtreeHeaders[i].m_subtreeSize = static_cast<int>( btSwapEndian( m_SubtreeHeaders[i].m_subtreeSize ) );
		}
	}
	else
	{
		for( int i = 0; i < m_subtreeHeaderCount; i++ )
		{
			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMin[0] = ( m_SubtreeHeaders[i].m_quantizedAabbMin[0] );
			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMin[1] = ( m_SubtreeHeaders[i].m_quantizedAabbMin[1] );
			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMin[2] = ( m_SubtreeHeaders[i].m_quantizedAabbMin[2] );

			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMax[0] = ( m_SubtreeHeaders[i].m_quantizedAabbMax[0] );
			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMax[1] = ( m_SubtreeHeaders[i].m_quantizedAabbMax[1] );
			targetBvh.m_SubtreeHeaders[i].m_quantizedAabbMax[2] = ( m_SubtreeHeaders[i].m_quantizedAabbMax[2] );

			targetBvh.m_SubtreeHeaders[i].m_rootNodeIndex = ( m_SubtreeHeaders[i].m_rootNodeIndex );
			targetBvh.m_SubtreeHeaders[i].m_subtreeSize = ( m_SubtreeHeaders[i].m_subtreeSize );

			// need to clear padding in destination buffer
			targetBvh.m_SubtreeHeaders[i].m_padding[0] = 0;
			targetBvh.m_SubtreeHeaders[i].m_padding[1] = 0;
			targetBvh.m_SubtreeHeaders[i].m_padding[2] = 0;
		}
	}
	nodeData += sizeof( btBvhSubtreeInfo ) * m_subtreeHeaderCount;

	// this clears the pointer in the member variable it doesn't really do anything to the data
	// it does call the destructor on the contained objects, but they are all classes with no destructor defined
	// so the memory (which is not freed) is left alone
	targetBvh.m_SubtreeHeaders.initializeFromBuffer( NULL, 0, 0 );

	// this wipes the virtual function table pointer at the start of the buffer for the class
	*( (object*)o_alignedDataBuffer ) = NULL;

	return true;
}

btQuantizedBvh* deSerializeInPlace( object i_alignedDataBuffer, uint i_dataBufferSize, bool i_swapEndian )
{

	if( i_alignedDataBuffer == NULL )// || (((unsigned)i_alignedDataBuffer & BVH_ALIGNMENT_MASK) != 0))
	{
		return NULL;
	}
	btQuantizedBvh* bvh = (btQuantizedBvh*)i_alignedDataBuffer;

	if( i_swapEndian )
	{
		bvh.m_curNodeIndex = static_cast<int>( btSwapEndian( bvh.m_curNodeIndex ) );

		btUnSwapVector3Endian( bvh.m_bvhAabbMin );
		btUnSwapVector3Endian( bvh.m_bvhAabbMax );
		btUnSwapVector3Endian( bvh.m_bvhQuantization );

		bvh.m_traversalMode = (btTraversalMode)btSwapEndian( bvh.m_traversalMode );
		bvh.m_subtreeHeaderCount = static_cast<int>( btSwapEndian( bvh.m_subtreeHeaderCount ) );
	}

	uint calculatedBufSize = bvh.calculateSerializeBufferSize();
	Debug.Assert( calculatedBufSize <= i_dataBufferSize );

	if( calculatedBufSize > i_dataBufferSize )
	{
		return NULL;
	}

	byte* nodeData = (byte*)bvh;
	nodeData += sizeof( btQuantizedBvh );

	unsigned sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;

	int nodeCount = bvh.m_curNodeIndex;

	// Must call placement new to fill in virtual function table, etc, but we don't want to overwrite most data, so call a special version of the constructor
	// Also, m_leafNodes and m_quantizedLeafNodes will be initialized to default values by the constructor
	new ( bvh ) btQuantizedBvh( *bvh, false );

	if( bvh.m_useQuantization )
	{
		bvh.m_quantizedContiguousNodes.initializeFromBuffer( nodeData, nodeCount, nodeCount );

		if( i_swapEndian )
		{
			for( int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++ )
			{
				bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] = btSwapEndian( bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] );
				bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] = btSwapEndian( bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] );
				bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] = btSwapEndian( bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] );

				bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] = btSwapEndian( bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] );
				bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] = btSwapEndian( bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] );
				bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] = btSwapEndian( bvh.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] );

				bvh.m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = static_cast<int>( btSwapEndian( bvh.m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex ) );
			}
		}
		nodeData += sizeof( btQuantizedBvhNode ) * nodeCount;
	}
	else
	{
		bvh.m_contiguousNodes.initializeFromBuffer( nodeData, nodeCount, nodeCount );

		if( i_swapEndian )
		{
			for( int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++ )
			{
				btUnSwapVector3Endian( bvh.m_contiguousNodes[nodeIndex].m_aabbMinOrg );
				btUnSwapVector3Endian( bvh.m_contiguousNodes[nodeIndex].m_aabbMaxOrg );

				bvh.m_contiguousNodes[nodeIndex].m_escapeIndex = static_cast<int>( btSwapEndian( bvh.m_contiguousNodes[nodeIndex].m_escapeIndex ) );
				bvh.m_contiguousNodes[nodeIndex].m_subPart = static_cast<int>( btSwapEndian( bvh.m_contiguousNodes[nodeIndex].m_subPart ) );
				bvh.m_contiguousNodes[nodeIndex].m_triangleIndex = static_cast<int>( btSwapEndian( bvh.m_contiguousNodes[nodeIndex].m_triangleIndex ) );
			}
		}
		nodeData += sizeof( btOptimizedBvhNode ) * nodeCount;
	}

	sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;

	// Now serialize the subtree headers
	bvh.m_SubtreeHeaders.initializeFromBuffer( nodeData, bvh.m_subtreeHeaderCount, bvh.m_subtreeHeaderCount );
	if( i_swapEndian )
	{
		for( int i = 0; i < bvh.m_subtreeHeaderCount; i++ )
		{
			bvh.m_SubtreeHeaders[i].m_quantizedAabbMin[0] = btSwapEndian( bvh.m_SubtreeHeaders[i].m_quantizedAabbMin[0] );
			bvh.m_SubtreeHeaders[i].m_quantizedAabbMin[1] = btSwapEndian( bvh.m_SubtreeHeaders[i].m_quantizedAabbMin[1] );
			bvh.m_SubtreeHeaders[i].m_quantizedAabbMin[2] = btSwapEndian( bvh.m_SubtreeHeaders[i].m_quantizedAabbMin[2] );

			bvh.m_SubtreeHeaders[i].m_quantizedAabbMax[0] = btSwapEndian( bvh.m_SubtreeHeaders[i].m_quantizedAabbMax[0] );
			bvh.m_SubtreeHeaders[i].m_quantizedAabbMax[1] = btSwapEndian( bvh.m_SubtreeHeaders[i].m_quantizedAabbMax[1] );
			bvh.m_SubtreeHeaders[i].m_quantizedAabbMax[2] = btSwapEndian( bvh.m_SubtreeHeaders[i].m_quantizedAabbMax[2] );

			bvh.m_SubtreeHeaders[i].m_rootNodeIndex = static_cast<int>( btSwapEndian( bvh.m_SubtreeHeaders[i].m_rootNodeIndex ) );
			bvh.m_SubtreeHeaders[i].m_subtreeSize = static_cast<int>( btSwapEndian( bvh.m_SubtreeHeaders[i].m_subtreeSize ) );
		}
	}

	return bvh;
}


		void deSerializeFloat(struct btQuantizedBvhFloatData& quantizedBvhFloatData)
{
	m_bvhAabbMax.deSerializeFloat(quantizedBvhFloatData.m_bvhAabbMax);
	m_bvhAabbMin.deSerializeFloat(quantizedBvhFloatData.m_bvhAabbMin);
	m_bvhQuantization.deSerializeFloat(quantizedBvhFloatData.m_bvhQuantization);

	m_curNodeIndex = quantizedBvhFloatData.m_curNodeIndex;
	m_useQuantization = quantizedBvhFloatData.m_useQuantization!=0;
	
	{
		int numElem = quantizedBvhFloatData.m_numContiguousLeafNodes;
m_contiguousNodes.resize(numElem);

		if (numElem)
		{
			btOptimizedBvhNodeFloatData* memPtr = quantizedBvhFloatData.m_contiguousNodesPtr;

			for (int i = 0; i<numElem;i++,memPtr++)
			{
				m_contiguousNodes[i].m_aabbMaxOrg.deSerializeFloat(memPtr.m_aabbMaxOrg);
				m_contiguousNodes[i].m_aabbMinOrg.deSerializeFloat(memPtr.m_aabbMinOrg);
				m_contiguousNodes[i].m_escapeIndex = memPtr.m_escapeIndex;
				m_contiguousNodes[i].m_subPart = memPtr.m_subPart;
				m_contiguousNodes[i].m_triangleIndex = memPtr.m_triangleIndex;
			}
		}
	}

	{
		int numElem = quantizedBvhFloatData.m_numQuantizedContiguousNodes;
m_quantizedContiguousNodes.resize(numElem);
		
		if (numElem)
		{
			btQuantizedBvhNodeData* memPtr = quantizedBvhFloatData.m_quantizedContiguousNodesPtr;
			for (int i = 0; i<numElem;i++,memPtr++)
			{
				m_quantizedContiguousNodes[i].m_escapeIndexOrTriangleIndex = memPtr.m_escapeIndexOrTriangleIndex;
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[0] = memPtr.m_quantizedAabbMax[0];
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[1] = memPtr.m_quantizedAabbMax[1];
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[2] = memPtr.m_quantizedAabbMax[2];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[0] = memPtr.m_quantizedAabbMin[0];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[1] = memPtr.m_quantizedAabbMin[1];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[2] = memPtr.m_quantizedAabbMin[2];
			}
		}
	}

	m_traversalMode = btTraversalMode( quantizedBvhFloatData.m_traversalMode);
	
	{
		int numElem = quantizedBvhFloatData.m_numSubtreeHeaders;
m_SubtreeHeaders.resize(numElem);
		if (numElem)
		{
			btBvhSubtreeInfoData* memPtr = quantizedBvhFloatData.m_subTreeInfoPtr;
			for (int i = 0; i<numElem;i++,memPtr++)
			{
				m_SubtreeHeaders[i].m_quantizedAabbMax[0] = memPtr.m_quantizedAabbMax[0] ;
				m_SubtreeHeaders[i].m_quantizedAabbMax[1] = memPtr.m_quantizedAabbMax[1];
				m_SubtreeHeaders[i].m_quantizedAabbMax[2] = memPtr.m_quantizedAabbMax[2];
				m_SubtreeHeaders[i].m_quantizedAabbMin[0] = memPtr.m_quantizedAabbMin[0];
				m_SubtreeHeaders[i].m_quantizedAabbMin[1] = memPtr.m_quantizedAabbMin[1];
				m_SubtreeHeaders[i].m_quantizedAabbMin[2] = memPtr.m_quantizedAabbMin[2];
				m_SubtreeHeaders[i].m_rootNodeIndex = memPtr.m_rootNodeIndex;
				m_SubtreeHeaders[i].m_subtreeSize = memPtr.m_subtreeSize;
			}
		}
	}
}

void deSerializeDouble(struct btQuantizedBvhDoubleData& quantizedBvhDoubleData)
{
	m_bvhAabbMax.deSerializeDouble(quantizedBvhDoubleData.m_bvhAabbMax);
	m_bvhAabbMin.deSerializeDouble(quantizedBvhDoubleData.m_bvhAabbMin);
	m_bvhQuantization.deSerializeDouble(quantizedBvhDoubleData.m_bvhQuantization);

	m_curNodeIndex = quantizedBvhDoubleData.m_curNodeIndex;
	m_useQuantization = quantizedBvhDoubleData.m_useQuantization!=0;
	
	{
		int numElem = quantizedBvhDoubleData.m_numContiguousLeafNodes;
m_contiguousNodes.resize(numElem);

		if (numElem)
		{
			btOptimizedBvhNodeDoubleData* memPtr = quantizedBvhDoubleData.m_contiguousNodesPtr;

			for (int i = 0; i<numElem;i++,memPtr++)
			{
				m_contiguousNodes[i].m_aabbMaxOrg.deSerializeDouble(memPtr.m_aabbMaxOrg);
				m_contiguousNodes[i].m_aabbMinOrg.deSerializeDouble(memPtr.m_aabbMinOrg);
				m_contiguousNodes[i].m_escapeIndex = memPtr.m_escapeIndex;
				m_contiguousNodes[i].m_subPart = memPtr.m_subPart;
				m_contiguousNodes[i].m_triangleIndex = memPtr.m_triangleIndex;
			}
		}
	}

	{
		int numElem = quantizedBvhDoubleData.m_numQuantizedContiguousNodes;
m_quantizedContiguousNodes.resize(numElem);
		
		if (numElem)
		{
			btQuantizedBvhNodeData* memPtr = quantizedBvhDoubleData.m_quantizedContiguousNodesPtr;
			for (int i = 0; i<numElem;i++,memPtr++)
			{
				m_quantizedContiguousNodes[i].m_escapeIndexOrTriangleIndex = memPtr.m_escapeIndexOrTriangleIndex;
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[0] = memPtr.m_quantizedAabbMax[0];
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[1] = memPtr.m_quantizedAabbMax[1];
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[2] = memPtr.m_quantizedAabbMax[2];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[0] = memPtr.m_quantizedAabbMin[0];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[1] = memPtr.m_quantizedAabbMin[1];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[2] = memPtr.m_quantizedAabbMin[2];
			}
		}
	}

	m_traversalMode = btTraversalMode( quantizedBvhDoubleData.m_traversalMode);
	
	{
		int numElem = quantizedBvhDoubleData.m_numSubtreeHeaders;
m_SubtreeHeaders.resize(numElem);
		if (numElem)
		{
			btBvhSubtreeInfoData* memPtr = quantizedBvhDoubleData.m_subTreeInfoPtr;
			for (int i = 0; i<numElem;i++,memPtr++)
			{
				m_SubtreeHeaders[i].m_quantizedAabbMax[0] = memPtr.m_quantizedAabbMax[0] ;
				m_SubtreeHeaders[i].m_quantizedAabbMax[1] = memPtr.m_quantizedAabbMax[1];
				m_SubtreeHeaders[i].m_quantizedAabbMax[2] = memPtr.m_quantizedAabbMax[2];
				m_SubtreeHeaders[i].m_quantizedAabbMin[0] = memPtr.m_quantizedAabbMin[0];
				m_SubtreeHeaders[i].m_quantizedAabbMin[1] = memPtr.m_quantizedAabbMin[1];
				m_SubtreeHeaders[i].m_quantizedAabbMin[2] = memPtr.m_quantizedAabbMin[2];
				m_SubtreeHeaders[i].m_rootNodeIndex = memPtr.m_rootNodeIndex;
				m_SubtreeHeaders[i].m_subtreeSize = memPtr.m_subtreeSize;
			}
		}
	}

}



///fills the dataBuffer and returns the struct name (and 0 on failure)
string serialize( object dataBuffer, btSerializer* serializer )
{
	btQuantizedBvhData* quantizedData = (btQuantizedBvhData*)dataBuffer;

	m_bvhAabbMax.serialize( quantizedData.m_bvhAabbMax );
	m_bvhAabbMin.serialize( quantizedData.m_bvhAabbMin );
	m_bvhQuantization.serialize( quantizedData.m_bvhQuantization );

	quantizedData.m_curNodeIndex = m_curNodeIndex;
	quantizedData.m_useQuantization = m_useQuantization;

	quantizedData.m_numContiguousLeafNodes = m_contiguousNodes.Count;
	quantizedData.m_contiguousNodesPtr = (btOptimizedBvhNodeData*)( m_contiguousNodes.Count ? serializer.getUniquePointer( (object)m_contiguousNodes ) : 0 );
	if( quantizedData.m_contiguousNodesPtr )
	{
		int sz = sizeof( btOptimizedBvhNodeData );
		int numElem = m_contiguousNodes.Count;
		btChunk* chunk = serializer.allocate( sz, numElem );
		btOptimizedBvhNodeData* memPtr = (btOptimizedBvhNodeData*)chunk.m_oldPtr;
		for( int i = 0; i < numElem; i++, memPtr++ )
		{
			m_contiguousNodes[i].m_aabbMaxOrg.serialize( memPtr.m_aabbMaxOrg );
			m_contiguousNodes[i].m_aabbMinOrg.serialize( memPtr.m_aabbMinOrg );
			memPtr.m_escapeIndex = m_contiguousNodes[i].m_escapeIndex;
			memPtr.m_subPart = m_contiguousNodes[i].m_subPart;
			memPtr.m_triangleIndex = m_contiguousNodes[i].m_triangleIndex;
		}
		serializer.finalizeChunk( chunk, "btOptimizedBvhNodeData", BT_ARRAY_CODE, (object)m_contiguousNodes );
	}

	quantizedData.m_numQuantizedContiguousNodes = m_quantizedContiguousNodes.Count;
	//	Console.WriteLine("quantizedData.m_numQuantizedContiguousNodes=%d\n",quantizedData.m_numQuantizedContiguousNodes);
	quantizedData.m_quantizedContiguousNodesPtr = (btQuantizedBvhNodeData*)( m_quantizedContiguousNodes.Count ? serializer.getUniquePointer( (object)m_quantizedContiguousNodes ) : 0 );
	if( quantizedData.m_quantizedContiguousNodesPtr )
	{
		int sz = sizeof( btQuantizedBvhNodeData );
		int numElem = m_quantizedContiguousNodes.Count;
		btChunk* chunk = serializer.allocate( sz, numElem );
		btQuantizedBvhNodeData* memPtr = (btQuantizedBvhNodeData*)chunk.m_oldPtr;
		for( int i = 0; i < numElem; i++, memPtr++ )
		{
			memPtr.m_escapeIndexOrTriangleIndex = m_quantizedContiguousNodes[i].m_escapeIndexOrTriangleIndex;
			memPtr.m_quantizedAabbMax[0] = m_quantizedContiguousNodes[i].m_quantizedAabbMax[0];
			memPtr.m_quantizedAabbMax[1] = m_quantizedContiguousNodes[i].m_quantizedAabbMax[1];
			memPtr.m_quantizedAabbMax[2] = m_quantizedContiguousNodes[i].m_quantizedAabbMax[2];
			memPtr.m_quantizedAabbMin[0] = m_quantizedContiguousNodes[i].m_quantizedAabbMin[0];
			memPtr.m_quantizedAabbMin[1] = m_quantizedContiguousNodes[i].m_quantizedAabbMin[1];
			memPtr.m_quantizedAabbMin[2] = m_quantizedContiguousNodes[i].m_quantizedAabbMin[2];
		}
		serializer.finalizeChunk( chunk, "btQuantizedBvhNodeData", BT_ARRAY_CODE, (object)m_quantizedContiguousNodes );
	}

	quantizedData.m_traversalMode = int( m_traversalMode );
	quantizedData.m_numSubtreeHeaders = m_SubtreeHeaders.Count;

	quantizedData.m_subTreeInfoPtr = (btBvhSubtreeInfoData*)( m_SubtreeHeaders.Count ? serializer.getUniquePointer( (object)m_SubtreeHeaders ) : 0 );
	if( quantizedData.m_subTreeInfoPtr )
	{
		int sz = sizeof( btBvhSubtreeInfoData );
		int numElem = m_SubtreeHeaders.Count;
		btChunk* chunk = serializer.allocate( sz, numElem );
		btBvhSubtreeInfoData* memPtr = (btBvhSubtreeInfoData*)chunk.m_oldPtr;
		for( int i = 0; i < numElem; i++, memPtr++ )
		{
			memPtr.m_quantizedAabbMax[0] = m_SubtreeHeaders[i].m_quantizedAabbMax[0];
			memPtr.m_quantizedAabbMax[1] = m_SubtreeHeaders[i].m_quantizedAabbMax[1];
			memPtr.m_quantizedAabbMax[2] = m_SubtreeHeaders[i].m_quantizedAabbMax[2];
			memPtr.m_quantizedAabbMin[0] = m_SubtreeHeaders[i].m_quantizedAabbMin[0];
			memPtr.m_quantizedAabbMin[1] = m_SubtreeHeaders[i].m_quantizedAabbMin[1];
			memPtr.m_quantizedAabbMin[2] = m_SubtreeHeaders[i].m_quantizedAabbMin[2];

			memPtr.m_rootNodeIndex = m_SubtreeHeaders[i].m_rootNodeIndex;
			memPtr.m_subtreeSize = m_SubtreeHeaders[i].m_subtreeSize;
		}
		serializer.finalizeChunk( chunk, "btBvhSubtreeInfoData", BT_ARRAY_CODE, (object)m_SubtreeHeaders );
	}
	return btQuantizedBvhDataName;
}

#endif




	}
;

#if SERIALIZE_DONE
struct btBvhSubtreeInfoData
{
	int m_rootNodeIndex;
	int m_subtreeSize;
	ushort m_quantizedAabbMin[3];
	ushort m_quantizedAabbMax[3];
};

struct btOptimizedBvhNodeFloatData
{
	btVector3FloatData m_aabbMinOrg;
	btVector3FloatData m_aabbMaxOrg;
	int m_escapeIndex;
	int m_subPart;
	int m_triangleIndex;
	char m_pad[4];
};

struct btOptimizedBvhNodeDoubleData
{
	btVector3DoubleData m_aabbMinOrg;
	btVector3DoubleData m_aabbMaxOrg;
	int m_escapeIndex;
	int m_subPart;
	int m_triangleIndex;
	char m_pad[4];
};


struct btQuantizedBvhNodeData
{
	ushort m_quantizedAabbMin[3];
	ushort m_quantizedAabbMax[3];
	int m_escapeIndexOrTriangleIndex;
};

struct btQuantizedBvhFloatData
{
	btVector3FloatData m_bvhAabbMin;
	btVector3FloatData m_bvhAabbMax;
	btVector3FloatData m_bvhQuantization;
	int m_curNodeIndex;
	int m_useQuantization;
	int m_numContiguousLeafNodes;
	int m_numQuantizedContiguousNodes;
	btOptimizedBvhNodeFloatData* m_contiguousNodesPtr;
	btQuantizedBvhNodeData* m_quantizedContiguousNodesPtr;
	btBvhSubtreeInfoData* m_subTreeInfoPtr;
	int m_traversalMode;
	int m_numSubtreeHeaders;

};

struct btQuantizedBvhDoubleData
{
	btVector3DoubleData m_bvhAabbMin;
	btVector3DoubleData m_bvhAabbMax;
	btVector3DoubleData m_bvhQuantization;
	int m_curNodeIndex;
	int m_useQuantization;
	int m_numContiguousLeafNodes;
	int m_numQuantizedContiguousNodes;
	btOptimizedBvhNodeDoubleData* m_contiguousNodesPtr;
	btQuantizedBvhNodeData* m_quantizedContiguousNodesPtr;

	int m_traversalMode;
	int m_numSubtreeHeaders;
	btBvhSubtreeInfoData* m_subTreeInfoPtr;
};


public int calculateSerializeBufferSizeNew()
{
	return sizeof( btQuantizedBvhData );
}

#endif



}