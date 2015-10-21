#if ! GIM_QUANTIZED_SET_H_INCLUDED
#define GIM_QUANTIZED_SET_H_INCLUDED

/*! \file btGImpactQuantizedBvh.h
\author Francisco Leon Najera
*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btGImpactBvh.h"
#include "btQuantization.h"





///btQuantizedBvhNode is a compressed aabb node, 16 bytes.
///Node can be used for leafnode or internal node. Leafnodes can point to 32-bit triangle index (non-negative range).
ATTRIBUTE_ALIGNED16	(struct) BT_QUANTIZED_BVH_NODE
{
	//12 bytes
	ushort int	m_quantizedAabbMin[3];
	ushort int	m_quantizedAabbMax[3];
	//4 bytes
	int	m_escapeIndexOrDataIndex;

	BT_QUANTIZED_BVH_NODE()
	{
		m_escapeIndexOrDataIndex = 0;
	}

	public bool isLeafNode()
	{
		//skipindex is negative (internal node), triangleindex >=0 (leafnode)
		return (m_escapeIndexOrDataIndex>=0);
	}

	public int getEscapeIndex()
	{
		//Debug.Assert(m_escapeIndexOrDataIndex < 0);
		return -m_escapeIndexOrDataIndex;
	}

	public void setEscapeIndex(int index)
	{
		m_escapeIndexOrDataIndex = -index;
	}

	public int getDataIndex()
	{
		//Debug.Assert(m_escapeIndexOrDataIndex >= 0);

		return m_escapeIndexOrDataIndex;
	}

	public void setDataIndex(int index)
	{
		m_escapeIndexOrDataIndex = index;
	}

	public bool testQuantizedBoxOverlapp(
		ushort * quantizedMin,ushort * quantizedMax)
	{
		if(m_quantizedAabbMin[0] > quantizedMax[0] ||
		   m_quantizedAabbMax[0] < quantizedMin[0] ||
		   m_quantizedAabbMin[1] > quantizedMax[1] ||
		   m_quantizedAabbMax[1] < quantizedMin[1] ||
		   m_quantizedAabbMin[2] > quantizedMax[2] ||
		   m_quantizedAabbMax[2] < quantizedMin[2])
		{
			return false;
		}
		return true;
	}

};



class GIM_QUANTIZED_BVH_NODE_ARRAY: btList<BT_QUANTIZED_BVH_NODE>
{
};




//! Basic Box tree structure
class btQuantizedBvhTree
{
protected:
	int m_num_nodes;
	GIM_QUANTIZED_BVH_NODE_ARRAY m_node_array;
	btAABB m_global_bound;
	btVector3 m_bvhQuantization;
protected:
	void calc_quantization(GIM_BVH_DATA_ARRAY & primitive_boxes, double boundMargin = (double)(1.0) );

	int _sort_and_calc_splitting_index(
		GIM_BVH_DATA_ARRAY & primitive_boxes,
		 int startIndex,  int endIndex, int splitAxis);

	int _calc_splitting_axis(GIM_BVH_DATA_ARRAY & primitive_boxes, int startIndex,  int endIndex);

	void _build_sub_tree(GIM_BVH_DATA_ARRAY & primitive_boxes, int startIndex,  int endIndex);
public:
	btQuantizedBvhTree()
	{
		m_num_nodes = 0;
	}

	//! prototype functions for box tree management
	//!@{
	void build_tree(GIM_BVH_DATA_ARRAY & primitive_boxes);

	public void quantizePoint(
		ushort * quantizedpoint, btVector3  point)
	{
		bt_quantize_clamp(quantizedpoint,point,m_global_bound.m_min,m_global_bound.m_max,m_bvhQuantization);
	}


	public bool testQuantizedBoxOverlapp(
		int node_index,
		ushort * quantizedMin,ushort * quantizedMax)
	{
		return m_node_array[node_index].testQuantizedBoxOverlapp(quantizedMin,quantizedMax);
	}

	public void clearNodes()
	{
		m_node_array.clear();
		m_num_nodes = 0;
	}

	//! node count
	public int getNodeCount()
	{
		return m_num_nodes;
	}

	//! tells if the node is a leaf
	public bool isLeafNode(int nodeindex)
	{
		return m_node_array[nodeindex].isLeafNode();
	}

	public int getNodeData(int nodeindex)
	{
		return m_node_array[nodeindex].getDataIndex();
	}

	public void getNodeBound(int nodeindex, btAABB & bound)
	{
		bound.m_min = bt_unquantize(
			m_node_array[nodeindex].m_quantizedAabbMin,
			m_global_bound.m_min,m_bvhQuantization);

		bound.m_max = bt_unquantize(
			m_node_array[nodeindex].m_quantizedAabbMax,
			m_global_bound.m_min,m_bvhQuantization);
	}

	public void setNodeBound(int nodeindex, btAABB & bound)
	{
		bt_quantize_clamp(	m_node_array[nodeindex].m_quantizedAabbMin,
							bound.m_min,
							m_global_bound.m_min,
							m_global_bound.m_max,
							m_bvhQuantization);

		bt_quantize_clamp(	m_node_array[nodeindex].m_quantizedAabbMax,
							bound.m_max,
							m_global_bound.m_min,
							m_global_bound.m_max,
							m_bvhQuantization);
	}

	public int getLeftNode(int nodeindex)
	{
		return nodeindex+1;
	}

	public int getRightNode(int nodeindex)
	{
		if(m_node_array[nodeindex+1].isLeafNode()) return nodeindex+2;
		return nodeindex+1 + m_node_array[nodeindex+1].getEscapeIndex();
	}

	public int getEscapeNodeIndex(int nodeindex)
	{
		return m_node_array[nodeindex].getEscapeIndex();
	}

	public bt_QUANTIZED_BVH_NODE * get_node_pointer(int index = 0)
	{
		return &m_node_array[index];
	}

	//!@}
};



//! Structure for containing Boxes
/*!
This class offers an structure for managing a box tree of primitives.
Requires a Primitive prototype (like btPrimitiveManagerBase )
*/
class btGImpactQuantizedBvh
{
protected:
	btQuantizedBvhTree m_box_tree;
	btPrimitiveManagerBase * m_primitive_manager;

protected:
	//stackless refit
	void refit();
public:

	//! this constructor doesn't build the tree. you must call	buildSet
	btGImpactQuantizedBvh()
	{
		m_primitive_manager = NULL;
	}

	//! this constructor doesn't build the tree. you must call	buildSet
	btGImpactQuantizedBvh(btPrimitiveManagerBase * primitive_manager)
	{
		m_primitive_manager = primitive_manager;
	}

	public btAABB getGlobalBox()  const
	{
		btAABB totalbox;
		getNodeBound(0, totalbox);
		return totalbox;
	}

	public void setPrimitiveManager(btPrimitiveManagerBase * primitive_manager)
	{
		m_primitive_manager = primitive_manager;
	}

	public btPrimitiveManagerBase * getPrimitiveManager()
	{
		return m_primitive_manager;
	}


//! node manager prototype functions
///@{

	//! this attemps to refit the box set.
	public void update()
	{
		refit();
	}

	//! this rebuild the entire set
	void buildSet();

	//! returns the indices of the primitives in the m_primitive_manager
	bool boxQuery(btAABB & box, btList<int> & collided_results);

	//! returns the indices of the primitives in the m_primitive_manager
	public bool boxQueryTrans(btAABB & box,
		 btTransform & transform, btList<int> & collided_results)
	{
		btAABB transbox=box;
		transbox.appy_transform(transform);
		return boxQuery(transbox,collided_results);
	}

	//! returns the indices of the primitives in the m_primitive_manager
	bool rayQuery(
		btVector3  ray_dir,btVector3  ray_origin ,
		List<int> & collided_results);

	//! tells if this set has hierarcht
	public bool hasHierarchy()
	{
		return true;
	}

	//! tells if this set is a trimesh
	public bool isTrimesh()  const
	{
		return m_primitive_manager.is_trimesh();
	}

	//! node count
	public int getNodeCount()
	{
		return m_box_tree.getNodeCount();
	}

	//! tells if the node is a leaf
	public bool isLeafNode(int nodeindex)
	{
		return m_box_tree.isLeafNode(nodeindex);
	}

	public int getNodeData(int nodeindex)
	{
		return m_box_tree.getNodeData(nodeindex);
	}

	public void getNodeBound(int nodeindex, btAABB & bound)  const
	{
		m_box_tree.getNodeBound(nodeindex, bound);
	}

	public void setNodeBound(int nodeindex, btAABB & bound)
	{
		m_box_tree.setNodeBound(nodeindex, bound);
	}


	public int getLeftNode(int nodeindex)
	{
		return m_box_tree.getLeftNode(nodeindex);
	}

	public int getRightNode(int nodeindex)
	{
		return m_box_tree.getRightNode(nodeindex);
	}

	public int getEscapeNodeIndex(int nodeindex)
	{
		return m_box_tree.getEscapeNodeIndex(nodeindex);
	}

	public void getNodeTriangle(int nodeindex,btPrimitiveTriangle & triangle)
	{
		m_primitive_manager.get_primitive_triangle(getNodeData(nodeindex),triangle);
	}


	public bt_QUANTIZED_BVH_NODE * get_node_pointer(int index = 0)
	{
		return m_box_tree.get_node_pointer(index);
	}

#if TRI_COLLISION_PROFILING
	static float getAverageTreeCollisionTime();
#endif //TRI_COLLISION_PROFILING

	static void find_collision(btGImpactQuantizedBvh * boxset1, btTransform & trans1,
		btGImpactQuantizedBvh * boxset2, btTransform & trans2,
		btPairSet & collision_pairs);
};


#endif // GIM_BOXPRUNING_H_INCLUDED
