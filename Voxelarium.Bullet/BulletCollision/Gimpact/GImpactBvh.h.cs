#if ! GIM_BOX_SET_H_INCLUDED
#define GIM_BOX_SET_H_INCLUDED

/*! \file gim_box_set.h
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


#include "LinearMath/List.h"

#include "btBoxCollision.h"
#include "btTriangleShapeEx.h"





//! Overlapping pair
struct GIM_PAIR
{
    int m_index1;
    int m_index2;
    GIM_PAIR()
    {}

    GIM_PAIR(string IM_PAIR & p)
    {
    	m_index1 = p.m_index1;
    	m_index2 = p.m_index2;
	}

	GIM_PAIR(int index1, int index2)
    {
    	m_index1 = index1;
    	m_index2 = index2;
	}
};

//! A pairset array
class btPairSet: btList<GIM_PAIR>
{
public:
	btPairSet()
	{
		reserve(32);
	}
	inline void push_pair(int index1,int index2)
	{
		push_back(GIM_PAIR(index1,index2));
	}

	inline void push_pair_inv(int index1,int index2)
	{
		push_back(GIM_PAIR(index2,index1));
	}
};


///GIM_BVH_DATA is an internal GIMPACT collision structure to contain axis aligned bounding box
struct GIM_BVH_DATA
{
	btAABB m_bound;
	int m_data;
};

//! Node Structure for trees
class GIM_BVH_TREE_NODE
{
public:
	btAABB m_bound;
protected:
	int	m_escapeIndexOrDataIndex;
public:
	GIM_BVH_TREE_NODE()
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

};


class GIM_BVH_DATA_ARRAY: btList<GIM_BVH_DATA>
{
};


class GIM_BVH_TREE_NODE_ARRAY: btList<GIM_BVH_TREE_NODE>
{
};




//! Basic Box tree structure
class btBvhTree
{
protected:
	int m_num_nodes;
	GIM_BVH_TREE_NODE_ARRAY m_node_array;
protected:
	int _sort_and_calc_splitting_index(
		GIM_BVH_DATA_ARRAY & primitive_boxes,
		 int startIndex,  int endIndex, int splitAxis);

	int _calc_splitting_axis(GIM_BVH_DATA_ARRAY & primitive_boxes, int startIndex,  int endIndex);

	void _build_sub_tree(GIM_BVH_DATA_ARRAY & primitive_boxes, int startIndex,  int endIndex);
public:
	btBvhTree()
	{
		m_num_nodes = 0;
	}

	//! prototype functions for box tree management
	//!@{
	void build_tree(GIM_BVH_DATA_ARRAY & primitive_boxes);

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
		bound = m_node_array[nodeindex].m_bound;
	}

	public void setNodeBound(int nodeindex, btAABB & bound)
	{
		m_node_array[nodeindex].m_bound = bound;
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

	public string IM_BVH_TREE_NODE * get_node_pointer(int index = 0)
	{
		return &m_node_array[index];
	}

	//!@}
};


//! Prototype Base class for primitive classification
/*!
This class is a wrapper for primitive collections.
This tells relevant info for the Bounding Box set classes, which take care of space classification.
This class can manage Compound shapes and trimeshes, and if it is managing trimesh then the  Hierarchy Bounding Box classes will take advantage of primitive Vs Box overlapping tests for getting optimal results and less Per Box compairisons.
*/
class btPrimitiveManagerBase
{
public:

	virtual ~btPrimitiveManagerBase() {}

	//! determines if this manager consist on only triangles, which special case will be optimized
	virtual bool is_trimesh() string = 0;
	virtual int get_primitive_count() string = 0;
	virtual void get_primitive_box(int prim_index ,btAABB & primbox) string = 0;
	//! retrieves only the points of the triangle, and the collision margin
	virtual void get_primitive_triangle(int prim_index,btPrimitiveTriangle & triangle)= 0;
};


//! Structure for containing Boxes
/*!
This class offers an structure for managing a box tree of primitives.
Requires a Primitive prototype (like btPrimitiveManagerBase )
*/
class btGImpactBvh
{
protected:
	btBvhTree m_box_tree;
	btPrimitiveManagerBase * m_primitive_manager;

protected:
	//stackless refit
	void refit();
public:

	//! this constructor doesn't build the tree. you must call	buildSet
	btGImpactBvh()
	{
		m_primitive_manager = NULL;
	}

	//! this constructor doesn't build the tree. you must call	buildSet
	btGImpactBvh(btPrimitiveManagerBase * primitive_manager)
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


	public string IM_BVH_TREE_NODE * get_node_pointer(int index = 0)
	{
		return m_box_tree.get_node_pointer(index);
	}

#if TRI_COLLISION_PROFILING
	static float getAverageTreeCollisionTime();
#endif //TRI_COLLISION_PROFILING

	static void find_collision(btGImpactBvh * boxset1, btTransform & trans1,
		btGImpactBvh * boxset2, btTransform & trans2,
		btPairSet & collision_pairs);
};


#endif // GIM_BOXPRUNING_H_INCLUDED
