/*! \file btGImpactShape.h
\author Francisco Len Nßjera
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


#if ! GIMPACT_SHAPE_H
#define GIMPACT_SHAPE_H

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btStridingMeshInterface.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/List.h"

#include "btGImpactQuantizedBvh.h" // box tree class


//! declare Quantized trees, (you can change to float based trees)
typedef btGImpactQuantizedBvh btGImpactBoxSet;

enum eGIMPACT_SHAPE_TYPE
{
	CONST_GIMPACT_COMPOUND_SHAPE = 0,
	CONST_GIMPACT_TRIMESH_SHAPE_PART,
	CONST_GIMPACT_TRIMESH_SHAPE
};



//! Helper class for tetrahedrons
class btTetrahedronShapeEx: btBU_Simplex1to4
{
public:
	btTetrahedronShapeEx()
	{
		m_numVertices = 4;
	}


	public void setVertices(
		btVector3  v0,btVector3  v1,
		btVector3  v2,btVector3  v3)
	{
		m_vertices[0] = v0;
		m_vertices[1] = v1;
		m_vertices[2] = v2;
		m_vertices[3] = v3;
		recalcLocalAabb();
	}
};


//! Base class for gimpact shapes
class btGImpactShapeInterface : btConcaveShape
{
protected:
    btAABB m_localAABB;
    bool m_needs_update;
    btVector3  localScaling;
    btGImpactBoxSet m_box_set;// optionally boxset

	//! use this function for perfofm refit in bounding boxes
    //! use this function for perfofm refit in bounding boxes
    virtual void calcLocalAABB()
    {
		lockChildShapes();
    	if(m_box_set.getNodeCount() == 0)
    	{
    		m_box_set.buildSet();
    	}
    	else
    	{
    		m_box_set.update();
    	}
    	unlockChildShapes();

    	m_localAABB = m_box_set.getGlobalBox();
    }


public:
	btGImpactShapeInterface()
	{
		m_shapeType=GIMPACT_SHAPE_PROXYTYPE;
		m_localAABB.invalidate();
		m_needs_update = true;
		localScaling.setValue(1,1,1);
	}


	//! performs refit operation
	/*!
	Updates the entire Box set of this shape.
	\pre postUpdate() must be called for attemps to calculating the box set, else this function
		will does nothing.
	\post if m_needs_update == true, then it calls calcLocalAABB();
	*/
    public void updateBound()
    {
    	if(!m_needs_update) return;
    	calcLocalAABB();
    	m_needs_update  = false;
    }

    //! If the Bounding box is not updated, then this class attemps to calculate it.
    /*!
    \post Calls updateBound() for update the box set.
    */
    void getAabb(ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax)
    {
        btAABB transformedbox = m_localAABB;
        transformedbox.appy_transform(t);
        aabbMin = transformedbox.m_min;
        aabbMax = transformedbox.m_max;
    }

    //! Tells to this object that is needed to refit the box set
    virtual void postUpdate()
    {
    	m_needs_update = true;
    }

	//! Obtains the local box, which is the global calculated box of the total of subshapes
	public btAABB & getLocalBox()
	{
		return m_localAABB;
	}


    virtual int	getShapeType()
    {
        return GIMPACT_SHAPE_PROXYTYPE;
    }

    /*!
	\post You must call updateBound() for update the box set.
	*/
	virtual void	setLocalScaling(ref btVector3 scaling)
	{
		localScaling = scaling;
		postUpdate();
	}

	virtual ref btVector3 getLocalScaling()
	{
		return localScaling;
	}


	virtual void setMargin(double margin)
    {
    	m_collisionMargin = margin;
    	int i = getNumChildShapes();
    	while(i--)
    	{
			btCollisionShape* child = getChildShape(i);
			child.setMargin(margin);
    	}

		m_needs_update = true;
    }


	//! Subshape member functions
	//!@{

	//! Base method for determinig which kind of GIMPACT shape we get
	virtual eGIMPACT_SHAPE_TYPE getGImpactShapeType() string  0 ;

	//! gets boxset
	public btGImpactBoxSet * getBoxSet()
	{
		return &m_box_set;
	}

	//! Determines if this class has a hierarchy structure for sorting its primitives
	public bool hasBoxSet()  const
	{
		if(m_box_set.getNodeCount() == 0) return false;
		return true;
	}

	//! Obtains the primitive manager
	virtual btPrimitiveManagerBase * getPrimitiveManager()  string = 0;


	//! Gets the number of children
	virtual int	getNumChildShapes() string = 0;

	//! if true, then its children must get transforms.
	virtual bool childrenHasTransform() string = 0;

	//! Determines if this shape has triangles
	virtual bool needsRetrieveTriangles() string = 0;

	//! Determines if this shape has tetrahedrons
	virtual bool needsRetrieveTetrahedrons() string = 0;

	virtual void getBulletTriangle(int prim_index,btTriangleShapeEx & triangle) string = 0;

	virtual void getBulletTetrahedron(int prim_index,btTetrahedronShapeEx & tetrahedron) string = 0;



	//! call when reading child shapes
	virtual void lockChildShapes()
	{
	}

	virtual void unlockChildShapes()
	{
	}

	//! if this trimesh
	public void getPrimitiveTriangle(int index,btPrimitiveTriangle & triangle)
	{
		getPrimitiveManager().get_primitive_triangle(index,triangle);
	}


	//! Retrieves the bound from a child
    /*!
    */
    virtual void getChildAabb(int child_index,ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax)
    {
        btAABB child_aabb;
        getPrimitiveManager().get_primitive_box(child_index,child_aabb);
        child_aabb.appy_transform(t);
        aabbMin = child_aabb.m_min;
        aabbMax = child_aabb.m_max;
    }

	//! Gets the children
	virtual btCollisionShape* getChildShape(int index) = 0;


	//! Gets the child
	virtual btCollisionShape* getChildShape(int index) string = 0;

	//! Gets the children transform
	virtual btTransform	getChildTransform(int index) string = 0;

	//! Sets the children transform
	/*!
	\post You must call updateBound() for update the box set.
	*/
	virtual void setChildTransform(int index, btTransform & transform) = 0;

	//!@}


	//! virtual method for ray collision
	virtual void rayTest(ref btVector3 rayFrom, ref btVector3 rayTo, btCollisionWorld::RayResultCallback& resultCallback)  const
	{
        (void) rayFrom; (void) rayTo; (void) resultCallback;
	}

	//! Function for retrieve triangles.
	/*!
	It gives the triangles in local space
	*/
	virtual void	processAllTriangles(btTriangleCallback* callback,ref btVector3 aabbMin,ref btVector3 aabbMax)
	{
        (void) callback; (void) aabbMin; (void) aabbMax;
	}

	//! Function for retrieve triangles.
	/*!
	It gives the triangles in local space
	*/
	virtual void processAllTrianglesRay(btTriangleCallback* /*callback*/,ref btVector3 /*rayFrom*/, ref btVector3 /*rayTo*/)
	{
		
	}

	//!@}

};


//! btGImpactCompoundShape allows to handle multiple btCollisionShape objects at once
/*!
This class only can manage Convex subshapes
*/
class btGImpactCompoundShape	: btGImpactShapeInterface
{
public:
	//! compound primitive manager
	class CompoundPrimitiveManager: btPrimitiveManagerBase
	{
	public:
		virtual ~CompoundPrimitiveManager() {}
		btGImpactCompoundShape * m_compoundShape;


		CompoundPrimitiveManager(string ompoundPrimitiveManager& compound)
            : btPrimitiveManagerBase()
		{
			m_compoundShape = compound.m_compoundShape;
		}

		CompoundPrimitiveManager(btGImpactCompoundShape * compoundShape)
		{
			m_compoundShape = compoundShape;
		}

		CompoundPrimitiveManager()
		{
			m_compoundShape = NULL;
		}

		virtual bool is_trimesh()
		{
			return false;
		}

		virtual int get_primitive_count()
		{
			return (int )m_compoundShape.getNumChildShapes();
		}

		virtual void get_primitive_box(int prim_index ,btAABB & primbox)
		{
			btTransform prim_trans;
			if(m_compoundShape.childrenHasTransform())
			{
				prim_trans = m_compoundShape.getChildTransform(prim_index);
			}
			else
			{
				prim_trans.setIdentity();
			}
			btCollisionShape* shape = m_compoundShape.getChildShape(prim_index);
			shape.getAabb(prim_trans,primbox.m_min,primbox.m_max);
		}

		virtual void get_primitive_triangle(int prim_index,btPrimitiveTriangle & triangle)
		{
			Debug.Assert(false);
            (void) prim_index; (void) triangle;
		}

	};



protected:
	CompoundPrimitiveManager m_primitive_manager;
	List<btTransform>		m_childTransforms;
	List<btCollisionShape*>	m_childShapes;


public:

	btGImpactCompoundShape(bool children_has_transform = true)
	{
        (void) children_has_transform;
		m_primitive_manager.m_compoundShape = this;
		m_box_set.setPrimitiveManager(&m_primitive_manager);
	}

	virtual ~btGImpactCompoundShape()
	{
	}


	//! if true, then its children must get transforms.
	virtual bool childrenHasTransform()
	{
		if(m_childTransforms.Count==0) return false;
		return true;
	}


	//! Obtains the primitive manager
	virtual btPrimitiveManagerBase * getPrimitiveManager()  const
	{
		return &m_primitive_manager;
	}

	//! Obtains the compopund primitive manager
	public CompoundPrimitiveManager * getCompoundPrimitiveManager()
	{
		return &m_primitive_manager;
	}

	//! Gets the number of children
	virtual int	getNumChildShapes()
	{
		return m_childShapes.Count;
	}


	//! Use this method for adding children. Only Convex shapes are allowed.
	void addChildShape(ref btTransform localTransform,btCollisionShape* shape)
	{
		Debug.Assert(shape.isConvex());
		m_childTransforms.Add(localTransform);
		m_childShapes.Add(shape);
	}

	//! Use this method for adding children. Only Convex shapes are allowed.
	void addChildShape(btCollisionShape* shape)
	{
		Debug.Assert(shape.isConvex());
		m_childShapes.Add(shape);
	}

	//! Gets the children
	virtual btCollisionShape* getChildShape(int index)
	{
		return m_childShapes[index];
	}

	//! Gets the children
	virtual btCollisionShape* getChildShape(int index)
	{
		return m_childShapes[index];
	}

	//! Retrieves the bound from a child
    /*!
    */
    virtual void getChildAabb(int child_index,ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax)
    {

    	if(childrenHasTransform())
    	{
    		m_childShapes[child_index].getAabb(t*m_childTransforms[child_index],aabbMin,aabbMax);
    	}
    	else
    	{
    		m_childShapes[child_index].getAabb(t,aabbMin,aabbMax);
    	}
    }


	//! Gets the children transform
	virtual btTransform	getChildTransform(int index)
	{
		Debug.Assert(m_childTransforms.Count == m_childShapes.Count);
		return m_childTransforms[index];
	}

	//! Sets the children transform
	/*!
	\post You must call updateBound() for update the box set.
	*/
	virtual void setChildTransform(int index, btTransform & transform)
	{
		Debug.Assert(m_childTransforms.Count == m_childShapes.Count);
		m_childTransforms[index] = transform;
		postUpdate();
	}

	//! Determines if this shape has triangles
	virtual bool needsRetrieveTriangles()
	{
		return false;
	}

	//! Determines if this shape has tetrahedrons
	virtual bool needsRetrieveTetrahedrons()
	{
		return false;
	}


	virtual void getBulletTriangle(int prim_index,btTriangleShapeEx  triangle)
	{
        (void) prim_index; (void) triangle;
		Debug.Assert(false);
	}

	virtual void getBulletTetrahedron(int prim_index,btTetrahedronShapeEx & tetrahedron)
	{
        (void) prim_index; (void) tetrahedron;
		Debug.Assert(false);
	}


	//! Calculates the exact inertia tensor for this shape
	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);

	virtual string	getName()const
	{
		return "GImpactCompound";
	}

	virtual eGIMPACT_SHAPE_TYPE getGImpactShapeType()
	{
		return CONST_GIMPACT_COMPOUND_SHAPE;
	}

};



//! This class manages a sub part of a mesh supplied by the btStridingMeshInterface interface.
/*!
- Simply create this shape by passing the btStridingMeshInterface to the constructor btGImpactMeshShapePart, then you must call updateBound() after creating the mesh
- When making operations with this shape, you must call <b>lock</b> before accessing to the trimesh primitives, and then call <b>unlock</b>
- You can handle deformable meshes with this shape, by calling postUpdate() every time when changing the mesh vertices.

*/
class btGImpactMeshShapePart : btGImpactShapeInterface
{
public:
	//! Trimesh primitive manager
	/*!
	Manages the info from btStridingMeshInterface object and controls the Lock/Unlock mechanism
	*/
	class TrimeshPrimitiveManager: btPrimitiveManagerBase
	{
	public:
		double m_margin;
		btStridingMeshInterface * m_meshInterface;
		btVector3 m_scale;
		int m_part;
		int m_lock_count;
		string nsigned char *vertexbase;
		int numverts;
		PHY_ScalarType type;
		int stride;
		string nsigned char *indexbase;
		int indexstride;
		int  numfaces;
		PHY_ScalarType indicestype;

		TrimeshPrimitiveManager()
		{
			m_meshInterface = NULL;
			m_part = 0;
			m_margin = 0.01f;
			m_scale = btVector3(1,1,1);
			m_lock_count = 0;
			vertexbase = 0;
			numverts = 0;
			stride = 0;
			indexbase = 0;
			indexstride = 0;
			numfaces = 0;
		}

 		TrimeshPrimitiveManager(string rimeshPrimitiveManager & manager)
            : btPrimitiveManagerBase()
		{
			m_meshInterface = manager.m_meshInterface;
			m_part = manager.m_part;
			m_margin = manager.m_margin;
			m_scale = manager.m_scale;
			m_lock_count = 0;
			vertexbase = 0;
			numverts = 0;
			stride = 0;
			indexbase = 0;
			indexstride = 0;
			numfaces = 0;

		}

		TrimeshPrimitiveManager(
			btStridingMeshInterface * meshInterface,	int part)
		{
			m_meshInterface = meshInterface;
			m_part = part;
			m_scale = m_meshInterface.getScaling();
			m_margin = 0.1f;
			m_lock_count = 0;
			vertexbase = 0;
			numverts = 0;
			stride = 0;
			indexbase = 0;
			indexstride = 0;
			numfaces = 0;

		}

		virtual ~TrimeshPrimitiveManager() {}

		void lock()
		{
			if(m_lock_count>0)
			{
				m_lock_count++;
				return;
			}
			m_meshInterface.getLockedReadOnlyVertexIndexBase(
				&vertexbase,numverts,
				type, stride,&indexbase, indexstride, numfaces,indicestype,m_part);

			m_lock_count = 1;
		}

		void unlock()
		{
			if(m_lock_count == 0) return;
			if(m_lock_count>1)
			{
				--m_lock_count;
				return;
			}
			m_meshInterface.unLockReadOnlyVertexBase(m_part);
			vertexbase = NULL;
			m_lock_count = 0;
		}

		virtual bool is_trimesh()
		{
			return true;
		}

		virtual int get_primitive_count()
		{
			return (int )numfaces;
		}

		public int get_vertex_count()
		{
			return (int )numverts;
		}

		public void get_indices(int face_index,uint &i0,uint &i1,uint &i2)
		{
			if(indicestype == PHY_SHORT)
			{
				ushort* s_indices = (ushort *)(indexbase + face_index * indexstride);
				i0 = s_indices;
				i1 = s_indices[1];
				i2 = s_indices[2];
			}
			else
			{
				uint * i_indices = (uint *)(indexbase + face_index*indexstride);
				i0 = i_indices[0];
				i1 = i_indices[1];
				i2 = i_indices[2];
			}
		}

		public void get_vertex(uint vertex_index, btVector3  vertex)
		{
			if(type == PHY_DOUBLE)
			{
				double * dvertices = (double *)(vertexbase + vertex_index*stride);
				vertex[0] = (double)(dvertices[0]*m_scale[0]);
				vertex[1] = (double)(dvertices[1]*m_scale[1]);
				vertex[2] = (double)(dvertices[2]*m_scale[2]);
			}
			else
			{
				float * svertices = (float *)(vertexbase + vertex_index*stride);
				vertex[0] = svertices[0]*m_scale[0];
				vertex[1] = svertices[1]*m_scale[1];
				vertex[2] = svertices[2]*m_scale[2];
			}
		}

		virtual void get_primitive_box(int prim_index ,btAABB  primbox)
		{
			btPrimitiveTriangle  triangle;
			get_primitive_triangle(prim_index,triangle);
			primbox.calc_from_triangle_margin(
				triangle.m_vertices,
				triangle.m_vertices[1],triangle.m_vertices[2],triangle.m_margin);
		}

		virtual void get_primitive_triangle(int prim_index,btPrimitiveTriangle & triangle)
		{
			uint indices[3];
			get_indices(prim_index,indices[0],indices[1],indices[2]);
			get_vertex(indices[0],triangle.m_vertices[0]);
			get_vertex(indices[1],triangle.m_vertices[1]);
			get_vertex(indices[2],triangle.m_vertices[2]);
			triangle.m_margin = m_margin;
		}

		public void get_bullet_triangle(int prim_index,btTriangleShapeEx & triangle)
		{
			uint indices[3];
			get_indices(prim_index,indices[0],indices[1],indices[2]);
			get_vertex(indices[0],triangle.m_vertices1[0]);
			get_vertex(indices[1],triangle.m_vertices1[1]);
			get_vertex(indices[2],triangle.m_vertices1[2]);
			triangle.setMargin(m_margin);
		}

	};


protected:
	TrimeshPrimitiveManager m_primitive_manager;
public:

	btGImpactMeshShapePart()
	{
		m_box_set.setPrimitiveManager(&m_primitive_manager);
	}


	btGImpactMeshShapePart(btStridingMeshInterface * meshInterface,	int part)
	{
		m_primitive_manager.m_meshInterface = meshInterface;
		m_primitive_manager.m_part = part;
		m_box_set.setPrimitiveManager(&m_primitive_manager);
	}

	virtual ~btGImpactMeshShapePart()
	{
	}

	//! if true, then its children must get transforms.
	virtual bool childrenHasTransform()
	{
		return false;
	}


	//! call when reading child shapes
	virtual void lockChildShapes()
	{
		object  dummy = (object)(m_box_set.getPrimitiveManager());
		TrimeshPrimitiveManager * dummymanager = static_cast<TrimeshPrimitiveManager *>(dummy);
		dummymanager.lock();
	}

	virtual void unlockChildShapes()  const
	{
		object  dummy = (object)(m_box_set.getPrimitiveManager());
		TrimeshPrimitiveManager * dummymanager = static_cast<TrimeshPrimitiveManager *>(dummy);
		dummymanager.unlock();
	}

	//! Gets the number of children
	virtual int	getNumChildShapes()
	{
		return m_primitive_manager.get_primitive_count();
	}


	//! Gets the children
	virtual btCollisionShape* getChildShape(int index)
	{
        (void) index;
		Debug.Assert(false);
		return NULL;
	}



	//! Gets the child
	virtual btCollisionShape* getChildShape(int index)
	{
        (void) index;
		Debug.Assert(false);
		return NULL;
	}

	//! Gets the children transform
	virtual btTransform	getChildTransform(int index)
	{
        (void) index;
		Debug.Assert(false);
		return btTransform();
	}

	//! Sets the children transform
	/*!
	\post You must call updateBound() for update the box set.
	*/
	virtual void setChildTransform(int index, btTransform & transform)
	{
        (void) index;
        (void) transform;
		Debug.Assert(false);
	}


	//! Obtains the primitive manager
	virtual btPrimitiveManagerBase * getPrimitiveManager()  const
	{
		return &m_primitive_manager;
	}

	public TrimeshPrimitiveManager * getTrimeshPrimitiveManager()
	{
		return &m_primitive_manager;
	}





	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);




	virtual string	getName()const
	{
		return "GImpactMeshShapePart";
	}

	virtual eGIMPACT_SHAPE_TYPE getGImpactShapeType()
	{
		return CONST_GIMPACT_TRIMESH_SHAPE_PART;
	}

	//! Determines if this shape has triangles
	virtual bool needsRetrieveTriangles()
	{
		return true;
	}

	//! Determines if this shape has tetrahedrons
	virtual bool needsRetrieveTetrahedrons()
	{
		return false;
	}

	virtual void getBulletTriangle(int prim_index,btTriangleShapeEx & triangle)
	{
		m_primitive_manager.get_bullet_triangle(prim_index,triangle);
	}

	virtual void getBulletTetrahedron(int prim_index,btTetrahedronShapeEx & tetrahedron)
	{
        (void) prim_index;
        (void) tetrahedron;
		Debug.Assert(false);
	}



	public int getVertexCount()
	{
		return m_primitive_manager.get_vertex_count();
	}

	public void getVertex(int vertex_index, btVector3  vertex)
	{
		m_primitive_manager.get_vertex(vertex_index,vertex);
	}

	public void setMargin(double margin)
    {
    	m_primitive_manager.m_margin = margin;
    	postUpdate();
    }

    public double getMargin()
    {
    	return m_primitive_manager.m_margin;
    }

    virtual void	setLocalScaling(ref btVector3 scaling)
    {
    	m_primitive_manager.m_scale = scaling;
    	postUpdate();
    }

    virtual ref btVector3 getLocalScaling()
    {
    	return m_primitive_manager.m_scale;
    }

    public int getPart()
    {
    	return (int)m_primitive_manager.m_part;
    }

	virtual void	processAllTriangles(btTriangleCallback* callback,ref btVector3 aabbMin,ref btVector3 aabbMax);
	virtual void	processAllTrianglesRay(btTriangleCallback* callback,ref btVector3 rayFrom,ref btVector3 rayTo);
};


//! This class manages a mesh supplied by the btStridingMeshInterface interface.
/*!
Set of btGImpactMeshShapePart parts
- Simply create this shape by passing the btStridingMeshInterface to the constructor btGImpactMeshShape, then you must call updateBound() after creating the mesh

- You can handle deformable meshes with this shape, by calling postUpdate() every time when changing the mesh vertices.

*/
class btGImpactMeshShape : btGImpactShapeInterface
{
	btStridingMeshInterface* m_meshInterface;

protected:
	List<btGImpactMeshShapePart*> m_mesh_parts;
	void buildMeshParts(btStridingMeshInterface * meshInterface)
	{
		for (int i=0;i<meshInterface.getNumSubParts() ;++i )
		{
			btGImpactMeshShapePart * newpart = new btGImpactMeshShapePart(meshInterface,i);
			m_mesh_parts.Add(newpart);
		}
	}

	//! use this function for perfofm refit in bounding boxes
    virtual void calcLocalAABB()
    {
    	m_localAABB.invalidate();
    	int i = m_mesh_parts.Count;
    	while(i--)
    	{
    		m_mesh_parts[i].updateBound();
    		m_localAABB.merge(m_mesh_parts[i].getLocalBox());
    	}
    }

public:
	btGImpactMeshShape(btStridingMeshInterface * meshInterface)
	{
		m_meshInterface = meshInterface;
		buildMeshParts(meshInterface);
	}

	virtual ~btGImpactMeshShape()
	{
		int i = m_mesh_parts.Count;
    	while(i--)
    	{
			btGImpactMeshShapePart * part = m_mesh_parts[i];
			delete part;
    	}
		m_mesh_parts.clear();
	}


	btStridingMeshInterface* getMeshInterface()
	{
		return m_meshInterface;
	}

	btStridingMeshInterface* getMeshInterface()
	{
		return m_meshInterface;
	}

	int getMeshPartCount()
	{
		return m_mesh_parts.Count;
	}

	btGImpactMeshShapePart * getMeshPart(int index)
	{
		return m_mesh_parts[index];
	}



	btGImpactMeshShapePart * getMeshPart(int index)
	{
		return m_mesh_parts[index];
	}


	virtual void	setLocalScaling(ref btVector3 scaling)
	{
		localScaling = scaling;

		int i = m_mesh_parts.Count;
    	while(i--)
    	{
			btGImpactMeshShapePart * part = m_mesh_parts[i];
			part.setLocalScaling(scaling);
    	}

		m_needs_update = true;
	}

	virtual void setMargin(double margin)
    {
    	m_collisionMargin = margin;

		int i = m_mesh_parts.Count;
    	while(i--)
    	{
			btGImpactMeshShapePart * part = m_mesh_parts[i];
			part.setMargin(margin);
    	}

		m_needs_update = true;
    }

	//! Tells to this object that is needed to refit all the meshes
    virtual void postUpdate()
    {
		int i = m_mesh_parts.Count;
    	while(i--)
    	{
			btGImpactMeshShapePart * part = m_mesh_parts[i];
			part.postUpdate();
    	}

    	m_needs_update = true;
    }

	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);


	//! Obtains the primitive manager
	virtual btPrimitiveManagerBase * getPrimitiveManager()  const
	{
		Debug.Assert(false);
		return NULL;
	}


	//! Gets the number of children
	virtual int	getNumChildShapes()
	{
		Debug.Assert(false);
		return 0;
	}


	//! if true, then its children must get transforms.
	virtual bool childrenHasTransform()
	{
		Debug.Assert(false);
		return false;
	}

	//! Determines if this shape has triangles
	virtual bool needsRetrieveTriangles()
	{
		Debug.Assert(false);
		return false;
	}

	//! Determines if this shape has tetrahedrons
	virtual bool needsRetrieveTetrahedrons()
	{
		Debug.Assert(false);
		return false;
	}

	virtual void getBulletTriangle(int prim_index,btTriangleShapeEx & triangle)
	{
        (void) prim_index; (void) triangle;
		Debug.Assert(false);
	}

	virtual void getBulletTetrahedron(int prim_index,btTetrahedronShapeEx & tetrahedron)
	{
        (void) prim_index; (void) tetrahedron;
		Debug.Assert(false);
	}

	//! call when reading child shapes
	virtual void lockChildShapes()
	{
		Debug.Assert(false);
	}

	virtual void unlockChildShapes()
	{
		Debug.Assert(false);
	}




	//! Retrieves the bound from a child
    /*!
    */
    virtual void getChildAabb(int child_index,ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax)
    {
        (void) child_index; (void) t; (void) aabbMin; (void) aabbMax;
        Debug.Assert(false);
    }

	//! Gets the children
	virtual btCollisionShape* getChildShape(int index)
	{
        (void) index;
		Debug.Assert(false);
		return NULL;
	}


	//! Gets the child
	virtual btCollisionShape* getChildShape(int index)
	{
        (void) index;
		Debug.Assert(false);
		return NULL;
	}

	//! Gets the children transform
	virtual btTransform	getChildTransform(int index)
	{
        (void) index;
		Debug.Assert(false);
		return btTransform();
	}

	//! Sets the children transform
	/*!
	\post You must call updateBound() for update the box set.
	*/
	virtual void setChildTransform(int index, btTransform & transform)
	{
        (void) index; (void) transform;
		Debug.Assert(false);
	}


	virtual eGIMPACT_SHAPE_TYPE getGImpactShapeType()
	{
		return CONST_GIMPACT_TRIMESH_SHAPE;
	}


	virtual string	getName()const
	{
		return "GImpactMesh";
	}

	virtual void rayTest(ref btVector3 rayFrom, ref btVector3 rayTo, btCollisionWorld::RayResultCallback& resultCallback)  const;

	//! Function for retrieve triangles.
	/*!
	It gives the triangles in local space
	*/
	virtual void	processAllTriangles(btTriangleCallback* callback,ref btVector3 aabbMin,ref btVector3 aabbMax);

	virtual void	processAllTrianglesRay (btTriangleCallback* callback,ref btVector3 rayFrom,ref btVector3 rayTo);

	virtual	int	calculateSerializeBufferSize();

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer, btSerializer* serializer);

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btGImpactMeshShapeData
{
	btCollisionShapeData	m_collisionShapeData;

	btStridingMeshInterfaceData m_meshInterface;

	btVector3FloatData	m_localScaling;

	float	m_collisionMargin;

	int		m_gimpactSubType;
};

public	int	btGImpactMeshShape::calculateSerializeBufferSize()
{
	return sizeof(btGImpactMeshShapeData);
}


#endif //GIMPACT_MESH_SHAPE_H
