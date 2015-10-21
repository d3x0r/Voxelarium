//#define GIMPACT_VS_PLANE_COLLISION 1

/*! \file btGImpactShape.h
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

namespace Bullet.Collision.Gimpact
{


//! Collision Algorithm for GImpact Shapes
/*!
For register this algorithm in Bullet, proceed as following:
 \code
btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(m_dynamicsWorld .getDispatcher());
btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
 \endcode
*/
public class btGImpactCollisionAlgorithm : btActivatingCollisionAlgorithm
{
protected:
	btCollisionAlgorithm * m_convex_algorithm;
    btPersistentManifold * m_manifoldPtr;
	btManifoldResult* m_resultOut;
	btDispatcherInfo * m_dispatchInfo;
	int m_triface0;
	int m_part0;
	int m_triface1;
	int m_part1;


	//! Creates a new contact point
	public btPersistentManifold* newContactManifold(btCollisionObject body0,btCollisionObject body1)
	{
		m_manifoldPtr = m_dispatcher.getNewManifold(body0,body1);
		return m_manifoldPtr;
	}

	public void destroyConvexAlgorithm()
	{
		if(m_convex_algorithm)
		{
			m_convex_algorithm.~btCollisionAlgorithm();
			m_dispatcher.freeCollisionAlgorithm( m_convex_algorithm);
			m_convex_algorithm = NULL;
		}
	}

	public void destroyContactManifolds()
	{
		if(m_manifoldPtr == NULL) return;
		m_dispatcher.releaseManifold(m_manifoldPtr);
		m_manifoldPtr = NULL;
	}

	public void clearCache()
	{
		destroyContactManifolds();
		destroyConvexAlgorithm();

		m_triface0 = -1;
		m_part0 = -1;
		m_triface1 = -1;
		m_part1 = -1;
	}

	public btPersistentManifold* getLastManifold()
	{
		return m_manifoldPtr;
	}


	// Call before process collision
	public void checkManifold(btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap)
	{
		if(getLastManifold() == 0)
		{
			newContactManifold(body0Wrap.getCollisionObject(),body1Wrap.getCollisionObject());
		}

		m_resultOut.setPersistentManifold(getLastManifold());
	}

	// Call before process collision
	public btCollisionAlgorithm * newAlgorithm(btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap)
	{
		checkManifold(body0Wrap,body1Wrap);

		btCollisionAlgorithm * convex_algorithm = m_dispatcher.findAlgorithm(
				body0Wrap,body1Wrap,getLastManifold());
		return convex_algorithm ;
	}

	// Call before process collision
	public void checkConvexAlgorithm(btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap)
	{
		if(m_convex_algorithm) return;
		m_convex_algorithm = newAlgorithm(body0Wrap,body1Wrap);
	}




	void addContactPoint(btCollisionObjectWrapper * body0Wrap,
					btCollisionObjectWrapper * body1Wrap,
					btVector3  point,
					btVector3  normal,
					double distance);

//! Collision routines
//!@{

	void collide_gjk_triangles(btCollisionObjectWrapper* body0Wrap,
				  btCollisionObjectWrapper* body1Wrap,
				  btGImpactMeshShapePart * shape0,
				  btGImpactMeshShapePart * shape1,
				  int * pairs, int pair_count);

	void collide_sat_triangles(btCollisionObjectWrapper* body0Wrap,
					  btCollisionObjectWrapper* body1Wrap,
					  btGImpactMeshShapePart * shape0,
					  btGImpactMeshShapePart * shape1,
					  int * pairs, int pair_count);




	void shape_vs_shape_collision(
					  btCollisionObjectWrapper* body0,
					  btCollisionObjectWrapper* body1,
					  btCollisionShape * shape0,
					  btCollisionShape * shape1);

	void convex_vs_convex_collision(btCollisionObjectWrapper* body0Wrap,
					  btCollisionObjectWrapper* body1Wrap,
					  btCollisionShape* shape0,
					  btCollisionShape* shape1);



	void gimpact_vs_gimpact_find_pairs(
					  btTransform & trans0,
					  btTransform & trans1,
					  btGImpactShapeInterface * shape0,
					  btGImpactShapeInterface * shape1,btPairSet & pairset);

	void gimpact_vs_shape_find_pairs(
					  btTransform & trans0,
					  btTransform & trans1,
					  btGImpactShapeInterface * shape0,
					  btCollisionShape * shape1,
					  btList<int> & collided_primitives);


	void gimpacttrimeshpart_vs_plane_collision(
					  btCollisionObjectWrapper * body0Wrap,
					  btCollisionObjectWrapper * body1Wrap,
					  btGImpactMeshShapePart * shape0,
					  btStaticPlaneShape * shape1,bool swapped);


public:

	btGImpactCollisionAlgorithm( btCollisionAlgorithmConstructionInfo& ci,btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap);

	virtual ~btGImpactCollisionAlgorithm();

	virtual void processCollision (btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	double	calculateTimeOfImpact(btCollisionObject body0,btCollisionObject body1,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray)
	{
		if (m_manifoldPtr)
			manifoldArray.Add(m_manifoldPtr);
	}

	btManifoldResult*	internalGetResultOut()
	{
		return m_resultOut;
	}

	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap)
		{
			object mem = ci.m_dispatcher1.allocateCollisionAlgorithm(sizeof(btGImpactCollisionAlgorithm));
			return new(mem) btGImpactCollisionAlgorithm(ci,body0Wrap,body1Wrap);
		}
	};

	//! Use this function for register the algorithm externally
	static void registerAlgorithm(btCollisionDispatcher * dispatcher);
#if TRI_COLLISION_PROFILING
	//! Gets the average time in miliseconds of tree collisions
	static float getAverageTreeCollisionTime();

	//! Gets the average time in miliseconds of triangle collisions
	static float getAverageTriangleCollisionTime();
#endif //TRI_COLLISION_PROFILING

	//! Collides two gimpact shapes
	/*!
	\pre shape0 and shape1 couldn't be btGImpactMeshShape objects
	*/


	void gimpact_vs_gimpact(btCollisionObjectWrapper* body0Wrap,
					  btCollisionObjectWrapper * body1Wrap,
					  btGImpactShapeInterface * shape0,
					  btGImpactShapeInterface * shape1);

	void gimpact_vs_shape(btCollisionObjectWrapper* body0Wrap,
					  btCollisionObjectWrapper* body1Wrap,
					  btGImpactShapeInterface * shape0,
					  btCollisionShape * shape1,bool swapped);

	void gimpact_vs_compoundshape(btCollisionObjectWrapper * body0Wrap,
					  btCollisionObjectWrapper * body1Wrap,
					  btGImpactShapeInterface * shape0,
					  btCompoundShape * shape1,bool swapped);

	void gimpact_vs_concave(
					  btCollisionObjectWrapper * body0Wrap,
					  btCollisionObjectWrapper * body1Wrap,
					  btGImpactShapeInterface * shape0,
					  btConcaveShape * shape1,bool swapped);




		/// Accessor/Mutator pairs for Part and triangleID
    void 	setFace0(int value) 
    { 
    	m_triface0 = value; 
    }
    int getFace0() 
    { 
    	return m_triface0; 
    }
    void setFace1(int value) 
    { 
    	m_triface1 = value; 
    }
    int getFace1() 
    { 
    	return m_triface1; 
    }
    void setPart0(int value) 
    { 
    	m_part0 = value; 
    }
    int getPart0() 
    { 
    	return m_part0; 
    }
    void setPart1(int value) 
    { 
    	m_part1 = value; 
		}
    int getPart1() 
    { 
    	return m_part1; 
    }

};


//algorithm details
//#define BULLET_TRIANGLE_COLLISION 1


}
