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

namespace BUllet.BulletCollision
{

	/// btSphereBoxCollisionAlgorithm  provides sphere-box collision detection.
	/// Other features are frame-coherency (persistent data) and collision response.
	class btConvexPlaneCollisionAlgorithm : btCollisionAlgorithm
{
	bool		m_ownManifold;
	btPersistentManifold*	m_manifoldPtr;
	bool		m_isSwapped;
	int			m_numPerturbationIterations;
	int			m_minimumPointsPerturbationThreshold;

public:

	btConvexPlaneCollisionAlgorithm(btPersistentManifold* mf,btCollisionAlgorithmConstructionInfo& ci,btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap, bool isSwapped, int numPerturbationIterations,int minimumPointsPerturbationThreshold);

	virtual ~btConvexPlaneCollisionAlgorithm();

	virtual void processCollision (btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	void collideSingleContact (btQuaternion& perturbeRot, btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual double calculateTimeOfImpact(btCollisionObject body0,btCollisionObject body1,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.Add(m_manifoldPtr);
		}
	}

	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		int	m_numPerturbationIterations;
		int m_minimumPointsPerturbationThreshold;
			
		CreateFunc() 
			: m_numPerturbationIterations(1),
			m_minimumPointsPerturbationThreshold(0)
		{
		}
		
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap)
		{
			object mem = ci.m_dispatcher1.allocateCollisionAlgorithm(sizeof(btConvexPlaneCollisionAlgorithm));
			if (!m_swapped)
			{
				return new(mem) btConvexPlaneCollisionAlgorithm(0,ci,body0Wrap,body1Wrap,false,m_numPerturbationIterations,m_minimumPointsPerturbationThreshold);
			} else
			{
				return new(mem) btConvexPlaneCollisionAlgorithm(0,ci,body0Wrap,body1Wrap,true,m_numPerturbationIterations,m_minimumPointsPerturbationThreshold);
			}
		}
	};

};

}
