/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#include "btCompoundCompoundCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btAabbUtil2.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"


btShapePairCallback gCompoundCompoundChildShapePairCallback = 0;

btCompoundCompoundCollisionAlgorithm::btCompoundCompoundCollisionAlgorithm( btCollisionAlgorithmConstructionInfo& ci,btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap,bool isSwapped)
:btCompoundCollisionAlgorithm(ci,body0Wrap,body1Wrap,isSwapped)
{

	object ptr = btAlignedAlloc(sizeof(btHashedSimplePairCache),16);
	m_childCollisionAlgorithmCache= new(ptr) btHashedSimplePairCache();

	btCollisionObjectWrapper* col0ObjWrap = body0Wrap;
	Debug.Assert (col0ObjWrap.getCollisionShape().isCompound());

	btCollisionObjectWrapper* col1ObjWrap = body1Wrap;
	Debug.Assert (col1ObjWrap.getCollisionShape().isCompound());
	
	btCompoundShape* compoundShape0 = static_cast<btCompoundShape*>(col0ObjWrap.getCollisionShape());
	m_compoundShapeRevision0 = compoundShape0.getUpdateRevision();

	btCompoundShape* compoundShape1 = static_cast<btCompoundShape*>(col1ObjWrap.getCollisionShape());
	m_compoundShapeRevision1 = compoundShape1.getUpdateRevision();
	
	
}


btCompoundCompoundCollisionAlgorithm::~btCompoundCompoundCollisionAlgorithm()
{
	removeChildAlgorithms();
	m_childCollisionAlgorithmCache.~btHashedSimplePairCache();
	btAlignedFree(m_childCollisionAlgorithmCache);
}

void	btCompoundCompoundCollisionAlgorithm::getAllContactManifolds(btManifoldArray&	manifoldArray)
{
	int i;
	btSimplePairArray& pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();
	for (i=0;i<pairs.Count;i++)
	{
		if (pairs[i].m_userPointer)
		{
			
			((btCollisionAlgorithm*)pairs[i].m_userPointer).getAllContactManifolds(manifoldArray);
		}
	}
}


void	btCompoundCompoundCollisionAlgorithm::removeChildAlgorithms()
{
	btSimplePairArray& pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();

	int numChildren = pairs.Count;
	int i;
	for (i=0;i<numChildren;i++)
	{
		if (pairs[i].m_userPointer)
		{
			btCollisionAlgorithm* algo = (btCollisionAlgorithm*) pairs[i].m_userPointer;
			algo.~btCollisionAlgorithm();
			m_dispatcher.freeCollisionAlgorithm(algo);
		}
	}
	m_childCollisionAlgorithmCache.removeAllPairs();
}

struct	btCompoundCompoundLeafCallback : btDbvt.ICollide
{
	int m_numOverlapPairs;


	btCollisionObjectWrapper* m_compound0ColObjWrap;
	btCollisionObjectWrapper* m_compound1ColObjWrap;
	btDispatcher* m_dispatcher;
	btDispatcherInfo m_dispatchInfo;
	btManifoldResult*	m_resultOut;
	
	
	class btHashedSimplePairCache*	m_childCollisionAlgorithmCache;
	
	btPersistentManifold*	m_sharedManifold;
	
	btCompoundCompoundLeafCallback (btCollisionObjectWrapper* compound1ObjWrap,
									btCollisionObjectWrapper* compound0ObjWrap,
									btDispatcher* dispatcher,
									btDispatcherInfo& dispatchInfo,
									btManifoldResult*	resultOut,
									btHashedSimplePairCache* childAlgorithmsCache,
									btPersistentManifold*	sharedManifold)
		:m_numOverlapPairs(0),m_compound0ColObjWrap(compound1ObjWrap),m_compound1ColObjWrap(compound0ObjWrap),m_dispatcher(dispatcher),m_dispatchInfo(dispatchInfo),m_resultOut(resultOut),
		m_childCollisionAlgorithmCache(childAlgorithmsCache),
		m_sharedManifold(sharedManifold)
	{

	}



	
	void		Process(btDbvtNode* leaf0,btDbvtNode* leaf1)
	{
		m_numOverlapPairs++;


		int childIndex0 = leaf0.dataAsInt;
		int childIndex1 = leaf1.dataAsInt;
		

		Debug.Assert(childIndex0>=0);
		Debug.Assert(childIndex1>=0);


		btCompoundShape* compoundShape0 = static_cast<btCompoundShape*>(m_compound0ColObjWrap.getCollisionShape());
		Debug.Assert(childIndex0<compoundShape0.getNumChildShapes());

		btCompoundShape* compoundShape1 = static_cast<btCompoundShape*>(m_compound1ColObjWrap.getCollisionShape());
		Debug.Assert(childIndex1<compoundShape1.getNumChildShapes());

		btCollisionShape* childShape0 = compoundShape0.getChildShape(childIndex0);
		btCollisionShape* childShape1 = compoundShape1.getChildShape(childIndex1);

		//backup
		btTransform	orgTrans0 = m_compound0ColObjWrap.getWorldTransform();
		ref btTransform childTrans0 = compoundShape0.getChildTransform(childIndex0);
		btTransform	newChildWorldTrans0 = orgTrans0*childTrans0 ;
		
		btTransform	orgTrans1 = m_compound1ColObjWrap.getWorldTransform();
		ref btTransform childTrans1 = compoundShape1.getChildTransform(childIndex1);
		btTransform	newChildWorldTrans1 = orgTrans1*childTrans1 ;
		

		//perform an AABB check first
		btVector3 aabbMin0,aabbMax0,aabbMin1,aabbMax1;
		childShape0.getAabb(newChildWorldTrans0,aabbMin0,aabbMax0);
		childShape1.getAabb(newChildWorldTrans1,aabbMin1,aabbMax1);
		
		if (gCompoundCompoundChildShapePairCallback)
		{
			if (!gCompoundCompoundChildShapePairCallback(childShape0,childShape1))
				return;
		}

		if (TestAabbAgainstAabb2(aabbMin0,aabbMax0,aabbMin1,aabbMax1))
		{
			btCollisionObjectWrapper compoundWrap0(this.m_compound0ColObjWrap,childShape0, m_compound0ColObjWrap.getCollisionObject(),newChildWorldTrans0,-1,childIndex0);
			btCollisionObjectWrapper compoundWrap1(this.m_compound1ColObjWrap,childShape1,m_compound1ColObjWrap.getCollisionObject(),newChildWorldTrans1,-1,childIndex1);
			

			btSimplePair* pair = m_childCollisionAlgorithmCache.findPair(childIndex0,childIndex1);

			btCollisionAlgorithm* colAlgo = 0;

			if (pair)
			{
				colAlgo = (btCollisionAlgorithm*)pair.m_userPointer;
				
			} else
			{
				colAlgo = m_dispatcher.findAlgorithm(&compoundWrap0,&compoundWrap1,m_sharedManifold);
				pair = m_childCollisionAlgorithmCache.addOverlappingPair(childIndex0,childIndex1);
				Debug.Assert(pair);
				pair.m_userPointer = colAlgo;
			}

			Debug.Assert(colAlgo);
						
			btCollisionObjectWrapper* tmpWrap0 = 0;
			btCollisionObjectWrapper* tmpWrap1 = 0;

			tmpWrap0 = m_resultOut.getBody0Wrap();
			tmpWrap1 = m_resultOut.getBody1Wrap();

			m_resultOut.setBody0Wrap(&compoundWrap0);
			m_resultOut.setBody1Wrap(&compoundWrap1);

			m_resultOut.setShapeIdentifiersA(-1,childIndex0);
			m_resultOut.setShapeIdentifiersB(-1,childIndex1);


			colAlgo.processCollision(&compoundWrap0,&compoundWrap1,m_dispatchInfo,m_resultOut);
			
			m_resultOut.setBody0Wrap(tmpWrap0);
			m_resultOut.setBody1Wrap(tmpWrap1);
			


		}
	}
};


static DBVT_INLINE bool		MyIntersect(	btDbvtAabbMm& a,
								  btDbvtAabbMm& b, ref btTransform xform)
{
	btVector3 newmin,newmax;
	btTransformAabb(b.Mins(),b.Maxs(),0,xform,newmin,newmax);
	btDbvtAabbMm newb = btDbvtAabbMm::FromMM(newmin,newmax);
	return Intersect(a,newb);
}


static inline void		MycollideTT(	btDbvtNode* root0,
								  btDbvtNode* root1,
								  ref btTransform xform,
								  btCompoundCompoundLeafCallback* callback)
{

		if(root0&&root1)
		{
			int								depth=1;
			int								treshold=btDbvt::DOUBLE_STACKSIZE-4;
			List<btDbvt::sStkNN>	stkStack;
			stkStack.resize(btDbvt::DOUBLE_STACKSIZE);
			stkStack=btDbvt::sStkNN(root0,root1);
			do	{
				btDbvt::sStkNN	p=stkStack[--depth];
				if(MyIntersect(p.a.volume,p.b.volume,xform))
				{
					if(depth>treshold)
					{
						stkStack.resize(stkStack.Count*2);
						treshold=stkStack.Count-4;
					}
					if(p.a.isinternal())
					{
						if(p.b.isinternal())
						{					
							stkStack[depth++]=btDbvt::sStkNN(p.a.childs[0],p.b.childs[0]);
							stkStack[depth++]=btDbvt::sStkNN(p.a.childs[1],p.b.childs[0]);
							stkStack[depth++]=btDbvt::sStkNN(p.a.childs[0],p.b.childs[1]);
							stkStack[depth++]=btDbvt::sStkNN(p.a.childs[1],p.b.childs[1]);
						}
						else
						{
							stkStack[depth++]=btDbvt::sStkNN(p.a.childs[0],p.b);
							stkStack[depth++]=btDbvt::sStkNN(p.a.childs[1],p.b);
						}
					}
					else
					{
						if(p.b.isinternal())
						{
							stkStack[depth++]=btDbvt::sStkNN(p.a,p.b.childs[0]);
							stkStack[depth++]=btDbvt::sStkNN(p.a,p.b.childs[1]);
						}
						else
						{
							callback.Process(p.a,p.b);
						}
					}
				}
			} while(depth);
		}
}

void btCompoundCompoundCollisionAlgorithm::processCollision (btCollisionObjectWrapper* body0Wrap,btCollisionObjectWrapper* body1Wrap,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{

	btCollisionObjectWrapper* col0ObjWrap = body0Wrap;
	btCollisionObjectWrapper* col1ObjWrap= body1Wrap;

	Debug.Assert (col0ObjWrap.getCollisionShape().isCompound());
	Debug.Assert (col1ObjWrap.getCollisionShape().isCompound());
	btCompoundShape* compoundShape0 = static_cast<btCompoundShape*>(col0ObjWrap.getCollisionShape());
	btCompoundShape* compoundShape1 = static_cast<btCompoundShape*>(col1ObjWrap.getCollisionShape());

	btDbvt* tree0 = compoundShape0.getDynamicAabbTree();
	btDbvt* tree1 = compoundShape1.getDynamicAabbTree();
	if (!tree0 || !tree1)
	{
		return btCompoundCollisionAlgorithm::processCollision(body0Wrap,body1Wrap,dispatchInfo,resultOut);
	}
	///btCompoundShape might have changed:
	////make sure the internal child collision algorithm caches are still valid
	if ((compoundShape0.getUpdateRevision() != m_compoundShapeRevision0) || (compoundShape1.getUpdateRevision() != m_compoundShapeRevision1))
	{
		///clear all
		removeChildAlgorithms();
		m_compoundShapeRevision0 = compoundShape0.getUpdateRevision();
		m_compoundShapeRevision1 = compoundShape1.getUpdateRevision();

	}


	///we need to refresh all contact manifolds
	///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
	///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
	{
		int i;
		btManifoldArray manifoldArray;
		btSimplePairArray& pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();
		for (i=0;i<pairs.Count;i++)
		{
			if (pairs[i].m_userPointer)
			{
				btCollisionAlgorithm* algo = (btCollisionAlgorithm*) pairs[i].m_userPointer;
				algo.getAllContactManifolds(manifoldArray);
				for (int m=0;m<manifoldArray.Count;m++)
				{
					if (manifoldArray[m].getNumContacts())
					{
						resultOut.setPersistentManifold(manifoldArray[m]);
						resultOut.refreshContactPoints();
						resultOut.setPersistentManifold(0);
					}
				}
				manifoldArray.resize(0);
			}
		}
	}


	

	btCompoundCompoundLeafCallback callback(col0ObjWrap,col1ObjWrap,this.m_dispatcher,dispatchInfo,resultOut,this.m_childCollisionAlgorithmCache,m_sharedManifold);


	btTransform	xform=col0ObjWrap.getWorldTransform().inverse()*col1ObjWrap.getWorldTransform();
	MycollideTT(tree0.m_root,tree1.m_root,xform,&callback);

	//Console.WriteLine("#compound-compound child/leaf overlap =%d                      \r",callback.m_numOverlapPairs);

	//remove non-overlapping child pairs

	{
		Debug.Assert(m_removePairs.Count==0);

		//iterate over all children, perform an AABB check inside ProcessChildShape
		btSimplePairArray& pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();
		
		int i;
		btManifoldArray	manifoldArray;
        
		

        
        
        btVector3 aabbMin0,aabbMax0,aabbMin1,aabbMax1;        
        
		for (i=0;i<pairs.Count;i++)
		{
			if (pairs[i].m_userPointer)
			{
				btCollisionAlgorithm* algo = (btCollisionAlgorithm*)pairs[i].m_userPointer;

				{
					btTransform	orgTrans0;
					btCollisionShape* childShape0 = 0;
					
					btTransform	newChildWorldTrans0;
					btTransform	orgInterpolationTrans0;
					childShape0 = compoundShape0.getChildShape(pairs[i].m_indexA);
					orgTrans0 = col0ObjWrap.getWorldTransform();
					orgInterpolationTrans0 = col0ObjWrap.getWorldTransform();
					ref btTransform childTrans0 = compoundShape0.getChildTransform(pairs[i].m_indexA);
					newChildWorldTrans0 = orgTrans0*childTrans0 ;
					childShape0.getAabb(newChildWorldTrans0,aabbMin0,aabbMax0);
				}

				{
					btTransform	orgInterpolationTrans1;
					btCollisionShape* childShape1 = 0;
					btTransform	orgTrans1;
					btTransform	newChildWorldTrans1;

					childShape1 = compoundShape1.getChildShape(pairs[i].m_indexB);
					orgTrans1 = col1ObjWrap.getWorldTransform();
					orgInterpolationTrans1 = col1ObjWrap.getWorldTransform();
					ref btTransform childTrans1 = compoundShape1.getChildTransform(pairs[i].m_indexB);
					newChildWorldTrans1 = orgTrans1*childTrans1 ;
					childShape1.getAabb(newChildWorldTrans1,aabbMin1,aabbMax1);
				}
				
				

				if (!TestAabbAgainstAabb2(aabbMin0,aabbMax0,aabbMin1,aabbMax1))
				{
					algo.~btCollisionAlgorithm();
					m_dispatcher.freeCollisionAlgorithm(algo);
					m_removePairs.Add(btSimplePair(pairs[i].m_indexA,pairs[i].m_indexB));
				}
			}
		}
		for (int i=0;i<m_removePairs.Count;i++)
		{
			m_childCollisionAlgorithmCache.removeOverlappingPair(m_removePairs[i].m_indexA,m_removePairs[i].m_indexB);
		}
		m_removePairs.clear();
	}

}

double	btCompoundCompoundCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject body0,btCollisionObject body1,btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	Debug.Assert(false);
	return 0;

}



