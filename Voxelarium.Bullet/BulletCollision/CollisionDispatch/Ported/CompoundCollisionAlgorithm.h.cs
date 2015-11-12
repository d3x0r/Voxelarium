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

using Bullet.Collision.BroadPhase;
using Bullet.Collision.NarrowPhase;
using Bullet.Collision.Shapes;
using Bullet.LinearMath;
using Bullet.Types;
using System.Diagnostics;
using System;

namespace Bullet.Collision.Dispatch
{

	delegate bool btShapePairCallback( btCollisionShape pShape0, btCollisionShape pShape1 );

	/// btCompoundCollisionAlgorithm  supports collision between CompoundCollisionShapes and other collision shapes
	internal class btCompoundCollisionAlgorithm : btActivatingCollisionAlgorithm
	{
		public static btShapePairCallback gCompoundChildShapePairCallback = null;

		protected btList<btCollisionAlgorithm> m_childCollisionAlgorithms = new btList<btCollisionAlgorithm>();
		protected bool m_isSwapped;

		protected btPersistentManifold m_sharedManifold;
		protected bool m_ownsManifold;


		protected int m_compoundShapeRevision;//to keep track of changes, so that childAlgorithm array can be updated


		btCollisionAlgorithm getChildAlgorithm( int n )
		{
			return m_childCollisionAlgorithms[n];
		}



		internal override  void getAllContactManifolds( btManifoldArray manifoldArray )
		{
			int i;
			for( i = 0; i < m_childCollisionAlgorithms.Count; i++ )
			{
				if( m_childCollisionAlgorithms[i] != null )
					m_childCollisionAlgorithms[i].getAllContactManifolds( manifoldArray );
			}
		}


		internal class CreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo  ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btCompoundCollisionAlgorithm ca = BulletGlobals.CompoundCollisionAlgorithmPool.Get();
				ca.Initialize( ci, body0Wrap, body1Wrap, false );
				return ca;
			}
		};

		internal class SwappedCreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo  ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btCompoundCollisionAlgorithm ca = BulletGlobals.CompoundCollisionAlgorithmPool.Get();
				ca.Initialize( ci, body0Wrap, body1Wrap, true );
				return ca;
			}
		};

		internal override void Cleanup()
		{
			BulletGlobals.CompoundCollisionAlgorithmPool.Free( this );
		}

		public btCompoundCollisionAlgorithm() { }

		internal void Initialize( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, bool isSwapped )
		{
			base.Initialize( ci, body0Wrap, body1Wrap );
			m_isSwapped = ( isSwapped );
			m_sharedManifold = ( ci.m_manifold );
			m_ownsManifold = false;

			btCollisionObjectWrapper colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
			Debug.Assert( colObjWrap.getCollisionShape().isCompound() );

			btCompoundShape compoundShape = (btCompoundShape)( colObjWrap.getCollisionShape() );
			m_compoundShapeRevision = compoundShape.getUpdateRevision();


			preallocateChildAlgorithms( body0Wrap, body1Wrap );
		}

		void preallocateChildAlgorithms( btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
		{
			btCollisionObjectWrapper colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
			btCollisionObjectWrapper otherObjWrap = m_isSwapped ? body0Wrap : body1Wrap;
			Debug.Assert( colObjWrap.getCollisionShape().isCompound() );

			btCompoundShape compoundShape = (btCompoundShape)( colObjWrap.getCollisionShape() );

			int numChildren = compoundShape.getNumChildShapes();
			int i;

			m_childCollisionAlgorithms.Capacity = ( numChildren );
			for( i = 0; i < numChildren; i++ )
			{
				if( compoundShape.getDynamicAabbTree() != null )
				{
					m_childCollisionAlgorithms[i] = null;
				}
				else
				{

					btCollisionShape childShape = compoundShape.getChildShape( i );

					btCollisionObjectWrapper childWrap = BulletGlobals.CollisionObjectWrapperPool.Get();
					childWrap.Initialize( colObjWrap, childShape, colObjWrap.m_collisionObject, ref colObjWrap.m_worldTransform, -1, i );//wrong child trans, but unused (hopefully)
					m_childCollisionAlgorithms[i] = m_dispatcher.findAlgorithm( childWrap, otherObjWrap, m_sharedManifold );
				}
			}
		}

		void removeChildAlgorithms()
		{
			int numChildren = m_childCollisionAlgorithms.Count;
			int i;
			for( i = 0; i < numChildren; i++ )
			{
				if( m_childCollisionAlgorithms[i] != null )
				{
					//m_childCollisionAlgorithms[i].~btCollisionAlgorithm();
					m_dispatcher.freeCollisionAlgorithm( m_childCollisionAlgorithms[i] );
				}
			}
		}

		~btCompoundCollisionAlgorithm()
		{
			removeChildAlgorithms();
		}




		internal class btCompoundLeafCallback : btDbvt.DefaultCollide
		{


			internal btCollisionObjectWrapper m_compoundColObjWrap;
			internal btCollisionObjectWrapper m_otherObjWrap;
			internal btDispatcher m_dispatcher;
			internal btDispatcherInfo m_dispatchInfo;
			internal btManifoldResult m_resultOut;
			internal btCollisionAlgorithm[] m_childCollisionAlgorithms;
			internal btPersistentManifold m_sharedManifold;

			public btCompoundLeafCallback() { }

			internal void Initialize( btCollisionObjectWrapper compoundObjWrap, btCollisionObjectWrapper otherObjWrap, btDispatcher dispatcher, btDispatcherInfo dispatchInfo, btManifoldResult resultOut, btCollisionAlgorithm[] childCollisionAlgorithms, btPersistentManifold sharedManifold )
			{
				m_compoundColObjWrap = ( compoundObjWrap );
				m_otherObjWrap = ( otherObjWrap );
				m_dispatcher = ( dispatcher );
				m_dispatchInfo = ( dispatchInfo );
				m_resultOut = ( resultOut );
				m_childCollisionAlgorithms = ( childCollisionAlgorithms );
				m_sharedManifold = ( sharedManifold );

			}


			public void ProcessChildShape( btCollisionShape childShape, int index )
			{
				Debug.Assert( index >= 0 );
				btCompoundShape compoundShape = (btCompoundShape)( m_compoundColObjWrap.getCollisionShape() );
				Debug.Assert( index < compoundShape.getNumChildShapes() );


				//backup
				//btTransform orgTrans = m_compoundColObjWrap.getWorldTransform();

				//btTransform childTrans = compoundShape.getChildTransform( index );
				btTransform newChildWorldTrans; m_compoundColObjWrap.m_worldTransform.Apply( ref compoundShape.m_children.InternalArray[index].m_transform
								, out newChildWorldTrans );

				//perform an AABB check first
				btVector3 aabbMin0, aabbMax0, aabbMin1, aabbMax1;
				childShape.getAabb( ref newChildWorldTrans, out aabbMin0, out aabbMax0 );
				m_otherObjWrap.getCollisionShape().getAabb( ref m_otherObjWrap.m_worldTransform
					, out aabbMin1, out aabbMax1 );

				if( gCompoundChildShapePairCallback != null )
				{
					if( !gCompoundChildShapePairCallback( m_otherObjWrap.getCollisionShape(), childShape ) )
						return;
				}

				if( btAabbUtil.TestAabbAgainstAabb2( ref aabbMin0, ref aabbMax0, ref aabbMin1, ref aabbMax1 ) )
				{

					btCollisionObjectWrapper compoundWrap = BulletGlobals.CollisionObjectWrapperPool.Get();
					compoundWrap.Initialize( this.m_compoundColObjWrap, childShape, m_compoundColObjWrap.m_collisionObject, ref newChildWorldTrans, -1, index);


					//the contactpoint is still projected back using the original inverted worldtrans
					if( m_childCollisionAlgorithms[index] == null )
						m_childCollisionAlgorithms[index] = m_dispatcher.findAlgorithm( compoundWrap, m_otherObjWrap, m_sharedManifold );


					btCollisionObjectWrapper tmpWrap = null;

					///detect swapping case
					if( m_resultOut.getBody0Internal() == m_compoundColObjWrap.m_collisionObject )
					{
						tmpWrap = m_resultOut.m_body0Wrap;
						m_resultOut.m_body0Wrap = compoundWrap;
						m_resultOut.setShapeIdentifiersA( -1, index );
					}
					else
					{
						tmpWrap = m_resultOut.m_body1Wrap;
						m_resultOut.m_body1Wrap =( compoundWrap );
						m_resultOut.setShapeIdentifiersB( -1, index );
					}


					m_childCollisionAlgorithms[index].processCollision( compoundWrap, m_otherObjWrap, m_dispatchInfo, m_resultOut );

#if false
			if (m_dispatchInfo.m_debugDraw && (m_dispatchInfo.m_debugDraw.getDebugMode() & btIDebugDraw::DBG_DrawAabb))
			{
				btVector3 worldAabbMin,worldAabbMax;
				m_dispatchInfo.m_debugDraw.drawAabb(aabbMin0,aabbMax0,btVector3(1,1,1));
				m_dispatchInfo.m_debugDraw.drawAabb(aabbMin1,aabbMax1,btVector3(1,1,1));
			}
#endif

					if( m_resultOut.getBody0Internal() == m_compoundColObjWrap.m_collisionObject )
					{
						m_resultOut.m_body0Wrap = tmpWrap;
					}
					else
					{
						m_resultOut.m_body1Wrap =( tmpWrap );
					}

				}
			}
			public override void Process( btDbvt.btDbvtNode leaf )
			{
				int index = leaf.dataAsInt;

				btCompoundShape compoundShape = (btCompoundShape)( m_compoundColObjWrap.getCollisionShape() );
				btCollisionShape childShape = compoundShape.getChildShape( index );

#if false
		if (m_dispatchInfo.m_debugDraw && (m_dispatchInfo.m_debugDraw.getDebugMode() & btIDebugDraw::DBG_DrawAabb))
		{
			btVector3 worldAabbMin,worldAabbMax;
			btTransform	orgTrans = m_compoundColObjWrap.getWorldTransform();
			btTransformAabb(leaf.volume.Mins(),leaf.volume.Maxs(),0.,orgTrans,worldAabbMin,worldAabbMax);
			m_dispatchInfo.m_debugDraw.drawAabb(worldAabbMin,worldAabbMax,btVector3(1,0,0));
		}
#endif

				ProcessChildShape( childShape, index );

			}
		};






		internal override void processCollision( btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			btCollisionObjectWrapper colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
			btCollisionObjectWrapper otherObjWrap = m_isSwapped ? body0Wrap : body1Wrap;

			Debug.Assert( colObjWrap.getCollisionShape().isCompound() );
			btCompoundShape compoundShape = (btCompoundShape)( colObjWrap.getCollisionShape() );

			///btCompoundShape might have changed:
			////make sure the internal child collision algorithm caches are still valid
			if( compoundShape.getUpdateRevision() != m_compoundShapeRevision )
			{
				///clear and update all
				removeChildAlgorithms();

				preallocateChildAlgorithms( body0Wrap, body1Wrap );
				m_compoundShapeRevision = compoundShape.getUpdateRevision();
			}

			if( m_childCollisionAlgorithms.Count == 0 )
				return;

			btDbvt tree = compoundShape.getDynamicAabbTree();
			//use a dynamic aabb tree to cull potential child-overlaps
			btCompoundLeafCallback callback = BulletGlobals.CompoundLeafCallbackPool.Get();
			callback.Initialize( colObjWrap, otherObjWrap, m_dispatcher, dispatchInfo, resultOut, m_childCollisionAlgorithms.InternalArray, m_sharedManifold);

			///we need to refresh all contact manifolds
			///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
			///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
			{
				int i;
				btManifoldArray manifoldArray = new btManifoldArray();
				for( i = 0; i < m_childCollisionAlgorithms.Count; i++ )
				{
					if( m_childCollisionAlgorithms[i] != null )
					{
						m_childCollisionAlgorithms[i].getAllContactManifolds( manifoldArray );
						for( int m = 0; m < manifoldArray.Count; m++ )
						{
							if( manifoldArray[m].m_cachedPoints > 0 )
							{
								resultOut.setPersistentManifold( manifoldArray[m] );
								resultOut.refreshContactPoints();
								resultOut.setPersistentManifold( null );//??necessary?
							}
						}
						manifoldArray.Count = ( 0 );
					}
				}
			}

			if( tree != null )
			{

				btVector3 localAabbMin, localAabbMax;
				btTransform otherInCompoundSpace;

				colObjWrap.m_worldTransform.inverseTimes(ref otherObjWrap.m_worldTransform, out otherInCompoundSpace);
				otherObjWrap.getCollisionShape().getAabb( ref otherInCompoundSpace, out localAabbMin, out localAabbMax );

				 btDbvt.btDbvtVolume  bounds = btDbvt.btDbvtVolume.FromMM( ref localAabbMin, ref localAabbMax );
				//process all children, that overlap with  the given AABB bounds
				btDbvt.CollideTV( tree.m_root, ref bounds, callback );

			}
			else
			{
				//iterate over all children, perform an AABB check inside ProcessChildShape
				int numChildren = m_childCollisionAlgorithms.Count;
				int i;
				for( i = 0; i < numChildren; i++ )
				{
					callback.ProcessChildShape( compoundShape.getChildShape( i ), i );
				}
			}

			{
				//iterate over all children, perform an AABB check inside ProcessChildShape
				int numChildren = m_childCollisionAlgorithms.Count;
				int i;
				//btManifoldArray manifoldArray;
				btCollisionShape childShape = null;
				//btITransform orgTrans;

				btTransform newChildWorldTrans;
				btVector3 aabbMin0, aabbMax0, aabbMin1, aabbMax1;

				for( i = 0; i < numChildren; i++ )
				{
					if( m_childCollisionAlgorithms[i] != null )
					{
						childShape = compoundShape.getChildShape( i );
						//if not longer overlapping, remove the algorithm
						//orgTrans = colObjWrap.m_worldTransform;

						//btTransform childTrans = compoundShape.getChildTransform( i );
						colObjWrap.m_worldTransform.Apply( ref compoundShape.m_children.InternalArray[i].m_transform, out newChildWorldTrans );

						//perform an AABB check first
						childShape.getAabb( ref newChildWorldTrans, out aabbMin0, out aabbMax0 );
						otherObjWrap.getCollisionShape().getAabb( ref otherObjWrap.m_worldTransform, out aabbMin1, out aabbMax1 );

						if( !btAabbUtil.TestAabbAgainstAabb2( ref aabbMin0, ref aabbMax0, ref aabbMin1, ref aabbMax1 ) )
						{
							//m_childCollisionAlgorithms[i].~btCollisionAlgorithm();
							m_dispatcher.freeCollisionAlgorithm( m_childCollisionAlgorithms[i] );
							m_childCollisionAlgorithms[i] = null;
						}
					}
				}
			}
		}

		internal override double calculateTimeOfImpact( btCollisionObject body0, btCollisionObject body1, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			Debug.Assert( false );
			//needs to be fixed, using btCollisionObjectWrapper and NOT modifying internal data structures
			btCollisionObject colObj = m_isSwapped ? body1 : body0;
			btCollisionObject otherObj = m_isSwapped ? body0 : body1;

			Debug.Assert( colObj.getCollisionShape().isCompound() );

			btCompoundShape compoundShape = (btCompoundShape)( colObj.getCollisionShape() );

			//We will use the OptimizedBVH, AABB tree to cull potential child-overlaps
			//If both proxies are Compound, we will deal with that directly, by performing sequential/parallel tree traversals
			//given Proxy0 and Proxy1, if both have a tree, Tree0 and Tree1, this means:
			//determine overlapping nodes of Proxy1 using Proxy0 AABB against Tree1
			//then use each overlapping node AABB against Tree0
			//and vise versa.

			double hitFraction = btScalar.BT_ONE;

			int numChildren = m_childCollisionAlgorithms.Count;
			int i;
			//btTransform orgTrans;
			double frac;
			for( i = 0; i < numChildren; i++ )
			{
				//btCollisionShape  childShape = compoundShape.getChildShape(i);

				//backup
				//orgTrans = colObj.m_worldTransform;

				btTransform childTrans = compoundShape.getChildTransform( i );
				//btTransform	newChildWorldTrans = orgTrans*childTrans ;
				//colObj.setWorldTransform( orgTrans * childTrans );
				btTransform tmp;
				colObj.m_worldTransform.Apply( ref childTrans, out tmp );
				colObj.m_worldTransform = tmp;
				//btCollisionShape  tmpShape = colObj.getCollisionShape();
				//colObj.internalSetTemporaryCollisionShape( childShape );
				frac = m_childCollisionAlgorithms[i].calculateTimeOfImpact( colObj, otherObj, dispatchInfo, resultOut );
				if( frac < hitFraction )
				{
					hitFraction = frac;
				}
				//revert back
				//colObj.internalSetTemporaryCollisionShape( tmpShape);
				colObj.setWorldTransform( ref colObj.m_worldTransform );
			}
			return hitFraction;

		}


	};

}