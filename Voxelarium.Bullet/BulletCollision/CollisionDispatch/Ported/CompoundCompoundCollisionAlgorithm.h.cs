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

using Bullet.Collision.BroadPhase;
using Bullet.Collision.NarrowPhase;
using Bullet.Collision.Shapes;
using Bullet.LinearMath;
using Bullet.Types;
using System.Diagnostics;

namespace Bullet.Collision.Dispatch
{


	/// btCompoundCompoundCollisionAlgorithm  supports collision between two btCompoundCollisionShape shapes
	public class btCompoundCompoundCollisionAlgorithm : btCompoundCollisionAlgorithm
	{
		public delegate bool btShapePairCallback( btCollisionShape pShape0, btCollisionShape pShape1 );
		public static btShapePairCallback gCompoundCompoundChildShapePairCallback;

		btHashedSimplePairCache m_childCollisionAlgorithmCache;
		btSimplePairArray m_removePairs = new btSimplePairArray();


		int m_compoundShapeRevision0;//to keep track of changes, so that childAlgorithm array can be updated
		int m_compoundShapeRevision1;

		//void	removeChildAlgorithms();

		//	void	preallocateChildAlgorithms(btCollisionObjectWrapper  body0Wrap,btCollisionObjectWrapper  body1Wrap);

		//public:

		//btCompoundCompoundCollisionAlgorithm( btCollisionAlgorithmConstructionInfo  ci,btCollisionObjectWrapper  body0Wrap,btCollisionObjectWrapper  body1Wrap,bool isSwapped);

		//virtual ~btCompoundCompoundCollisionAlgorithm();



		//virtual void processCollision( btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, btDispatcherInfo dispatchInfo, btManifoldResult resultOut );

		//double calculateTimeOfImpact( btCollisionObject body0, btCollisionObject body1, btDispatcherInfo dispatchInfo, btManifoldResult resultOut );

		//virtual void getAllContactManifolds( btManifoldArray&	manifoldArray);


		new internal class CreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btCompoundCompoundCollisionAlgorithm ca = BulletGlobals.CompoundCompoundCollisionAlgoritmPool.Get();
				ca.Initialize( ci, body0Wrap, body1Wrap, false );
				return ca;
			}
		};

		internal class sSwappedCreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btCompoundCompoundCollisionAlgorithm ca = BulletGlobals.CompoundCompoundCollisionAlgoritmPool.Get();
				ca.Initialize( ci, body0Wrap, body1Wrap, true );
				return ca;
			}
		};

		public btCompoundCompoundCollisionAlgorithm() { }

		new internal void Initialize( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, bool isSwapped )
		{
			base.Initialize( ci, body0Wrap, body1Wrap, isSwapped );

			m_childCollisionAlgorithmCache = new btHashedSimplePairCache();

			btCollisionObjectWrapper col0ObjWrap = body0Wrap;
			Debug.Assert( col0ObjWrap.getCollisionShape().isCompound() );

			btCollisionObjectWrapper col1ObjWrap = body1Wrap;
			Debug.Assert( col1ObjWrap.getCollisionShape().isCompound() );

			btCompoundShape compoundShape0 = (btCompoundShape)( col0ObjWrap.getCollisionShape() );
			m_compoundShapeRevision0 = compoundShape0.getUpdateRevision();

			btCompoundShape compoundShape1 = (btCompoundShape)( col1ObjWrap.getCollisionShape() );
			m_compoundShapeRevision1 = compoundShape1.getUpdateRevision();


		}

		internal override void Cleanup()
		{
			removeChildAlgorithms();
			BulletGlobals.CompoundCompoundCollisionAlgoritmPool.Free( this );
			m_childCollisionAlgorithmCache = null;
		}

		internal override void getAllContactManifolds( btManifoldArray manifoldArray )
		{
			int i;
			btSimplePairArray pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();
			for( i = 0; i < pairs.Count; i++ )
			{
				if( pairs[i].m_userPointer != null )
				{

					( (btCollisionAlgorithm)pairs[i].m_userPointer ).getAllContactManifolds( manifoldArray );
				}
			}
		}


		void removeChildAlgorithms()
		{
			btSimplePairArray pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();

			int numChildren = pairs.Count;
			int i;
			for( i = 0; i < numChildren; i++ )
			{
				if( pairs[i].m_userPointer != null )
				{
					btCollisionAlgorithm algo = (btCollisionAlgorithm)pairs[i].m_userPointer;
					//algo.~btCollisionAlgorithm();
					m_dispatcher.freeCollisionAlgorithm( algo );
				}
			}
			m_childCollisionAlgorithmCache.removeAllPairs();
		}

		internal class btCompoundCompoundLeafCallback : btDbvt.DefaultCollide
		{
			int m_numOverlapPairs;


			btCollisionObjectWrapper m_compound0ColObjWrap;
			btCollisionObjectWrapper m_compound1ColObjWrap;
			btDispatcher m_dispatcher;
			btDispatcherInfo m_dispatchInfo;
			btManifoldResult m_resultOut;


			btHashedSimplePairCache m_childCollisionAlgorithmCache;

			btPersistentManifold m_sharedManifold;

			internal btCompoundCompoundLeafCallback( btCollisionObjectWrapper compound1ObjWrap,
											btCollisionObjectWrapper compound0ObjWrap,
											btDispatcher dispatcher,
											btDispatcherInfo dispatchInfo,
											btManifoldResult resultOut,
											btHashedSimplePairCache childAlgorithmsCache,
											btPersistentManifold sharedManifold )
			{
				m_numOverlapPairs = ( 0 );
				m_compound0ColObjWrap = ( compound1ObjWrap );
				m_compound1ColObjWrap = ( compound0ObjWrap );
				m_dispatcher = ( dispatcher );
				m_dispatchInfo = ( dispatchInfo );
				m_resultOut = ( resultOut );
				m_childCollisionAlgorithmCache = ( childAlgorithmsCache );
				m_sharedManifold = ( sharedManifold );

			}

			public override void Process( btDbvt.btDbvtNode leaf0, btDbvt.btDbvtNode leaf1 )
			{
				m_numOverlapPairs++;


				int childIndex0 = leaf0.dataAsInt;
				int childIndex1 = leaf1.dataAsInt;


				Debug.Assert( childIndex0 >= 0 );
				Debug.Assert( childIndex1 >= 0 );


				btCompoundShape compoundShape0 = (btCompoundShape)( m_compound0ColObjWrap.getCollisionShape() );
				Debug.Assert( childIndex0 < compoundShape0.getNumChildShapes() );

				btCompoundShape compoundShape1 = (btCompoundShape)( m_compound1ColObjWrap.getCollisionShape() );
				Debug.Assert( childIndex1 < compoundShape1.getNumChildShapes() );

				btCollisionShape childShape0 = compoundShape0.getChildShape( childIndex0 );
				btCollisionShape childShape1 = compoundShape1.getChildShape( childIndex1 );

				//backup
				btTransform orgTrans0 = m_compound0ColObjWrap.m_collisionObject.m_worldTransform;
				btTransform childTrans0 = compoundShape0.getChildTransform( childIndex0 );
				btTransform newChildWorldTrans0; orgTrans0.Apply( ref childTrans0, out newChildWorldTrans0 );

				btTransform orgTrans1 = m_compound1ColObjWrap.m_collisionObject.m_worldTransform;
				btTransform childTrans1 = compoundShape1.getChildTransform( childIndex1 );
				btTransform newChildWorldTrans1; orgTrans1.Apply( ref childTrans1, out newChildWorldTrans1 );


				//perform an AABB check first
				btVector3 aabbMin0, aabbMax0, aabbMin1, aabbMax1;
				childShape0.getAabb( ref newChildWorldTrans0, out aabbMin0, out aabbMax0 );
				childShape1.getAabb( ref newChildWorldTrans1, out aabbMin1, out aabbMax1 );

				if( gCompoundCompoundChildShapePairCallback != null )
				{
					if( !gCompoundCompoundChildShapePairCallback( childShape0, childShape1 ) )
						return;
				}

				if( btAabbUtil.TestAabbAgainstAabb2( ref aabbMin0, ref aabbMax0, ref aabbMin1, ref aabbMax1 ) )
				{
					using( btCollisionObjectWrapper compoundWrap0 = BulletGlobals.CollisionObjectWrapperPool.Get()
											, compoundWrap1 = BulletGlobals.CollisionObjectWrapperPool.Get() )
					{
						compoundWrap0.Initialize( this.m_compound0ColObjWrap, childShape0, m_compound0ColObjWrap.m_collisionObject, -1, childIndex0 );
						compoundWrap1.Initialize( this.m_compound1ColObjWrap, childShape1, m_compound1ColObjWrap.m_collisionObject, -1, childIndex1 );


						btSimplePair pair = m_childCollisionAlgorithmCache.findPair( childIndex0, childIndex1 );

						btCollisionAlgorithm colAlgo = null;

						if( pair != null )
						{
							colAlgo = (btCollisionAlgorithm)pair.m_userPointer;

						}
						else
						{
							colAlgo = m_dispatcher.findAlgorithm( compoundWrap0, compoundWrap1, m_sharedManifold );
							pair = m_childCollisionAlgorithmCache.addOverlappingPair( childIndex0, childIndex1 );
							Debug.Assert( pair != null );
							pair.m_userPointer = colAlgo;
						}

						Debug.Assert( colAlgo != null );

						btCollisionObjectWrapper tmpWrap0;
						btCollisionObjectWrapper tmpWrap1;

						tmpWrap0 = m_resultOut.m_body0Wrap;
						tmpWrap1 = m_resultOut.m_body1Wrap;

						m_resultOut.m_body0Wrap=( compoundWrap0 );
						m_resultOut.m_body1Wrap=( compoundWrap1 );

						m_resultOut.setShapeIdentifiersA( -1, childIndex0 );
						m_resultOut.setShapeIdentifiersB( -1, childIndex1 );


						colAlgo.processCollision( compoundWrap0, ref newChildWorldTrans0, compoundWrap1, ref newChildWorldTrans1, m_dispatchInfo, m_resultOut );

						m_resultOut.m_body0Wrap=( tmpWrap0 );
						m_resultOut.m_body1Wrap=( tmpWrap1 );

					}

				}
			}
		};


#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		static bool MyIntersect( btDbvt.btDbvtVolume a,
										  btDbvt.btDbvtVolume b, ref btTransform xform )
		{
			btVector3 newmin, newmax;
			btAabbUtil.btTransformAabb( ref b._min, ref b._max, 0, ref xform, out newmin, out newmax );
			btDbvt.btDbvtVolume newb = btDbvt.btDbvtVolume.FromMM( ref newmin, ref newmax );
			return btDbvt.btDbvtVolume.Intersect( ref a, ref newb );
		}


#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		static void MycollideTT( btDbvt.btDbvtNode root0,
										  btDbvt.btDbvtNode root1,
										  ref btTransform xform,
										  btCompoundCompoundLeafCallback callback )
		{

			if( root0 != null && root1 != null )
			{
				int depth = 1;
				int treshold = btDbvt.DOUBLE_STACKSIZE - 4;
				btList<btDbvt.sStkNN> stkStack = new btList<btDbvt.sStkNN>( btDbvt.DOUBLE_STACKSIZE );
				stkStack[0].Initialize( root0, root1 );
				do
				{
					btDbvt.sStkNN p = stkStack[--depth];
					if( MyIntersect( p.a.volume, p.b.volume, ref xform ) )
					{
						if( depth > treshold )
						{
							stkStack.Capacity = ( stkStack.Count * 2 );
							treshold = stkStack.Count - 4;
						}
						if( p.a.IsInternal() )
						{
							if( p.b.IsInternal() )
							{
								stkStack[depth++].Initialize( p.a._children0, p.b._children0 );
								stkStack[depth++].Initialize( p.a._children1, p.b._children0 );
								stkStack[depth++].Initialize( p.a._children0, p.b._children1 );
								stkStack[depth++].Initialize( p.a._children1, p.b._children1 );
							}
							else
							{
								stkStack[depth++].Initialize( p.a._children0, p.b );
								stkStack[depth++].Initialize( p.a._children1, p.b );
							}
						}
						else
						{
							if( p.b.IsInternal() )
							{
								stkStack[depth++].Initialize( p.a, p.b._children0 );
								stkStack[depth++].Initialize( p.a, p.b._children1 );
							}
							else
							{
								callback.Process( p.a, p.b );
							}
						}
					}
				} while( depth != 0 );
			}
		}

		internal override void processCollision( btCollisionObjectWrapper body0Wrap
			, ref btTransform body0Transform
			, btCollisionObjectWrapper body1Wrap
			, ref btTransform body1Transform
			, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{

			btCollisionObjectWrapper col0ObjWrap = body0Wrap;
			btCollisionObjectWrapper col1ObjWrap = body1Wrap;

			Debug.Assert( col0ObjWrap.getCollisionShape().isCompound() );
			Debug.Assert( col1ObjWrap.getCollisionShape().isCompound() );
			btCompoundShape compoundShape0 = (btCompoundShape)( col0ObjWrap.getCollisionShape() );
			btCompoundShape compoundShape1 = (btCompoundShape)( col1ObjWrap.getCollisionShape() );

			btDbvt tree0 = compoundShape0.getDynamicAabbTree();
			btDbvt tree1 = compoundShape1.getDynamicAabbTree();
			if( tree0 == null || tree1 == null )
			{
				base.processCollision( body0Wrap, ref body0Transform, body1Wrap, ref body1Transform, dispatchInfo, resultOut );
				return;
			}
			///btCompoundShape might have changed:
			////make sure the internal child collision algorithm caches are still valid
			if( ( compoundShape0.getUpdateRevision() != m_compoundShapeRevision0 ) || ( compoundShape1.getUpdateRevision() != m_compoundShapeRevision1 ) )
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
				btManifoldArray manifoldArray = new btManifoldArray();
				btSimplePairArray pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();
				for( i = 0; i < pairs.Count; i++ )
				{
					if( pairs[i].m_userPointer != null )
					{
						btCollisionAlgorithm algo = (btCollisionAlgorithm)pairs[i].m_userPointer;
						algo.getAllContactManifolds( manifoldArray );
						for( int m = 0; m < manifoldArray.Count; m++ )
						{
							if( manifoldArray[m].m_cachedPoints != 0 )
							{
								resultOut.setPersistentManifold( manifoldArray[m] );
								resultOut.refreshContactPoints();
								resultOut.setPersistentManifold( null );
							}
						}
						manifoldArray.Count =( 0 );
					}
				}
			}




			btCompoundCompoundLeafCallback callback = new btCompoundCompoundLeafCallback
				( col0ObjWrap, col1ObjWrap,this.m_dispatcher, dispatchInfo, resultOut, this.m_childCollisionAlgorithmCache, m_sharedManifold);


			btTransform xform; body0Transform.inverseTimes( ref body1Transform, out xform );
			MycollideTT( tree0.m_root, tree1.m_root, ref xform, callback );

			//Console.WriteLine("#compound-compound child/leaf overlap =%d                      \r",callback.m_numOverlapPairs);

			//remove non-overlapping child pairs

			{
				Debug.Assert( m_removePairs.Count == 0 );

				//iterate over all children, perform an AABB check inside ProcessChildShape
				btSimplePairArray pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();

				int i;
				//btManifoldArray manifoldArray;


				btVector3 aabbMin0, aabbMax0, aabbMin1, aabbMax1;

				for( i = 0; i < pairs.Count; i++ )
				{
					if( pairs[i].m_userPointer != null )
					{
						btCollisionAlgorithm algo = (btCollisionAlgorithm)pairs[i].m_userPointer;

						{
							btCollisionShape childShape0 = null;

							btTransform newChildWorldTrans0;
							//btTransform orgInterpolationTrans0;
							childShape0 = compoundShape0.getChildShape( pairs[i].m_indexA );
							//orgInterpolationTrans0 = col0ObjWrap.m_worldTransform;
							btTransform childTrans0 = compoundShape0.getChildTransform( pairs[i].m_indexA );
							body0Transform.Apply( ref childTrans0, out newChildWorldTrans0 );
							childShape0.getAabb( ref newChildWorldTrans0, out aabbMin0, out aabbMax0 );
						}

						{
							btCollisionShape childShape1 = null;
							btTransform newChildWorldTrans1;

							childShape1 = compoundShape1.getChildShape( pairs[i].m_indexB );
							btTransform childTrans1 = compoundShape1.getChildTransform( pairs[i].m_indexB );
							body1Transform.Apply( ref childTrans1, out newChildWorldTrans1 );
							childShape1.getAabb( ref newChildWorldTrans1, out aabbMin1, out aabbMax1 );
						}



						if( !btAabbUtil.TestAabbAgainstAabb2( ref aabbMin0, ref aabbMax0, ref aabbMin1, ref aabbMax1 ) )
						{
							//algo.~btCollisionAlgorithm();
							m_dispatcher.freeCollisionAlgorithm( algo );
							m_removePairs.Add( new btSimplePair( pairs[i].m_indexA, pairs[i].m_indexB ) );
						}
					}
				}
				for( i = 0; i < m_removePairs.Count; i++ )
				{
					m_childCollisionAlgorithmCache.removeOverlappingPair( m_removePairs[i].m_indexA, m_removePairs[i].m_indexB );
				}
				m_removePairs.Clear();
			}

		}

		internal override double calculateTimeOfImpact( btCollisionObject body0, btCollisionObject body1, btDispatcherInfo dispatchInfo, btManifoldResult resultOut )
		{
			Debug.Assert( false );
			return 0;

		}



	};

}
