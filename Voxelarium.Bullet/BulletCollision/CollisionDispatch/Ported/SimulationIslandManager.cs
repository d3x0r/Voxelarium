
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

using System;
using System.Collections.Generic;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.NarrowPhase;
using Bullet.Types;
using System.Diagnostics;
using Bullet.LinearMath;

namespace Bullet.Collision.Dispatch
{

	///SimulationIslandManager creates and handles simulation islands, using btUnionFind
	public class btSimulationIslandManager
	{
		btUnionFind m_unionFind = new btUnionFind();

		btList<btPersistentManifold> m_islandmanifold = new btList<btPersistentManifold>();
		btList<btCollisionObject> m_islandBodies = new btList<btCollisionObject>();

		bool m_splitIslands;

		public btUnionFind getUnionFind() { return m_unionFind; }


		internal abstract class IslandCallback
		{
			internal abstract void processIsland( btCollisionObject[] bodies, int numBodies, btPersistentManifold[] manifolds, int first_manifold, int numManifolds, int islandId );
		};


		public bool getSplitIslands()
		{
			return m_splitIslands;
		}
		public void setSplitIslands( bool doSplitIslands )
		{
			m_splitIslands = doSplitIslands;
		}


		public btSimulationIslandManager()
		{
			m_splitIslands = ( true );
		}


		public void initUnionFind( int n )
		{
			m_unionFind.reset( n );
		}


		internal void findUnions( btDispatcher dispatcher, btCollisionWorld colWorld )
		{

			{
				btOverlappingPairCache pairCachePtr = colWorld.getPairCache();
				int numOverlappingPairs = pairCachePtr.getNumOverlappingPairs();
				if( numOverlappingPairs > 0 )
				{
					btBroadphasePair[] pairPtr = pairCachePtr.getOverlappingPairArrayPtr();

					for( int i = 0; i < numOverlappingPairs; i++ )
					{
						btBroadphasePair collisionPair = pairPtr[i];
						btCollisionObject colObj0 = (btCollisionObject)collisionPair.m_pProxy0.m_clientObject;
						btCollisionObject colObj1 = (btCollisionObject)collisionPair.m_pProxy1.m_clientObject;

						if( ( ( colObj0 != null ) && ( ( colObj0 ).mergesSimulationIslands() ) ) &&
							( ( colObj1 != null ) && ( ( colObj1 ).mergesSimulationIslands() ) ) )
						{
							m_unionFind.unite( ( colObj0 ).getIslandTag(),
								( colObj1 ).getIslandTag() );
						}
					}
				}
			}
		}

#if STATIC_SIMULATION_ISLAND_OPTIMIZATION
public virtual void   btSimulationIslandManager::updateActivationState(btCollisionWorld* colWorld,btDispatcher* dispatcher)
{

	// put the index into m_controllers into m_tag   
	int index = 0;
	{

		int i;
		for (i=0;i<colWorld.getCollisionObjectArray().Count; i++)
		{
			btCollisionObject   collisionObject= colWorld.getCollisionObjectArray()[i];
			//Adding filtering here
			if (!collisionObject.isStaticOrKinematicObject())
			{
				collisionObject.setIslandTag(index++);
			}
			collisionObject.setCompanionId(-1);
			collisionObject.setHitFraction(btScalar.BT_ONE);
		}
	}
	// do the union find

	initUnionFind( index );

	findUnions(dispatcher,colWorld);
}

public void void   btSimulationIslandManager::storeIslandActivationState(btCollisionWorld* colWorld)
{
	// put the islandId ('find' value) into m_tag   
	{
		int index = 0;
		int i;
		for (i=0;i<colWorld.getCollisionObjectArray().Count;i++)
		{
			btCollisionObject collisionObject= colWorld.getCollisionObjectArray()[i];
			if (!collisionObject.isStaticOrKinematicObject())
			{
				collisionObject.setIslandTag( m_unionFind.find(index) );
				//Set the correct object offset in Collision Object Array
				m_unionFind.getElement(index).m_sz = i;
				collisionObject.setCompanionId(-1);
				index++;
			} else
			{
				collisionObject.setIslandTag(-1);
				collisionObject.setCompanionId(-2);
			}
		}
	}
}


#else //STATIC_SIMULATION_ISLAND_OPTIMIZATION
		internal virtual void updateActivationState( btCollisionWorld colWorld, btDispatcher dispatcher )
		{
			// put the index into m_controllers into m_tag	
			int index = 0;
			{
				int i;
				for( i = 0; i < colWorld.getCollisionObjectArray().Count; i++ )
				{
					btCollisionObject collisionObject = colWorld.getCollisionObjectArray()[i];
					if( !collisionObject.isStaticOrKinematicObject() )
					{
						collisionObject.setIslandTag( index++ );
					}
					collisionObject.setCompanionBody( null );
					collisionObject.setHitFraction( btScalar.BT_ONE );
				}
				initUnionFind( index );
			}
			// do the union find

			findUnions( dispatcher, colWorld );
		}

		public virtual void storeIslandActivationState( btCollisionWorld colWorld )
		{
			// put the islandId ('find' value) into m_tag	
			{
				int index = 0;
				int i;
				for( i = 0; i < colWorld.getCollisionObjectArray().Count; i++ )
				{
					btCollisionObject collisionObject = colWorld.getCollisionObjectArray()[i];
					if( !collisionObject.isStaticOrKinematicObject() )
					{
						collisionObject.setIslandTag( m_unionFind.find( index ) );
						m_unionFind.setElementSz( index, i );
						collisionObject.setCompanionBody( null );
						index++;
					}
					else
					{
						collisionObject.setIslandTag( -1 );
						collisionObject.setCompanionBody( null );
					}
				}
			}
		}

#endif //STATIC_SIMULATION_ISLAND_OPTIMIZATION

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		static int getIslandId( btPersistentManifold lhs )
		{
			int islandId;
			btCollisionObject rcolObj0 = lhs.m_body0;
			btCollisionObject rcolObj1 = lhs.m_body1;
			islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
			return islandId;

		}



		/// function object that routes calls to operator<
		class btPersistentManifoldSortPredicate : IComparer<btPersistentManifold>
		{
			int IComparer<btPersistentManifold>.Compare( btPersistentManifold lhs, btPersistentManifold rhs )
			{
				return getIslandId( lhs ) < getIslandId( rhs ) ? -1 :
					getIslandId( lhs ) < getIslandId( rhs ) ? 1 : 0;
			}
		};


		internal void buildIslands( btDispatcher dispatcher, btCollisionWorld collisionWorld )
		{

			CProfileSample sample = new CProfileSample( "islandUnionFindAndQuickSort" );

			btCollisionObjectArray collisionObjects = collisionWorld.getCollisionObjectArray();

			m_islandmanifold.Count = ( 0 );

			//we are going to sort the unionfind array, and store the element id in the size
			//afterwards, we clean unionfind, to make sure no-one uses it anymore

			m_unionFind.sortIslands();
			int numElem = m_unionFind.getNumElements();

			int endIslandIndex = 1;
			int startIslandIndex;


			//update the sleeping state for bodies, if all are sleeping
			for( startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex )
			{
				int islandId = m_unionFind.getElementId( startIslandIndex );
				for( endIslandIndex = startIslandIndex + 1; ( endIslandIndex < numElem ) && ( m_unionFind.getElementId( endIslandIndex ) == islandId ); endIslandIndex++ )
				{
				}

				//int numSleeping = 0;

				bool allSleeping = true;

				int idx;
				for( idx = startIslandIndex; idx < endIslandIndex; idx++ )
				{
					int i2 = m_unionFind.getElementSz( idx );

					btCollisionObject colObj0 = collisionObjects[i2];
					if( ( colObj0.getIslandTag() != islandId ) && ( colObj0.getIslandTag() != -1 ) )
					{
						//				Console.WriteLine("error in island management\n");
					}

					Debug.Assert( ( colObj0.getIslandTag() == islandId ) || ( colObj0.getIslandTag() == -1 ) );
					if( colObj0.getIslandTag() == islandId )
					{
						if( colObj0.getActivationState() == ActivationState.ACTIVE_TAG )
						{
							allSleeping = false;
						}
						if( colObj0.getActivationState() == ActivationState.DISABLE_DEACTIVATION )
						{
							allSleeping = false;
						}
					}
				}


				if( allSleeping )
				{
					int idx2;
					for( idx2 = startIslandIndex; idx2 < endIslandIndex; idx2++ )
					{
						int i2 = m_unionFind.getElementSz( idx2 );
						btCollisionObject colObj0 = collisionObjects[i2];
						if( ( colObj0.getIslandTag() != islandId ) && ( colObj0.getIslandTag() != -1 ) )
						{
							//					Console.WriteLine("error in island management\n");
						}

						Debug.Assert( ( colObj0.getIslandTag() == islandId ) || ( colObj0.getIslandTag() == -1 ) );

						if( colObj0.getIslandTag() == islandId )
						{
							colObj0.setActivationState( ActivationState.ISLAND_SLEEPING );
						}
					}
				}
				else
				{

					int idx2;
					for( idx2 = startIslandIndex; idx2 < endIslandIndex; idx2++ )
					{
						int i2 = m_unionFind.getElementSz( idx2 );

						btCollisionObject colObj0 = collisionObjects[i2];
						if( ( colObj0.getIslandTag() != islandId ) && ( colObj0.getIslandTag() != -1 ) )
						{
							//					Console.WriteLine("error in island management\n");
						}

						Debug.Assert( ( colObj0.getIslandTag() == islandId ) || ( colObj0.getIslandTag() == -1 ) );

						if( colObj0.getIslandTag() == islandId )
						{
							if( colObj0.getActivationState() == ActivationState.ISLAND_SLEEPING )
							{
								colObj0.setActivationState( ActivationState.WANTS_DEACTIVATION );
								colObj0.setDeactivationTime( 0 );
							}
						}
					}
				}
			}


			int i;
			int maxNumManifolds = dispatcher.getNumManifolds();

			//#define SPLIT_ISLANDS 1
			//#if SPLIT_ISLANDS


			//#endif //SPLIT_ISLANDS


			for( i = 0; i < maxNumManifolds; i++ )
			{
				btPersistentManifold manifold = dispatcher.getManifoldByIndexInternal( i );

				btCollisionObject colObj0 = manifold.m_body0;
				btCollisionObject colObj1 = manifold.m_body1;

				///@todo: check sleeping conditions!
				if( ( ( colObj0 != null ) && colObj0.getActivationState() != ActivationState.ISLAND_SLEEPING ) ||
				   ( ( colObj1 != null ) && colObj1.getActivationState() != ActivationState.ISLAND_SLEEPING ) )
				{

					//kinematic objects don't merge islands, but wake up all connected objects
					if( colObj0.isKinematicObject() && colObj0.getActivationState() != ActivationState.ISLAND_SLEEPING )
					{
						if( colObj0.hasContactResponse() )
							colObj1.activate();
					}
					if( colObj1.isKinematicObject() && colObj1.getActivationState() != ActivationState.ISLAND_SLEEPING )
					{
						if( colObj1.hasContactResponse() )
							colObj0.activate();
					}
					if( m_splitIslands )
					{
						//filtering for response
						if( dispatcher.needsResponse( colObj0, colObj1 ) )
							m_islandmanifold.Add( manifold );
					}
				}
			}
		}



		///@todo: this is random access, it can be walked 'cache friendly'!
		internal void buildAndProcessIslands( btDispatcher dispatcher, btCollisionWorld collisionWorld, IslandCallback callback )
		{
			btCollisionObjectArray collisionObjects = collisionWorld.getCollisionObjectArray();

			buildIslands( dispatcher, collisionWorld );

			int endIslandIndex = 1;
			int startIslandIndex;
			int numElem = m_unionFind.getNumElements();

			CProfileSample sample = new CProfileSample( "processIslands" );

			if( !m_splitIslands )
			{
				btPersistentManifold[] manifold = dispatcher.getInternalManifoldPointer();
				int maxNumManifolds = dispatcher.getNumManifolds();
				callback.processIsland( collisionObjects.InternalArray, collisionObjects.Count, manifold, 0, maxNumManifolds, -1 );
			}
			else
			{
				// Sort manifolds, based on islands
				// Sort the vector using predicate and std::sort
				//std::sort(islandmanifold.begin(), islandmanifold.end(), btPersistentManifoldSortPredicate);

				int numManifolds = m_islandmanifold.Count;

				//tried a radix sort, but quicksort/heapsort seems still faster
				//@todo rewrite island management
				m_islandmanifold.Sort( new btPersistentManifoldSortPredicate() );
				//m_islandmanifold.heapSort(btPersistentManifoldSortPredicate());

				//now process all active islands (sets of manifolds for now)

				int startManifoldIndex = 0;
				int endManifoldIndex = 1;

				//int islandId;



				//	Console.WriteLine("Start Islands\n");

				//traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
				for( startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex )
				{
					int islandId = m_unionFind.getElementId( startIslandIndex );


					bool islandSleeping = true;

					for( endIslandIndex = startIslandIndex; ( endIslandIndex < numElem ) && ( m_unionFind.getElementId( endIslandIndex ) == islandId ); endIslandIndex++ )
					{
						int i = m_unionFind.getElementSz( endIslandIndex );
						btCollisionObject colObj0 = collisionObjects[i];
						m_islandBodies.Add( colObj0 );
						if( colObj0.isActive() )
							islandSleeping = false;
					}


					//find the accompanying contact manifold for this islandId
					int numIslandManifolds = 0;
					int start_manifold_offset = 0;
					//btPersistentManifold startManifold = null;

					if( startManifoldIndex < numManifolds )
					{
						int curIslandId = getIslandId( m_islandmanifold[startManifoldIndex] );
						if( curIslandId == islandId )
						{
							start_manifold_offset = startManifoldIndex;
							//startManifold = m_islandmanifold[startManifoldIndex];

							for( endManifoldIndex = startManifoldIndex + 1; ( endManifoldIndex < numManifolds ) && ( islandId == getIslandId( m_islandmanifold[endManifoldIndex] ) ); endManifoldIndex++ )
							{

							}
							/// Process the actual simulation, only if not sleeping/deactivated
							numIslandManifolds = endManifoldIndex - startManifoldIndex;
						}

					}

					if( !islandSleeping )
					{
						callback.processIsland( m_islandBodies.InternalArray, m_islandBodies.Count, m_islandmanifold.InternalArray, start_manifold_offset, numIslandManifolds, islandId );
						//			Console.WriteLine("Island callback of size:%d bodies, %d manifolds\n",islandBodies.Count,numIslandManifolds);
					}

					if( numIslandManifolds != 0 )
					{
						startManifoldIndex = endManifoldIndex;
					}

					m_islandBodies.Count = ( 0 );
				}
			} // else if(!splitIslands) 

		}
	}
}