
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

namespace Bullet.Collision.Dispatch
{

	///SimulationIslandManager creates and handles simulation islands, using btUnionFind
	public class btSimulationIslandManager
	{
		btUnionFind m_unionFind;

		btList<btPersistentManifold> m_islandmanifold;
		btList<btCollisionObject> m_islandBodies;

		bool m_splitIslands;

		public btUnionFind getUnionFind() { return m_unionFind; }


		delegate void IslandCallback( btCollisionObject bodies, int numBodies
			, btPersistentManifold[] manifolds, int numManifolds, int islandId );


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
            m_splitIslands = (true);
        }


		public void initUnionFind( int n )
		{
			m_unionFind.reset( n );
		}


		public void findUnions( btDispatcher dispatcher, btCollisionWorld colWorld )
		{

			{
				btOverlappingPairCache pairCachePtr = colWorld.getPairCache();
				int numOverlappingPairs = pairCachePtr.getNumOverlappingPairs();
				if( numOverlappingPairs > 0 )
				{
					btBroadphasePair[] pairPtr = pairCachePtr.getOverlappingPairArrayPtr();

					for( int i = 0; i < numOverlappingPairs; i++ )
					{
						btBroadphasePair  collisionPair = pairPtr[i];
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
		public virtual void updateActivationState( btCollisionWorld colWorld, btDispatcher dispatcher )
		{

			initUnionFind( colWorld.getCollisionObjectArray().Count );

			// put the index into m_controllers into m_tag	
			{

				int index = 0;
				int i;
				for( i = 0; i < colWorld.getCollisionObjectArray().Count; i++ )
				{
					btCollisionObject collisionObject = colWorld.getCollisionObjectArray()[i];
					collisionObject.setIslandTag( index );
					collisionObject.setCompanionId( -1 );
					collisionObject.setHitFraction( (double)(1.0) );
					index++;

				}
			}
			// do the union find

			findUnions( dispatcher, colWorld );
		}

		public virtual void btSimulationIslandManager::storeIslandActivationState( btCollisionWorld* colWorld )
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
						collisionObject.setCompanionId( -1 );
					}
					else
					{
						collisionObject.setIslandTag( -1 );
						collisionObject.setCompanionId( -2 );
					}
					index++;
				}
			}
		}

#endif //STATIC_SIMULATION_ISLAND_OPTIMIZATION

		inline int getIslandId( btPersistentManifold* lhs )
		{
			int islandId;
			btCollisionObject rcolObj0 = static_cast<btCollisionObject>( lhs.getBody0() );
			btCollisionObject rcolObj1 = static_cast<btCollisionObject>( lhs.getBody1() );
			islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
			return islandId;

		}



		/// function object that routes calls to operator<
		class btPersistentManifoldSortPredicate : IComparer<btPersistentManifold>
		{
			int IComparer<btPersistentManifold>.Compare( btPersistentManifold x, btPersistentManifold y )
			{
				return getIslandId( lhs ) < getIslandId( rhs ) ? -1 :
					getIslandId( lhs ) < getIslandId( rhs ) ? 1 : 0;
			}
		};


		public void buildIslands( btDispatcher dispatcher, btCollisionWorld collisionWorld )
		{

			CProfileSample sample = new CProfileSample( "islandUnionFindAndQuickSort" );

			btCollisionObjectArray & collisionObjects = collisionWorld.getCollisionObjectArray();

			m_islandmanifold.resize( 0 );

			//we are going to sort the unionfind array, and store the element id in the size
			//afterwards, we clean unionfind, to make sure no-one uses it anymore

			getUnionFind().sortIslands();
			int numElem = getUnionFind().getNumElements();

			int endIslandIndex = 1;
			int startIslandIndex;


			//update the sleeping state for bodies, if all are sleeping
			for( startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex )
			{
				int islandId = getUnionFind().getElement( startIslandIndex ).m_id;
				for( endIslandIndex = startIslandIndex + 1; ( endIslandIndex < numElem ) && ( getUnionFind().getElement( endIslandIndex ).m_id == islandId ); endIslandIndex++ )
				{
				}

				//int numSleeping = 0;

				bool allSleeping = true;

				int idx;
				for( idx = startIslandIndex; idx < endIslandIndex; idx++ )
				{
					int i = getUnionFind().getElement( idx ).m_sz;

					btCollisionObject colObj0 = collisionObjects[i];
					if( ( colObj0.getIslandTag() != islandId ) && ( colObj0.getIslandTag() != -1 ) )
					{
						//				Console.WriteLine("error in island management\n");
					}

					Debug.Assert( ( colObj0.getIslandTag() == islandId ) || ( colObj0.getIslandTag() == -1 ) );
					if( colObj0.getIslandTag() == islandId )
					{
						if( colObj0.getActivationState() == ACTIVE_TAG )
						{
							allSleeping = false;
						}
						if( colObj0.getActivationState() == DISABLE_DEACTIVATION )
						{
							allSleeping = false;
						}
					}
				}


				if( allSleeping )
				{
					int idx;
					for( idx = startIslandIndex; idx < endIslandIndex; idx++ )
					{
						int i = getUnionFind().getElement( idx ).m_sz;
						btCollisionObject colObj0 = collisionObjects[i];
						if( ( colObj0.getIslandTag() != islandId ) && ( colObj0.getIslandTag() != -1 ) )
						{
							//					Console.WriteLine("error in island management\n");
						}

						Debug.Assert( ( colObj0.getIslandTag() == islandId ) || ( colObj0.getIslandTag() == -1 ) );

						if( colObj0.getIslandTag() == islandId )
						{
							colObj0.setActivationState( ISLAND_SLEEPING );
						}
					}
				}
				else
				{

					int idx;
					for( idx = startIslandIndex; idx < endIslandIndex; idx++ )
					{
						int i = getUnionFind().getElement( idx ).m_sz;

						btCollisionObject colObj0 = collisionObjects[i];
						if( ( colObj0.getIslandTag() != islandId ) && ( colObj0.getIslandTag() != -1 ) )
						{
							//					Console.WriteLine("error in island management\n");
						}

						Debug.Assert( ( colObj0.getIslandTag() == islandId ) || ( colObj0.getIslandTag() == -1 ) );

						if( colObj0.getIslandTag() == islandId )
						{
							if( colObj0.getActivationState() == ISLAND_SLEEPING )
							{
								colObj0.setActivationState( WANTS_DEACTIVATION );
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
				btPersistentManifold* manifold = dispatcher.getManifoldByIndexInternal( i );

				btCollisionObject colObj0 = static_cast<btCollisionObject>( manifold.getBody0() );
				btCollisionObject colObj1 = static_cast<btCollisionObject>( manifold.getBody1() );

				///@todo: check sleeping conditions!
				if( ( ( colObj0 ) && colObj0.getActivationState() != ISLAND_SLEEPING ) ||
				   ( ( colObj1 ) && colObj1.getActivationState() != ISLAND_SLEEPING ) )
				{

					//kinematic objects don't merge islands, but wake up all connected objects
					if( colObj0.isKinematicObject() && colObj0.getActivationState() != ISLAND_SLEEPING )
					{
						if( colObj0.hasContactResponse() )
							colObj1.activate();
					}
					if( colObj1.isKinematicObject() && colObj1.getActivationState() != ISLAND_SLEEPING )
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
		void buildAndProcessIslands( btDispatcher dispatcher, btCollisionWorld collisionWorld, IslandCallback callback )
		{
			btCollisionObjectArray & collisionObjects = collisionWorld.getCollisionObjectArray();

			buildIslands( dispatcher, collisionWorld );

			int endIslandIndex = 1;
			int startIslandIndex;
			int numElem = getUnionFind().getNumElements();

			CProfileSample sample = new CProfileSample( "processIslands" );

			if( !m_splitIslands )
			{
				btPersistentManifold[] manifold = dispatcher.getInternalManifoldPointer();
				int maxNumManifolds = dispatcher.getNumManifolds();
				callback( collisionObjects[0], collisionObjects.Count, manifold, maxNumManifolds, -1 );
			}
			else
			{
				// Sort manifolds, based on islands
				// Sort the vector using predicate and std::sort
				//std::sort(islandmanifold.begin(), islandmanifold.end(), btPersistentManifoldSortPredicate);

				int numManifolds = int( m_islandmanifold.Count );

				//tried a radix sort, but quicksort/heapsort seems still faster
				//@todo rewrite island management
				m_islandmanifold.quickSort( btPersistentManifoldSortPredicate() );
				//m_islandmanifold.heapSort(btPersistentManifoldSortPredicate());

				//now process all active islands (sets of manifolds for now)

				int startManifoldIndex = 0;
				int endManifoldIndex = 1;

				//int islandId;



				//	Console.WriteLine("Start Islands\n");

				//traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
				for( startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex )
				{
					int islandId = getUnionFind().getElement( startIslandIndex ).m_id;


					bool islandSleeping = true;

					for( endIslandIndex = startIslandIndex; ( endIslandIndex < numElem ) && ( getUnionFind().getElement( endIslandIndex ).m_id == islandId ); endIslandIndex++ )
					{
						int i = getUnionFind().getElement( endIslandIndex ).m_sz;
						btCollisionObject colObj0 = collisionObjects[i];
						m_islandBodies.Add( colObj0 );
						if( colObj0.isActive() )
							islandSleeping = false;
					}


					//find the accompanying contact manifold for this islandId
					int numIslandManifolds = 0;
					btPersistentManifold[] startManifold = 0;

					if( startManifoldIndex < numManifolds )
					{
						int curIslandId = getIslandId( m_islandmanifold[startManifoldIndex] );
						if( curIslandId == islandId )
						{
							startManifold = &m_islandmanifold[startManifoldIndex];

							for( endManifoldIndex = startManifoldIndex + 1; ( endManifoldIndex < numManifolds ) && ( islandId == getIslandId( m_islandmanifold[endManifoldIndex] ) ); endManifoldIndex++ )
							{

							}
							/// Process the actual simulation, only if not sleeping/deactivated
							numIslandManifolds = endManifoldIndex - startManifoldIndex;
						}

					}

					if( !islandSleeping )
					{
						callback.processIsland( m_islandBodies, m_islandBodies.Count, startManifold, numIslandManifolds, islandId );
						//			Console.WriteLine("Island callback of size:%d bodies, %d manifolds\n",islandBodies.Count,numIslandManifolds);
					}

					if( numIslandManifolds )
					{
						startManifoldIndex = endManifoldIndex;
					}

					m_islandBodies.resize( 0 );
				}
			} // else if(!splitIslands) 

		}
	}
}