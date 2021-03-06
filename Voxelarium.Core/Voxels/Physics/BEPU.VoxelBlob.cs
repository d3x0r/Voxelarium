﻿/* Originally found in BEPUPhysics/Demos/Extras/SimpleVoxelCollidableDemo.
 * Implements a low level collision shape.
 * 
 * Heavily modified for fitting to Voxelarium voxel system.
 *
 * Copyright 2015-2016 James Buckeyne  *** Added 11/22/2015
 *
 * Voxelarium is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Voxelarium is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.Events;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionTests;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.CollisionTests.Manifolds;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.Entities;
using BEPUphysics.NarrowPhaseSystems;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.OtherSpaceStages;
using BEPUphysics.PositionUpdating;
using BEPUphysics.Settings;
using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Diagnostics;

namespace Voxelarium.Core.Voxels.Physics
{
	/// <summary>
	/// Extremely simple and unoptimized voxel grid shape.
	/// Shapes can be shared between multiple collidables.
	/// </summary>
	public class VoxelBlobShape : EntityShape
	{
		/// <summary>
		/// Three dimensional grid of cells. A true value means the cell is filled, and false means it's empty.
		/// </summary>
		internal VoxelShape[] Cells;// { get; set; }
									//Note: This representation is very inefficient. Each bool occupies a full byte, and there is no space skipping.
									//Large blocks of empty space take just as much space as high frequency data.
									//If memory is a concern, it would be a good idea to optimize this. It would be pretty easy to get an order of magnitude (or three) improvement.

		/// <summary>
		/// Width of a single voxel cell.
		/// </summary>
		public float CellWidth { get; private set; }

		public void GetBoundingBox( ref Vector3 position, out BoundingBox boundingBox )
		{
			var size = new Vector3( CellWidth * VoxelSector.ZVOXELBLOCSIZE_X
								, CellWidth * VoxelSector.ZVOXELBLOCSIZE_Y
								, CellWidth * VoxelSector.ZVOXELBLOCSIZE_Z );
			boundingBox.Min = position;
			Vector3.Add( ref size, ref position, out boundingBox.Max );
		}

		internal VoxelBlobShape( VoxelShape[] cells, float cellWidth )
		{
			Cells = cells;
			CellWidth = cellWidth;
		}

		public void GetOverlaps( Vector3 gridPosition, BoundingBox boundingBox, ref QuickList<Int3> overlaps )
		{
			Vector3.Subtract( ref boundingBox.Min, ref gridPosition, out boundingBox.Min );
			Vector3.Subtract( ref boundingBox.Max, ref gridPosition, out boundingBox.Max );
			var inverseWidth = 1f / CellWidth;
			var min = new Int3
			{
				X = Math.Max( 0, (uint)( boundingBox.Min.X * inverseWidth ) ),
				Y = Math.Max( 0, (uint)( boundingBox.Min.Y * inverseWidth ) ),
				Z = Math.Max( 0, (uint)( boundingBox.Min.Z * inverseWidth ) )
			};
			var max = new Int3
			{
				X = Math.Min( VoxelSector.ZVOXELBLOCSIZE_X - 1, (uint)( boundingBox.Max.X * inverseWidth ) ),
				Y = Math.Min( VoxelSector.ZVOXELBLOCSIZE_Y - 1, (uint)( boundingBox.Max.Y * inverseWidth ) ),
				Z = Math.Min( VoxelSector.ZVOXELBLOCSIZE_Z - 1, (uint)( boundingBox.Max.Z * inverseWidth ) )
			};

			for( uint i = min.X; i <= max.X; ++i )
			{
				for( uint j = min.Y; j <= max.Y; ++j )
				{
					for( uint k = min.Z; k <= max.Z; ++k )
					{
						uint offset = i * VoxelSector.ZVOXELBLOCSIZE_Y + j + k * VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y;
						if( Cells[offset] != VoxelShape.Empty )
						{
							overlaps.Add( new Int3 { X = i, Y = j, Z = k } );
						}
					}
				}
			}
		}

		public override EntityCollidable GetCollidableInstance()
		{
			throw new NotImplementedException();
            //return new EntityCollidable( this );
		}

		public override void GetBoundingBox( ref RigidTransform transform, out BoundingBox boundingBox )
		{
			throw new NotImplementedException();
		}
	}

	/// <summary>
	/// Simple voxel grid collidable. Uses the VoxelGridShape as a data source and provides the 
	/// </summary>
	public class VoxelBlob : EntityCollidable
	{
		public new VoxelBlobShape Shape
		{
			get { return (VoxelBlobShape)base.Shape; }
		}

		/// <summary>
		/// Position of the minimum corner of the voxel grid.
		/// </summary>
		public Vector3 Position;

		public VoxelBlob( VoxelBlobShape shape, Vector3 position )
		{
			Position = position;
			base.Shape = shape;
			events = new ContactEventManager<VoxelBlob>( this );
		}

		public override bool RayCast( Ray ray, float maximumLength, out RayHit rayHit )
		{
			//This example is primarily to show custom collidable pair management with a minimum of other complexity, and this isn't vital.
			//Note: the character controller makes significant use of ray casts. While its basic features work without ray casts, 
			//implementing them will unlock more features and improve behavior.
			rayHit = new RayHit();
			return false;
		}

		public override bool ConvexCast( ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, out RayHit hit )
		{
			//This example is primarily to show custom collidable pair management with a minimum of other complexity, and this isn't vital.
			hit = new RayHit();
			return false;
		}

		public override void UpdateBoundingBox()
		{
			Shape.GetBoundingBox( ref Position, out boundingBox );
		}

		protected override void UpdateBoundingBoxInternal( float dt )
		{
			Shape.GetBoundingBox( ref Position, out boundingBox );
			//throw new NotImplementedException();
		}

		//For simplicity, the event manager is read only. The other collidables like StaticMesh and InstancedMesh have a setter, but it complicates things
		//and doesn't add a lot. For example implementations of setters, check those classes out.
		new protected internal ContactEventManager<VoxelBlob> events;
		///<summary>
		/// Gets the event manager of the mesh.
		///</summary>
		new public ContactEventManager<VoxelBlob> Events
		{
			get { return events; }
		}
		protected override IContactEventTriggerer EventTriggerer
		{
			get { return events; }
		}

		/*
		protected override IDeferredEventCreator EventCreator
		{
			get { return events; }
		}
		*/

		public override bool IsActive
		{
			get
			{
				return true;
			}
		}
	}

	/// <summary>
	/// Manages the contacts associated with a convex-voxelgrid collision.
	/// This is a slightly different kind of manifold- instead of directly managing collision, it manages
	/// a set of testers for each box near the opposing convex.
	/// </summary>
	public class VoxelBlobConvexContactManifold : ContactManifold
	{


		static LockingResourcePool<ReusableBoxCollidable> boxCollidablePool = new LockingResourcePool<ReusableBoxCollidable>();
		static LockingResourcePool<GeneralConvexPairTester> testerPool = new LockingResourcePool<GeneralConvexPairTester>();

		private VoxelBlob voxelBlob;
		private ConvexCollidable convex;


		public QuickDictionary<Int3, GeneralConvexPairTester> ActivePairs;
		private QuickDictionary<Int3, GeneralConvexPairTester> activePairsBackBuffer;
		protected RawValueList<ContactSupplementData> supplementData = new RawValueList<ContactSupplementData>( 4 );

		public VoxelBlobConvexContactManifold()
		{
			contacts = new RawList<Contact>( 4 );
			unusedContacts = new UnsafeResourcePool<Contact>( 4 );
			contactIndicesToRemove = new RawList<int>( 4 );
		}


		private GeneralConvexPairTester GetPair( ref Int3 position )
		{
			var pair = testerPool.Take();
			var boxCollidable = boxCollidablePool.Take();
			boxCollidable.Shape.Width = voxelBlob.Shape.CellWidth;
			boxCollidable.Shape.Height = voxelBlob.Shape.CellWidth;
			boxCollidable.Shape.Length = voxelBlob.Shape.CellWidth;
			pair.Initialize( convex, boxCollidable );
			boxCollidable.WorldTransform = new RigidTransform( new Vector3(
				voxelBlob.Position.X + ( position.X + 0.5f ) * voxelBlob.Shape.CellWidth,
				voxelBlob.Position.Y + ( position.Y + 0.5f ) * voxelBlob.Shape.CellWidth,
				voxelBlob.Position.Z + ( position.Z + 0.5f ) * voxelBlob.Shape.CellWidth ) );
			return pair;
		}


		private void ReturnPair( GeneralConvexPairTester pair )
		{
			boxCollidablePool.GiveBack( (ReusableBoxCollidable)pair.CollidableB );
			pair.CleanUp();
			testerPool.GiveBack( pair );
		}



		public override void Initialize( Collidable newCollidableA, Collidable newCollidableB )
		{
			convex = newCollidableA as ConvexCollidable;
			voxelBlob = newCollidableB as VoxelBlob;


			if( convex == null || voxelBlob == null )
			{
				convex = newCollidableB as ConvexCollidable;
				voxelBlob = newCollidableA as VoxelBlob;
				if( convex == null || voxelBlob == null )
					throw new ArgumentException( "Inappropriate types used to initialize contact manifold." );
			}
			ActivePairs = new QuickDictionary<Int3, GeneralConvexPairTester>( BufferPools<Int3>.Locking, BufferPools<GeneralConvexPairTester>.Locking, BufferPools<int>.Locking, 3 );
			activePairsBackBuffer = new QuickDictionary<Int3, GeneralConvexPairTester>( BufferPools<Int3>.Locking, BufferPools<GeneralConvexPairTester>.Locking, BufferPools<int>.Locking, 3 );

		}

		public override void CleanUp()
		{
			convex = null;

			for( int i = ActivePairs.Count - 1; i >= 0; --i )
			{
				ReturnPair( ActivePairs.Values[i] );
				ActivePairs.Values[i].CleanUp();
			}
			//Clear->dispose is technically unnecessary now, but it may avoid some pain later on when this behavior changes in v2.
			//This will be a very sneaky breaking change...
			ActivePairs.Clear();
			ActivePairs.Dispose();
			Debug.Assert( activePairsBackBuffer.Count == 0 );
			activePairsBackBuffer.Dispose();
			base.CleanUp();
		}


		public override void Update( float dt )
		{
			//Refresh the contact manifold for this frame.
			var transform = new RigidTransform( voxelBlob.Position );
			var convexTransform = convex.WorldTransform;
			ContactRefresher.ContactRefresh( contacts, supplementData, ref convexTransform, ref transform, contactIndicesToRemove );
			RemoveQueuedContacts();

			//Collect the set of overlapped cell indices.
			//Not the fastest way to do this, but it's relatively simple and easy.
			var overlaps = new QuickList<Int3>( BufferPools<Int3>.Thread );
			voxelBlob.Shape.GetOverlaps( voxelBlob.Position, convex.BoundingBox, ref overlaps );

			var candidatesToAdd = new QuickList<ContactData>( BufferPools<ContactData>.Thread, BufferPool<int>.GetPoolIndex( overlaps.Count ) );
			for( int i = 0; i < overlaps.Count; ++i )
			{
				GeneralConvexPairTester manifold;
				if( !ActivePairs.TryGetValue( overlaps.Elements[i], out manifold ) )
				{
					//This manifold did not previously exist.
					manifold = GetPair( ref overlaps.Elements[i] );
				}
				else
				{
					//It did previously exist.
					ActivePairs.FastRemove( overlaps.Elements[i] );
				}
				activePairsBackBuffer.Add( overlaps.Elements[i], manifold );
				ContactData contactCandidate;
				if( manifold.GenerateContactCandidate( out contactCandidate ) )
				{

					candidatesToAdd.Add( ref contactCandidate );
				}
			}
			overlaps.Dispose();
			//Any pairs remaining in the activePairs set no longer exist. Clean them up.
			for( int i = ActivePairs.Count - 1; i >= 0; --i )
			{
				ReturnPair( ActivePairs.Values[i] );
				ActivePairs.FastRemove( ActivePairs.Keys[i] );
			}
			//Swap the pair sets.
			var temp = ActivePairs;
			ActivePairs = activePairsBackBuffer;
			activePairsBackBuffer = temp;

			//Check if adding the new contacts would overflow the manifold.
			if( contacts.Count + candidatesToAdd.Count > 4 )
			{
				//Adding all the contacts would overflow the manifold.  Reduce to the best subset.
				var reducedCandidates = new QuickList<ContactData>( BufferPools<ContactData>.Thread, 3 );
				ContactReducer.ReduceContacts( contacts, ref candidatesToAdd, contactIndicesToRemove, ref reducedCandidates );
				RemoveQueuedContacts();
				for( int i = reducedCandidates.Count - 1; i >= 0; i-- )
				{
					Add( ref reducedCandidates.Elements[i] );
					reducedCandidates.RemoveAt( i );
				}
				reducedCandidates.Dispose();
			}
			else if( candidatesToAdd.Count > 0 )
			{
				//Won't overflow the manifold, so just toss it in.
				for( int i = 0; i < candidatesToAdd.Count; i++ )
				{
					Add( ref candidatesToAdd.Elements[i] );
				}
			}


			candidatesToAdd.Dispose();

		}


		protected override void Add( ref ContactData contactCandidate )
		{
			ContactSupplementData supplement;
			supplement.BasePenetrationDepth = contactCandidate.PenetrationDepth;
			var convexTransform = convex.WorldTransform;
			var gridTransform = new RigidTransform( voxelBlob.Position );
			RigidTransform.TransformByInverse( ref contactCandidate.Position, ref convexTransform, out supplement.LocalOffsetA );
			RigidTransform.TransformByInverse( ref contactCandidate.Position, ref gridTransform, out supplement.LocalOffsetB );
			supplementData.Add( ref supplement );
			base.Add( ref contactCandidate );
		}
		protected override void Remove( int contactIndex )
		{
			supplementData.RemoveAt( contactIndex );
			base.Remove( contactIndex );
		}


	}

	public class VoxelBlobConvexPairHandler : StandardPairHandler
	{
		public static void EnsurePairsAreRegistered()
		{
			//Assume if one has been added, all have.
			if( !NarrowPhaseHelper.CollisionManagers.ContainsKey( new TypePair( typeof( ConvexCollidable<BoxShape> ), typeof( VoxelBlob ) ) ) )
			{
				var factory = new NarrowPhasePairFactory<VoxelBlobConvexPairHandler>();
				NarrowPhaseHelper.CollisionManagers.Add( new TypePair( typeof( ConvexCollidable<BoxShape> ), typeof( VoxelBlob ) ), factory );
				NarrowPhaseHelper.CollisionManagers.Add( new TypePair( typeof( ConvexCollidable<SphereShape> ), typeof( VoxelBlob ) ), factory );
				NarrowPhaseHelper.CollisionManagers.Add( new TypePair( typeof( ConvexCollidable<CapsuleShape> ), typeof( VoxelBlob ) ), factory );
				NarrowPhaseHelper.CollisionManagers.Add( new TypePair( typeof( ConvexCollidable<TriangleShape> ), typeof( VoxelBlob ) ), factory );
				NarrowPhaseHelper.CollisionManagers.Add( new TypePair( typeof( ConvexCollidable<CylinderShape> ), typeof( VoxelBlob ) ), factory );
				NarrowPhaseHelper.CollisionManagers.Add( new TypePair( typeof( ConvexCollidable<ConeShape> ), typeof( VoxelBlob ) ), factory );
				NarrowPhaseHelper.CollisionManagers.Add( new TypePair( typeof( ConvexCollidable<TransformableShape> ), typeof( VoxelBlob ) ), factory );
				NarrowPhaseHelper.CollisionManagers.Add( new TypePair( typeof( ConvexCollidable<MinkowskiSumShape> ), typeof( VoxelBlob ) ), factory );
				NarrowPhaseHelper.CollisionManagers.Add( new TypePair( typeof( ConvexCollidable<WrappedShape> ), typeof( VoxelBlob ) ), factory );
				NarrowPhaseHelper.CollisionManagers.Add( new TypePair( typeof( ConvexCollidable<ConvexHullShape> ), typeof( VoxelBlob ) ), factory );
			}
		}

		private VoxelBlob voxelBlob;
		private ConvexCollidable convex;
		public override Collidable CollidableA
		{
			get { return convex; }
		}

		public override Collidable CollidableB
		{
			get { return voxelBlob; }
		}

		public override Entity EntityA
		{
			get { return convex.Entity; }
		}

		public override Entity EntityB
		{
			get { return null; }
		}

		public override void UpdateTimeOfImpact( Collidable requester, float dt )
		{
			//Complicated and not vital. Leaving it out for simplicity. Check out InstancedMeshPairHandler for an example implementation.
			//Notice that we don't test for convex entity null explicitly.  The convex.IsActive property does that for us.
			if( convex.IsActive && convex.Entity.PositionUpdateMode == PositionUpdateMode.Continuous )
			{
				//Only perform the test if the minimum radii are small enough relative to the size of the velocity.
				Vector3 velocity = convex.Entity.LinearVelocity * dt;
				float velocitySquared = velocity.LengthSquared();

				var minimumRadius = convex.Shape.MinimumRadius * MotionSettings.CoreShapeScaling;
				timeOfImpact = 1;
				if( minimumRadius * minimumRadius < velocitySquared )
				{
					for( int i = 0; i < contactManifold.ActivePairs.Count; ++i )
					{
						var pair = contactManifold.ActivePairs.Values[i];
						//In the contact manifold, the box collidable is always put into the second slot.
						var boxCollidable = (ReusableBoxCollidable)pair.CollidableB;
						RayHit rayHit;
						var worldTransform = boxCollidable.WorldTransform;
						if( GJKToolbox.CCDSphereCast( new Ray( convex.WorldTransform.Position, velocity ), minimumRadius, boxCollidable.Shape, ref worldTransform, timeOfImpact, out rayHit ) &&
							rayHit.T > Toolbox.BigEpsilon )
						{
							timeOfImpact = rayHit.T;
						}
					}
				}
			}
		}

		protected override void GetContactInformation( int index, out ContactInformation info )
		{
			info.Contact = contactManifold.Contacts[index];
			//Find the contact's normal and friction forces.
			info.FrictionImpulse = 0;
			info.NormalImpulse = 0;

			for( int i = 0; i < constraint.ContactFrictionConstraints.Count; i++ )
			{
				if( constraint.ContactFrictionConstraints[i].PenetrationConstraint.Contact == info.Contact )
				{
					info.FrictionImpulse = constraint.ContactFrictionConstraints[i].TotalImpulse;
					info.NormalImpulse = constraint.ContactFrictionConstraints[i].PenetrationConstraint.NormalImpulse;
					break;
				}
			}

			//Compute relative velocity
			if( convex.Entity != null )
			{
				info.RelativeVelocity = Toolbox.GetVelocityOfPoint( ref info.Contact.Position, ref convex.Entity.position, ref convex.Entity.linearVelocity, ref convex.Entity.angularVelocity );
			}
			else
				info.RelativeVelocity = new Vector3();


			info.Pair = this;
		}

		public VoxelBlobConvexPairHandler()
		{
			constraint = new NonConvexContactManifoldConstraint( this );
		}

		private VoxelBlobConvexContactManifold contactManifold = new VoxelBlobConvexContactManifold();
		private NonConvexContactManifoldConstraint constraint;
		public override ContactManifold ContactManifold
		{
			get { return contactManifold; }
		}

		public override ContactManifoldConstraint ContactConstraint
		{
			get { return constraint; }
		}


		public override void Initialize( BroadPhaseEntry entryA, BroadPhaseEntry entryB )
		{

			voxelBlob = entryA as VoxelBlob;
			convex = entryB as ConvexCollidable;

			if( voxelBlob == null || convex == null )
			{
				voxelBlob = entryB as VoxelBlob;
				convex = entryA as ConvexCollidable;

				if( voxelBlob == null || convex == null )
					throw new ArgumentException( "Inappropriate types used to initialize pair." );
			}

			//Contact normal goes from A to B.
			broadPhaseOverlap = new BroadPhaseOverlap( convex, voxelBlob, broadPhaseOverlap.CollisionRule );

			Debugger.Break();
			//UpdateMaterialProperties( convex.Entity != null ? convex.Entity.Material : null, voxelBlob.Material );


			base.Initialize( entryA, entryB );

		}


		///<summary>
		/// Cleans up the pair handler.
		///</summary>
		public override void CleanUp()
		{
			base.CleanUp();
			voxelBlob = null;
			convex = null;
		}
	}
}
