/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel. (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
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
using Voxelarium.LinearMath;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Game
{
	public class Actor
	{
		VoxelGameEnvironment GameEnv;
		Actor Next;
		Actor Pred;
		//ActorPhysicEngine PhysicsEngine;

		//btVector3 Location;        // The player position.
		btVector3 Location_Old;    // The old player position.
		btVector3 Velocity;        // Player velocity
		btVector3 Deplacement;     // Deplacement is legs movement speed.
		internal btTransform ViewDirection;   // Player viewing and displacement Direction.
		btVector3 EyesPosition;    // Player Eyes relative to the foot center.
		double DammageThreshold;
		double LifePoints;
		bool CollideWithVoxels;
		bool CollideWithActors;
		bool Flag_ActivateAntiFall;
		bool KeepControlOnJumping;
		bool IsDead;
		int VoxelSelectDistance;
		bool IsOnGround;
		bool TakesGravity;
		uint JumpDebounce;
		bool order;
		double DeathChronometer;

		//Camera Camera;
		//RayCast_out PointedVoxel;
		bool[] MouseButtonMatrix = new bool[8];

		//Inventory Inventory;

		ushort BuildingMaterial;
		ushort LearningModePage;

		// Game Time

		uint Time_TotalGameTime;
		uint Time_ElapsedTimeSinceLastRespawn;

		Actor()
		{
			int i;
			Next = Pred = null;
			VoxelSelectDistance = 6;
			Velocity.x = Velocity.y = Velocity.z = 0.0f;
			Deplacement.x = Deplacement.y = Deplacement.z = 0.0f;
			//Location.x = Location.y = Location.z = 0.0;
			//ViewDirection.yaw = ViewDirection.pitch = ViewDirection.roll = ViewDirection.Len = 0.0;
			CollideWithVoxels = false;
			CollideWithActors = false;
			Flag_ActivateAntiFall = false;
			TakesGravity = true;
			DammageThreshold = 599.0;
			LifePoints = 1000;
			JumpDebounce = 0;
			BuildingMaterial = 1;
			for( i = 0; i < 8; i++ ) MouseButtonMatrix[i] = false;
#if FINISH_PORTING
			Inventory = null;
			Camera.ColoredVision.Activate = false;
			Camera.ColoredVision.Blue = 1.0f;
			Camera.ColoredVision.Red = 1.0f;
			Camera.ColoredVision.Green = 1.0f;
			PhysicsEngine = null;
#endif
			KeepControlOnJumping = true;
			IsDead = false;
			IsOnGround = false;

			order = false;
			DeathChronometer = 0.0;
			GameEnv = null;

			Time_TotalGameTime = 0;
			Time_ElapsedTimeSinceLastRespawn = 0;
			LearningModePage = 0;

		}

		public virtual void Init( bool Death = false ) { }
		public virtual void TakeDammage( double Dammage )
		{
			if( Dammage > DammageThreshold ) LifePoints -= (long)( Dammage / 6.0 );
			if( LifePoints < 0 ) { LifePoints = 0; }
		}

		public virtual void SetPosition( btVector3 NewLocation )
		{
			ViewDirection.Translate( ref NewLocation );
		}
		public virtual void GetPosition( out ZVector3L BlocLocation ) {
			BlocLocation.x = ( (int)ViewDirection.x ) >> VoxelSector.ZVOXELBLOCSHIFT_X ;
			BlocLocation.y = ( (int)ViewDirection.y ) >> VoxelSector.ZVOXELBLOCSHIFT_Y;
			BlocLocation.z = ( (int)ViewDirection.z ) >> VoxelSector.ZVOXELBLOCSHIFT_Z; }
		public virtual void Action_SetActorMode( uint ActorMode ) { }
		public virtual void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }
		public virtual bool Save( BinaryWriter OutStream ) { return ( true ); }
		public virtual bool Load( BinaryReader InStream ) { return ( true ); }

		/*
			public virtual void DoPhysic(double CycleTime)
			{
			  btVector3 NewLocation;
			  btVector3 P[32];
			  Bool PEnable[32];
			  uint i;

			  NewLocation.x = Deplacement.x / 25.0 + Location.x + Velocity.x * CycleTime / 1000.0;
			  NewLocation.y = Deplacement.y / 25.0 + Location.y + Velocity.y * CycleTime / 1000.0;
			  NewLocation.z = Deplacement.z / 25.0 + Location.z + Velocity.z * CycleTime / 1000.0;

			  P[0] = NewLocation + btVector3(-75.0,+0.0,+75.0);
			  P[1] = NewLocation + btVector3(+75.0,+0.0,+75.0);
			  P[2] = NewLocation + btVector3(+75.0,+0.0,-75.0);
			  P[3] = NewLocation + btVector3(-75.0,+0.0,-75.0);

			  P[4] = NewLocation + btVector3(-75.0,+500.0,+75.0);
			  P[5] = NewLocation + btVector3(+75.0,+500.0,+75.0);
			  P[6] = NewLocation + btVector3(+75.0,+500.0,-75.0);
			  P[7] = NewLocation + btVector3(-75.0,+500.0,-75.0);

			  P[8]  = NewLocation + btVector3(+85.0,+128.0,+85.0);
			  P[9]  = NewLocation + btVector3(-85.0,+128.0,+85.0);
			  P[10] = NewLocation + btVector3(+85.0,+384.0,+85.0);
			  P[11] = NewLocation + btVector3(-85.0,+384.0,+85.0);

			  P[12] = NewLocation + btVector3(+85.0,+128.0,+85.0);
			  P[13] = NewLocation + btVector3(+85.0,+128.0,-85.0);
			  P[14] = NewLocation + btVector3(+85.0,+384.0,+85.0);
			  P[15] = NewLocation + btVector3(+85.0,+384.0,-85.0);

			  P[16] = NewLocation + btVector3(+85.0,+128.0,-85.0);
			  P[17] = NewLocation + btVector3(-85.0,+128.0,-85.0);
			  P[18] = NewLocation + btVector3(+85.0,+384.0,-85.0);
			  P[19] = NewLocation + btVector3(-85.0,+384.0,-85.0);

			  P[20] = NewLocation + btVector3(-85.0,+128.0,+85.0);
			  P[21] = NewLocation + btVector3(-85.0,+128.0,-85.0);
			  P[22] = NewLocation + btVector3(-85.0,+384.0,+85.0);
			  P[23] = NewLocation + btVector3(-85.0,+384.0,-85.0);

			  for (i=0;i<32;i++) PEnable[i] = false;

			  if (NewLocation.x > Location.x) { PEnable[12] = true; PEnable[13] = true; PEnable[14] = true; PEnable[15] = true; }
			  if (NewLocation.x < Location.x) { PEnable[20] = true; PEnable[21] = true; PEnable[22] = true; PEnable[23] = true; }

			  if (NewLocation.z > Location.z) { PEnable[8] = true; PEnable[9] = true; PEnable[10] = true; PEnable[11] = true; }
			  if (NewLocation.z < Location.z) { PEnable[16] = true; PEnable[17] = true; PEnable[18] = true; PEnable[19] = true; }

			}
		*/
#if FINISH_PORTING
		public virtual void DoPhysic( uint FrameTime );
#endif
		public virtual void Action_GoFastForward( double speed ) { }
		public virtual void Action_GoForward() { }
		public virtual void Action_GoBackward() { }
		public virtual void Action_GoLeftStraff() { }
		public virtual void Action_GoRightStraff() { }
		public virtual void Action_Jump() { }
		public virtual void Action_GoUp() { }
		public virtual void Action_GoDown() { }
		public virtual void Action_MouseMove( float Delta_x, float Delta_y ) { }
		public virtual void Action_MouseButtonClick( uint Button ) { }
		public virtual void Action_MouseButtonRelease( uint Button ) { }
		public virtual bool Action_StillEvents( bool[] MouseMatrix, byte[] KeyboardMatrix ) { return true; }
		public virtual void Action_NextBuildingMaterial()
		{
			VoxelTypeManager VoxelTypeManager = GameEnv.GetVoxelTypeManager();
			VoxelType VoxelType;
			if( VoxelTypeManager == null ) return;
			BuildingMaterial++;
			if( null == ( VoxelType = VoxelTypeManager.GetVoxelType( BuildingMaterial ) ) )
			{
				BuildingMaterial--;
			}
			else
			{
				Log.log( "Selected Material {0}: {1}", BuildingMaterial, VoxelType.properties.VoxelTypeName );
			}
		}
		public virtual void Action_PrevBuildingMaterial()
		{
			VoxelTypeManager VoxelTypeManager = GameEnv.GetVoxelTypeManager();
			VoxelType VoxelType;
			if( VoxelTypeManager == null ) return;
			if( BuildingMaterial > 1 ) BuildingMaterial--;
			if( null == ( VoxelType = VoxelTypeManager.GetVoxelType( BuildingMaterial ) ) )
			{
				BuildingMaterial++;
			}
			else
			{
				Log.log( "Selected Material {0}: {1}", BuildingMaterial, VoxelType.properties.VoxelTypeName );
			}
		}


		public virtual void Event_Collision( double RelativeVelocity ) { }
		public virtual void Event_Death() { }


		public virtual void Start_Riding( int x, int y, int z ) { }
		public virtual void Stop_Riding() { }
		public virtual void Action_GetInOutOfVehicle() { }
	}
}
