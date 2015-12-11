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
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels.IO;
using Voxelarium.Core.Voxels.UI;
using System.Threading;
using Voxelarium.Common;
using Voxelarium.Core.Voxels.Types;
using OpenTK;
using System;

namespace Voxelarium.Core.Voxels
{
	public class VoxelWorld
	{
		internal struct RayCast_in
		{
			// Input fields
			internal Camera Camera;
			internal int MaxCubeIterations;
			internal int PlaneCubeDiff;
			internal float MaxDetectionDistance;
		};

		internal struct RayCast_out
		{
			// Output fields
			internal bool Collided;
			internal byte CollisionAxe;  // 0=x , 1=y , 2 = z;
			internal byte CollisionFace; // 0=Front , 1=Back, 2=Left, 3=Right, 4=Top, 5=Bottom;
			internal VoxelCoords PointedVoxel;
			internal VoxelCoords PredPointedVoxel;
			internal btVector3 CollisionPoint;
			internal Vector2 PointInCubeFace;
			internal float CollisionDistance;
		};


		public static VoxelSector WorkingFullSector;
		public static VoxelSector WorkingEmptySector;
		//public static VoxelSector WorkingScratchSector;

		VoxelGameEnvironment GameEnv;
		internal SectorRingList SectorEjectList;
		public VoxelSector SectorList;

		internal RenderInterface renderer;
		VoxelSector[] SectorTable;
		internal VoxelTypeManager VoxelTypeManager;
		internal SectorLoader SectorLoader;

		// world has voxel size... all blocks in a 'world' are constant
		public int VoxelBlockSizeBits = 0;
		public float VoxelBlockSize = 1;

		const int TableSize = SectorHashSize_x * SectorHashSize_y * SectorHashSize_z;
		public const int SectorHashSize_x = 32;
		public const int SectorHashSize_y = 32;
		public const int SectorHashSize_z = 32;
		public btMatrix3x3 location;
		public int UniverseNum;

		internal TextureAtlas TextureAtlas;
		bool nogui;

		static VoxelWorld()
		{
			WorkingFullSector = new VoxelSector( null, (VoxelWorld)null );
			//GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingFullSector );
			WorkingFullSector.Fill( 0x0001 );
			WorkingEmptySector = new VoxelSector( null, (VoxelWorld)null );
			//GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingEmptySector );
			WorkingEmptySector.Fill( 0 );
			//WorkingScratchSector = new VoxelSector( null, (VoxelWorld)null );
			//GameEnv.Basic_Renderer.GetCuller().InitFaceCullData( WorkingScratchSector );
		}

		private void Display_OnInvalidate()
		{
			VoxelSector sector;
			for( sector = SectorList; sector != null; sector = sector.GlobalList_Next )
			{
				sector.Invalidate();
			}
		}

		public VoxelWorld( bool nogui, VoxelGameEnvironment GameEnv )
		{
			uint i;
			this.nogui = nogui;
			this.GameEnv = GameEnv;
			Display.OnInvalidate += Display_OnInvalidate;
			SectorEjectList = new SectorRingList( 256 * 256 * 32/*65536*/);
			if( !nogui )
				TextureAtlas = new TextureAtlas( 32, 64 );

			SectorTable = new VoxelSector[TableSize];

			for( i = 0; i < TableSize; i++ ) SectorTable[i] = null;

			SectorList = null;
			UniverseNum = 1;
			VoxelTypeManager = null;
		}

		~VoxelWorld()
		{
			VoxelSector Sector, NewSector;

			Sector = SectorList;

			while( Sector != null )
			{
				NewSector = Sector.GlobalList_Next;
				if( VoxelGlobalSettings.COMPILEOPTION_ALLOWSAVE )
				{
					if( VoxelGlobalSettings.COMPILEOPTION_SAVEONLYMODIFIED )
					{
						//Log.log( " *** SAVE INCOMPLETE *** " );

						if( Sector.IsMustBeSaved() )
						{
							Sector.Save( UniverseNum );
						}
					}
				}
				Sector.Dispose();
				Sector = NewSector;
			}

			if( SectorTable != null ) { SectorTable = null; }

			if( WorkingFullSector != null ) { WorkingFullSector.Dispose(); WorkingFullSector = null; }
			if( WorkingEmptySector != null ) { WorkingEmptySector.Dispose(); WorkingEmptySector = null; }
			//if( WorkingScratchSector != null ) { WorkingScratchSector.Dispose(); WorkingScratchSector = null; }
			SectorList = null;
			UniverseNum = 0;
			if( SectorEjectList != null ) SectorEjectList.Dispose();
			SectorEjectList = null;
		}

		internal VoxelSector FindSector( int x, int y, int z )
		{
			int xs, ys, zs, Offset;
			VoxelSector SectorPointer;

			xs = x & ( SectorHashSize_x - 1 );
			ys = y & ( SectorHashSize_y - 1 );
			zs = z & ( SectorHashSize_z - 1 );

			Offset = xs + ys * SectorHashSize_x + ( zs * SectorHashSize_x * SectorHashSize_y );

			SectorPointer = SectorTable[Offset];
			while( SectorPointer != null )
			{
				if( ( SectorPointer.Pos_x == x ) && ( SectorPointer.Pos_y == y ) && ( SectorPointer.Pos_z == z ) ) return ( SectorPointer );
				SectorPointer = SectorPointer.Next;
			}
			return null;
		}

		VoxelSector FindSector_Secure( int x, int y, int z ) // Create sector if not in memory.
		{
			int xs, ys, zs, Offset;
			VoxelSector SectorPointer;

			xs = x & ( SectorHashSize_x - 1 );
			ys = y & ( SectorHashSize_y - 1 );
			zs = z & ( SectorHashSize_z - 1 );

			Offset = xs + ys * SectorHashSize_x + ( zs * SectorHashSize_x * SectorHashSize_y );
			bool requested = false;

			while( true )
			{
				AutoResetEvent wait_event = new AutoResetEvent( false );
				SectorPointer = SectorTable[Offset];
				while( SectorPointer != null )
				{
					if( ( SectorPointer.Pos_x == x ) && ( SectorPointer.Pos_y == y ) && ( SectorPointer.Pos_z == z ) ) return ( SectorPointer );
					SectorPointer = SectorPointer.Next;
				}
				if( !requested )
				{
					RequestSector( x, y, z, 5, wait_event );
					requested = true;
				}
				// need a while to load a sector...
				wait_event.WaitOne( -1 );
				//System.Threading.Thread.Sleep( 30 );
				ProcessNewLoadedSectors();
			}
		}

		internal void ProcessNewLoadedSectors()
		{
			VoxelSector Sector, AdjSector;
			//if( SectorLoader == null ) return;

			while( ( Sector = SectorLoader.GetRequested() ) != null )
			{
				if( FindSector( Sector.Pos_x, Sector.Pos_y, Sector.Pos_z ) == null )
				{
					AddSector( Sector );
					GameEnv.Engine.Add( Sector.physics );
					Sector.Culler = renderer.GetCuller();
					Sector.Culler.InitFaceCullData( Sector );

					Sector.Culler.CullSector( Sector, true, 0 );

					Sector.Flag_Void_Regular = false;
					Sector.Flag_Void_Transparent = false;

					Sector.Flag_Render_Dirty = true;
					//printf("AddSector: %ld,%ld,%ld\n",Sector.Pos_x, Sector.Pos_y, Sector.Pos_z);

					// Partial face culing for adjacent sectors
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1]; // find to the left... update its right
					if( AdjSector != null )
					{
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.RIGHT );
					}
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1]; // find to the right... update its left
					if( AdjSector != null )
					{
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.LEFT );
					}
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1]; // behind 'behind' update its ahead...
					if( AdjSector != null )
					{
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.AHEAD );
					}
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1]; // found to the front, update its behind
					if( AdjSector != null )
					{
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.BEHIND );
					}
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1]; // found below update its above
					if( AdjSector != null )
					{
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.ABOVE );
					}
					AdjSector = Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1]; // found above, udpate its below
					if( AdjSector != null )
					{
						AdjSector.Culler.CullSector( AdjSector, false, VoxelSector.FACEDRAW_Operations.BELOW );
					}
				}
				else { Sector.Dispose(); Log.log( "Loading already used sector***\n" ); }
			}
		}

		void AddSector( VoxelSector Sector )
		{
			int x, y, z, Offset;
			VoxelSector SectorPointer;

			{
				int n;
				for( n = 0; n < 6; n++ )
				{
					VoxelSector near_sec = FindSector( Sector.Pos_x + VoxelSector.NormalBasePosition[n].x
													  , Sector.Pos_y + VoxelSector.NormalBasePosition[n].y
													  , Sector.Pos_z + VoxelSector.NormalBasePosition[n].z );
					if( near_sec != null )
					{
						Sector.near_sectors[n] = near_sec;
						near_sec.near_sectors[n ^ 1] = Sector;
					}
				}
			}

			// Adding to fast access hash

			x = Sector.Pos_x & ( SectorHashSize_x - 1 );
			y = Sector.Pos_y & ( SectorHashSize_y - 1 );
			z = Sector.Pos_z & ( SectorHashSize_z - 1 );

			Offset = x + y * SectorHashSize_x + ( z * SectorHashSize_x * SectorHashSize_y );

			if( SectorTable[Offset] == null )
			{
				SectorTable[Offset] = Sector; Sector.Next = null; Sector.Pred = null;
			}
			else
			{
				SectorPointer = SectorTable[Offset];
				while( SectorPointer.Next != null ) SectorPointer = SectorPointer.Next;
				SectorPointer.Next = Sector;
				Sector.Next = null;
				Sector.Pred = SectorPointer;
			}

			// Adding to sequential access global list

			if( SectorList == null )
			{
				SectorList = Sector;
				Sector.GlobalList_Next = null;
				Sector.GlobalList_Pred = null;
			}
			else
			{
				Sector.GlobalList_Next = SectorList;
				Sector.GlobalList_Pred = null;
				SectorList.GlobalList_Pred = Sector;
				SectorList = Sector;
			}
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public bool GetVoxelRef( out VoxelRef result, int x, int y, int z )
		{
			result.Sector = FindSector( x >> VoxelSector.ZVOXELBLOCSHIFT_X, y >> VoxelSector.ZVOXELBLOCSHIFT_Y, z >> VoxelSector.ZVOXELBLOCSHIFT_Z );
			result.wx = x;
			result.wy = y;
			result.wz = z;
			result.Offset = (uint)( ( result.y = (byte)( y & VoxelSector.ZVOXELBLOCMASK_Y ) )
				   + ( ( result.x = (byte)( x & VoxelSector.ZVOXELBLOCMASK_X ) ) << VoxelSector.ZVOXELBLOCSHIFT_Y )
				   + ( ( result.z = (byte)( z & VoxelSector.ZVOXELBLOCMASK_Z ) ) << ( VoxelSector.ZVOXELBLOCSHIFT_Y + VoxelSector.ZVOXELBLOCSHIFT_X ) ) );

			result.World = this;
			result.VoxelTypeManager = VoxelTypeManager;
			if( result.Sector == null )
			{
				result.Type = null;
				result.VoxelExtension = null;
				return false;
			}
			result.Type = VoxelTypeManager[result.Sector.Data.Data[result.Offset]];
			result.VoxelExtension = result.Sector.Data.OtherInfos[result.Offset];

			return true;
		}

		internal void SetUniverseNum( int UniverseNum ) { this.UniverseNum = UniverseNum; }
		internal void SetVoxelTypeManager( VoxelTypeManager Manager, ref int percent, ref int step, ref int steps )
		{
			VoxelTypeManager = Manager;
			if( !nogui )
				Manager.LoadTexturesToAtlas( TextureAtlas, ref percent, ref step, ref steps );
		}

		internal void CreateDemoWorld( ref int start_percent, ref int start_step, ref int start_steps )
		{
			int x, y, z;
			start_steps += 64;

			for( x = -2; x <= 1; x++ )
			{
				for( y = -2; y <= 1; y++ )
				{
					for( z = -2; z <= 1; z++ )
					{
						//Log.log( "Load guarantee {0} {1} {2}", x, y, z );
						FindSector_Secure( x, y, z );
						start_percent = ( ++start_step * 100 ) / start_steps;
					}
				}
			}
		}

		internal void SetVoxel( int x, int y, int z, int VoxelValue )
		{
			VoxelSector Sector;
			Sector = FindSector( x >> VoxelSector.ZVOXELBLOCSHIFT_X, y >> VoxelSector.ZVOXELBLOCSHIFT_Y, z >> VoxelSector.ZVOXELBLOCSHIFT_Z );
			if( Sector == null ) return;
			Sector.SetCube( x, y, z, VoxelValue );
		}

		internal void RequestSector( int x, int y, int z, int Priority, EventWaitHandle wait_event = null )
		{
			if( SectorLoader == null ) return;

			SectorLoader.Request_Sector( x, y, z, Priority, wait_event );
		}

		internal void SetSectorLoader( SectorLoader SectorLoader )
		{
			this.SectorLoader = SectorLoader;
		}
		internal bool RequestSectorEject( VoxelSector SectorToEject )
		{ return ( SectorEjectList.PushToList( SectorToEject ) ); }

		internal void ProcessOldEjectedSectors()
		{
			VoxelSector Sector;

			if( SectorLoader == null ) return;

			while( SectorLoader.Is_EjectFileNotFull() )
			{
				if( ( Sector = SectorEjectList.PullFromList() ) == null ) break;

				// printf("EjectPass : %lx %lu\n",Sector,++debug_ejectpass);
				RemoveSector( Sector );
				SectorLoader.Eject_Sector( Sector );
				// ejecting sectors are re-used not disposed.
				//Sector.Dispose();
			}
		}

		void RemoveSector( VoxelSector Sector )
		{
			int x, y, z, Offset;
			VoxelSector SectorPointer;

			for( x = 0; x < 6; x++ )
			{
				if( Sector.near_sectors[x] != null ) Sector.near_sectors[x].near_sectors[x ^ 1] = null;
				Sector.near_sectors[x] = null;
			}
			GameEnv.Engine.Remove( Sector.physics );
			// Finding sector in hash
			x = ( Sector.Pos_x & ( SectorHashSize_x - 1 ) );
			y = ( Sector.Pos_y & ( SectorHashSize_y - 1 ) );
			z = ( Sector.Pos_z & ( SectorHashSize_z - 1 ) );
			Offset = x + y * SectorHashSize_x + ( z * SectorHashSize_x * SectorHashSize_y );
			SectorPointer = SectorTable[Offset];
			while( SectorPointer != Sector )
			{
				if( SectorPointer == null ) return;
				SectorPointer = SectorPointer.Next;
			}

			// Remove from hash
			if( SectorPointer == SectorTable[Offset] ) SectorTable[Offset] = Sector.Next;
			else Sector.Pred.Next = Sector.Next;
			if( Sector.Next != null ) Sector.Next.Pred = Sector.Pred;

			// Remove from global list

			if( Sector.GlobalList_Pred != null ) Sector.GlobalList_Pred.GlobalList_Next = Sector.GlobalList_Next;
			else { SectorList = Sector.GlobalList_Next; }
			if( Sector.GlobalList_Next != null ) Sector.GlobalList_Next.GlobalList_Pred = Sector.GlobalList_Pred;

			// Zeroing fields

			Sector.Next = null; Sector.Pred = null; Sector.GlobalList_Next = null; Sector.GlobalList_Pred = null;
		}
		public void Swap( ref VoxelRef self, ref VoxelRef other, VoxelSector.ModifiedFieldFlags importance )
		{
			ushort selfType = self.Type.properties.Type;// Sector.Data.Data[self.Offset];
			VoxelExtension selfOtherInfo = self.VoxelExtension;//Sector.Data.OtherInfos[self.Offset];
			ushort otherType = other.Type.properties.Type;//Sector.Data.Data[other.Offset];
			VoxelExtension otherOtherInfo = other.VoxelExtension;//.Sector.Data.OtherInfos[other.Offset];

			self.Sector.Data.Data[self.Offset] = otherType;
			self.Sector.Data.OtherInfos[self.Offset] = otherOtherInfo;
			other.Sector.Data.Data[other.Offset] = selfType;
			other.Sector.Data.OtherInfos[other.Offset] = selfOtherInfo;

			self.Sector.Culler.CullSingleVoxel( self.Sector, self.Offset );
			other.Sector.Culler.CullSingleVoxel( other.Sector, other.Offset );

			self.Sector.Flag_IsModified |= importance;
			other.Sector.Flag_IsModified |= importance;
		}

		public void Swap( ref VoxelRef self, ref NearVoxelRef other, VoxelSector.ModifiedFieldFlags importance )
		{
			ushort selfType = self.Type.properties.Type;// Sector.Data.Data[self.Offset];
			VoxelExtension selfOtherInfo = self.VoxelExtension;//Sector.Data.OtherInfos[self.Offset];
			ushort otherType = other.Type.properties.Type;//Sector.Data.Data[other.Offset];
			VoxelExtension otherOtherInfo = other.VoxelExtension;//.Sector.Data.OtherInfos[other.Offset];

			self.Sector.Data.Data[self.Offset] = otherType;
			self.Sector.Data.OtherInfos[self.Offset] = otherOtherInfo;
			other.Sector.Data.Data[other.Offset] = selfType;
			other.Sector.Data.OtherInfos[other.Offset] = selfOtherInfo;

			self.Sector.Culler.CullSingleVoxel( self.Sector, self.Offset );
			other.Sector.Culler.CullSingleVoxel( other.Sector, other.Offset );
			self.Sector.Flag_IsModified |= importance;
			other.Sector.Flag_IsModified |= importance;
		}

		public void Swap( ref VoxelRef self, VoxelSector.RelativeVoxelOrds direction, VoxelSector.ModifiedFieldFlags importance )
		{
			VoxelRef other; VoxelRef.GetNearVoxelRef( out other, ref self, direction );
			Swap( ref self, ref other, importance );
		}


		internal bool SetVoxel_WithCullingUpdate( int x, int y, int z, ushort VoxelValue, VoxelSector.ModifiedFieldFlags ImportanceFactor
			, bool CreateExtension, out VoxelRef Location )
		{
			int sx = x >> VoxelSector.ZVOXELBLOCSHIFT_X;
			int sy = y >> VoxelSector.ZVOXELBLOCSHIFT_X;
			int sz = z >> VoxelSector.ZVOXELBLOCSHIFT_X;
			Location.World = this;
			Location.wx = x;
			Location.wy = y;
			Location.wz = z;
			Location.Sector = FindSector( sx, sy, sz );
			Location.Offset = (uint)( ( Location.y = (byte)( y & VoxelSector.ZVOXELBLOCMASK_Y ) )
				+ ( Location.x = (byte)( x & VoxelSector.ZVOXELBLOCMASK_X ) ) * Location.Sector.Size_y
				+ ( Location.z = (byte)( z & VoxelSector.ZVOXELBLOCMASK_Z ) ) * Location.Sector.Size_y * Location.Sector.Size_x );
			Location.Sector.SetCube( Location.x, Location.y, Location.z, VoxelValue );
			Location.Sector.Flag_IsModified |= ImportanceFactor;
			Location.Sector.Culler.CullSingleVoxel( Location.Sector, Location.Offset );
			Location.VoxelTypeManager = VoxelTypeManager;
			Location.Type = VoxelTypeManager.VoxelTable[VoxelValue];
			Location.VoxelExtension = null;
			return true;
		}

		internal ushort GetVoxelExt( int x, int y, int z, out VoxelExtension OtherInfos )
		{
			VoxelSector Sector;
			uint Offset;

			Sector = FindSector( x >> VoxelSector.ZVOXELBLOCSHIFT_X, y >> VoxelSector.ZVOXELBLOCSHIFT_Y, z >> VoxelSector.ZVOXELBLOCSHIFT_Z );

			if( Sector == null )
			{
				OtherInfos = null;
				return 0;
			}

			Offset = (uint)(( y & VoxelSector.ZVOXELBLOCMASK_Y )
				   + ( ( x & VoxelSector.ZVOXELBLOCMASK_X ) << VoxelSector.ZVOXELBLOCSHIFT_Y )
				   + ( ( z & VoxelSector.ZVOXELBLOCMASK_Z ) << ( VoxelSector.ZVOXELBLOCSHIFT_Y + VoxelSector.ZVOXELBLOCSHIFT_X ) ) );

			OtherInfos = Sector.Data.OtherInfos[Offset];
			return ( Sector.Data.Data[Offset] );
		}

		/*
		bool RayCast(const ZRayCast_in* In, ZRayCast_out * Out );
		bool RayCast_Vector(const ZVector3d & Pos, const ZVector3d & Vector, const ZRayCast_in* In, ZRayCast_out * Out, bool InvertCollision );

		bool RayCast_Vector( ZMatrix & Pos, const ZVector3d & Vector, const ZRayCast_in* In, ZRayCast_out * Out, bool InvertCollision = false );
		bool RayCast_Vector2(const ZVector3d & Pos, const ZVector3d & Vector, const ZRayCast_in* In, ZRayCast_out * Out, bool InvertCollision );

		bool RayCast_Vector_special(const ZVector3d & Pos, const ZVector3d & Vector, const ZRayCast_in* In, ZRayCast_out * Out, bool InvertCollision = false );

		bool RayCast2( double x, double y, double z, double yaw, double pitch, double roll, ZVoxelCoords & PointedCube, ZVoxelCoords CubeBeforePointed );
		*/
		internal ushort GetVoxel( int x, int y, int z )
		{
			VoxelSector Sector;
			int Offset;

			Sector = FindSector( x >> VoxelSector.ZVOXELBLOCSHIFT_X, y >> VoxelSector.ZVOXELBLOCSHIFT_Y, z >> VoxelSector.ZVOXELBLOCSHIFT_Z );

			if( Sector == null ) return 0;

			Offset = ( y & VoxelSector.ZVOXELBLOCMASK_Y )
					 + ( ( x & VoxelSector.ZVOXELBLOCMASK_X ) << VoxelSector.ZVOXELBLOCSHIFT_Y )
					 + ( ( z & VoxelSector.ZVOXELBLOCMASK_Z ) << ( VoxelSector.ZVOXELBLOCSHIFT_Y + VoxelSector.ZVOXELBLOCSHIFT_X ) );

			return ( Sector.Data.Data[Offset] );
		}

		bool RayCast2( float x, float y, float z, float yaw, float pitch, float roll, ref VoxelCoords PointedCube, ref VoxelCoords CubeBeforePointed )
		{
			btVector3 Delta_h;
			Vector3 Offset_h;
			Vector3 Norm_h;
			Vector3 Collision_h;
			int ActualCube_x, ActualCube_y, ActualCube_z;
			int NewCube_x, NewCube_y, NewCube_z;

			int i;


			// Delta_h.X = tan((- yaw)/57.295779513);
			Delta_h.y = (float)Math.Tan( -pitch / 57.295779513 );
			Delta_h.z = (float)Math.Tan( ( yaw + 90.0 ) / 57.295779513 );



			Collision_h.X = ( (float)Math.Floor( x / VoxelBlockSize ) + 1.0f ) * VoxelBlockSize;
			Collision_h.Y = ( Collision_h.X - x ) * Delta_h.y + y;
			Collision_h.Z = ( Collision_h.X - x ) * Delta_h.z + z;

			if( yaw >= 0.0 && yaw < 180.0 ) Offset_h.X = VoxelBlockSize;
			else Offset_h.X = -VoxelBlockSize;
			Offset_h.Y = Delta_h.y * VoxelBlockSize;
			Offset_h.Z = Delta_h.z * VoxelBlockSize;

			Norm_h.X = Offset_h.X / VoxelBlockSize;
			Norm_h.Y = Offset_h.Y / VoxelBlockSize;
			Norm_h.Z = Offset_h.Z / VoxelBlockSize;

			// printf("Angle (y:%lf p:%lf) XYZ:(%lf %lf %lf) Off(%lf %lf %lf) Coll(%lf %lf %lf)\n", yaw,pitch,x,y,Z, Offset_h.X, Offset_h.Y, Offset_h.Z, Collision_h.X, Collision_h.Y, Collision_h.Z);

			for( i = 0; i < 50; i++ )
			{
				ActualCube_x = (int)( ( Collision_h.X - Norm_h.X ) / VoxelBlockSize );
				ActualCube_y = (int)( ( Collision_h.Y - Norm_h.Y ) / VoxelBlockSize );
				ActualCube_z = (int)( ( Collision_h.Z - Norm_h.Z ) / VoxelBlockSize );
				NewCube_x = (int)Math.Floor( ( Collision_h.X + Norm_h.X ) / VoxelBlockSize );
				NewCube_y = (int)Math.Floor( ( Collision_h.Y + Norm_h.Y ) / VoxelBlockSize );
				NewCube_z = (int)Math.Floor( ( Collision_h.Z + Norm_h.Z ) / VoxelBlockSize );

				Collision_h.X += Offset_h.X; Collision_h.Y += Offset_h.Y; Collision_h.Z += Offset_h.Z;

				if( GetVoxel( NewCube_x, NewCube_y, NewCube_z ) > 0 )
				{
					CubeBeforePointed.X = ActualCube_x; CubeBeforePointed.Y = ActualCube_y; CubeBeforePointed.Z = ActualCube_z;
					PointedCube.X = NewCube_x; PointedCube.Y = NewCube_y; PointedCube.Z = NewCube_z;
					//printf("MATCH: %ld %ld %ld POS %lf %lf %lf\n",PointedCube.X, PointedCube.Y, PointedCube.Z, Collision_h.X, Collision_h.Y, Collision_h.Z);
					return ( true );
				}

			}
			// printf("\n");
			return ( false );
			//printf("first_h_x : %lf first_h_y %lf\n",first_h_x,first_h_y);
		}


		bool RayCast( ref RayCast_in In, ref RayCast_out Out )
		{
			btVector3 Delta_h, Delta_v, Delta_s;
			btVector3 Offset_h = btVector3.Zero, Offset_v = btVector3.Zero, Offset_s = btVector3.Zero;
			btVector3 Norm_h = btVector3.Zero, Norm_v = btVector3.Zero, Norm_s = btVector3.Zero;
			btVector3 Collision_h = btVector3.Zero
				, Collision_v = btVector3.Zero
				, Collision_s = btVector3.Zero;

			/*
			Offset_h.X = Offset_h.Y = Offset_h.Z = Offset_h.w = 0.0;
			Offset_v.X = Offset_v.Y = Offset_v.Z = Offset_v.w = 0.0;
			Offset_s.X = Offset_s.Y = Offset_s.Z = Offset_s.w = 0.0;
			Collision_h.X = Collision_h.Y = Collision_h.Z = Collision_h.w = 0.0;
			Collision_v.X = Collision_v.Y = Collision_v.Z = Collision_v.w = 0.0;
			Collision_s.X = Collision_s.Y = Collision_s.Z = Collision_s.w = 0.0;
			*/


			int ActualCube_x, ActualCube_y, ActualCube_z;
			int NewCube_x, NewCube_y, NewCube_z;
			bool Collide_X, Collide_Y, Collide_Z;
			int i;

			btVector3 Norm;
			btVector3 Tmp;

			Norm.x = 0; Norm.y = 0; Norm.z = 1; Norm.w = 0;
			In.Camera.location.Apply( ref Norm, out Tmp );
			// X axis rotation
			//Tmp.Y = Norm.Y * cos(-In.Camera.Pitch/57.295779513) - Norm.Z * sin(-In.Camera.Pitch/57.295779513);
			//Tmp.Z = Norm.Y * sin(-In.Camera.Pitch/57.295779513) + Norm.Z * cos(-In.Camera.Pitch/57.295779513);
			//Norm.Y = Tmp.Y; Norm.Z = Tmp.Z;
			// Y axis rotation
			//Tmp.X = Norm.Z*sin(In.Camera.Yaw/57.295779513) + Norm.X * cos(In.Camera.Yaw/57.295779513);
			//Tmp.Z = Norm.Z*cos(In.Camera.Yaw/57.295779513) - Norm.X * sin(In.Camera.Yaw/57.295779513);
			//Norm.X = Tmp.X; Norm.Z = Tmp.Z;
			/*
			  // Z axis rotation
			  Tmp.X = Norm.X * cos(roll/57.295779513) - Norm.Y * sin(roll/57.295779513);
			  Tmp.Y = Norm.X * sin(roll/57.295779513) + Norm.Y * cos(roll/57.295779513);
			  Norm.X = Tmp.X; Norm.Y = Tmp.Y;
			*/
			// Delta_h.X = tan((- yaw)/57.295779513);

			Collide_X = Collide_Y = Collide_Z = false;

			if( In.Camera.location.m_basis.m_el0.x >= 0.01 )
			{
				Collide_X = true;
				Delta_h.y = Norm.y / -Norm.x;
				Delta_h.z = Norm.z / -Norm.x;
				Delta_h.w = 1.0f / Norm.x;
				Collision_h.x = (float)( Math.Floor( In.Camera.location.m_origin.x / VoxelBlockSize ) + 1.0f ) * VoxelBlockSize;
				Collision_h.y = ( Collision_h.x - In.Camera.location.m_origin.x ) * Delta_h.y + In.Camera.location.m_origin.y;
				Collision_h.z = ( Collision_h.x - In.Camera.location.m_origin.x ) * Delta_h.z + In.Camera.location.m_origin.z;
				Collision_h.w = ( Collision_h.x - In.Camera.location.m_origin.x ) * Delta_h.w;

				Offset_h.x = VoxelBlockSize;
				Offset_h.y = Delta_h.y * VoxelBlockSize;
				Offset_h.z = Delta_h.z * VoxelBlockSize;
				Offset_h.w = Delta_h.w * VoxelBlockSize;
				Norm_h.x = Offset_h.x / ( VoxelBlockSize / 2 );
				Norm_h.y = Offset_h.y / ( VoxelBlockSize / 2 );
				Norm_h.z = Offset_h.z / ( VoxelBlockSize / 2 );
			}
			else if( In.Camera.location.m_basis.m_el0.x <= -0.01 )
			{
				Collide_X = true;

				Delta_h.y = Norm.y / Norm.y;
				Delta_h.z = Norm.z / Norm.z;
				Delta_h.w = 1.0f / Math.Abs( Norm.x );

				Collision_h.x = ( (float)Math.Floor( In.Camera.location.m_origin.x / VoxelBlockSize ) ) * VoxelBlockSize;
				Collision_h.y = ( In.Camera.location.m_origin.x - Collision_h.x ) * Delta_h.y + In.Camera.location.m_origin.y;
				Collision_h.z = ( In.Camera.location.m_origin.x - Collision_h.x ) * Delta_h.z + In.Camera.location.m_origin.z;
				Collision_h.w = ( In.Camera.location.m_origin.x - Collision_h.x ) * Delta_h.w;
				Offset_h.x = -VoxelBlockSize;
				Offset_h.y = Delta_h.y * VoxelBlockSize;
				Offset_h.z = Delta_h.z * VoxelBlockSize;
				Offset_h.w = Delta_h.w * VoxelBlockSize;
				Norm_h.x = Offset_h.x / ( VoxelBlockSize / 2 );
				Norm_h.y = Offset_h.y / ( VoxelBlockSize / 2 );
				Norm_h.z = Offset_h.z / ( VoxelBlockSize / 2 );
			}

			if( In.Camera.location.m_basis.m_el1.y >= 0.01 )
			{
				Collide_Y = true;
				Delta_v.x = Norm.x / Norm.y;
				Delta_v.z = Norm.z / -Norm.y;
				Delta_v.w = 1 / Norm.y;
				Collision_v.y = ( (float)Math.Floor( In.Camera.location.m_origin.y / VoxelBlockSize ) ) * VoxelBlockSize;
				Collision_v.x = ( In.Camera.location.m_origin.y - Collision_v.y ) * Delta_v.x + In.Camera.location.m_origin.x;
				Collision_v.z = ( In.Camera.location.m_origin.y - Collision_v.y ) * Delta_v.z + In.Camera.location.m_origin.z;
				Collision_v.w = ( In.Camera.location.m_origin.y - Collision_v.y ) * Delta_v.w;
				Offset_v.y = -VoxelBlockSize;
				Offset_v.x = Delta_v.x * VoxelBlockSize;
				Offset_v.z = Delta_v.z * VoxelBlockSize;
				Offset_v.w = Delta_v.w * VoxelBlockSize;
				Norm_v.x = Offset_v.x / ( VoxelBlockSize / 2 );
				Norm_v.y = Offset_v.y / ( VoxelBlockSize / 2 );
				Norm_v.z = Offset_v.z / ( VoxelBlockSize / 2 );
			}
			else if( In.Camera.location.m_basis.m_el1.y <= -0.01 )
			{
				Collide_Y = true;
				Delta_v.x = Norm.x / -Norm.y;
				Delta_v.z = Norm.z / +Norm.y;
				Delta_v.w = 1.0f / -Norm.y;
				Collision_v.y = ( (float)Math.Floor( In.Camera.location.m_origin.y / VoxelBlockSize ) + 1 ) * VoxelBlockSize;
				Collision_v.x = ( Collision_v.y - In.Camera.location.m_origin.y ) * Delta_v.x + In.Camera.location.m_origin.x;
				Collision_v.z = ( Collision_v.y - In.Camera.location.m_origin.y ) * Delta_v.z + In.Camera.location.m_origin.z;
				Collision_v.w = ( Collision_v.y - In.Camera.location.m_origin.y ) * Delta_v.w;

				Offset_v.y = VoxelBlockSize;
				Offset_v.x = Delta_v.x * VoxelBlockSize;
				Offset_v.z = Delta_v.z * VoxelBlockSize;
				Offset_v.w = Delta_v.w * VoxelBlockSize;
				Norm_v.x = Offset_v.x / ( VoxelBlockSize / 2 );
				Norm_v.y = Offset_v.y / ( VoxelBlockSize / 2 );
				Norm_v.z = Offset_v.z / ( VoxelBlockSize / 2 );
			}

			if( In.Camera.location.m_basis.m_el2.z >= 0.01 )
			{
				Collide_Z = true;
				Delta_s.x = Norm.x / -Norm.z;
				Delta_s.y = Norm.y / Norm.z;
				Delta_s.w = 1.0f / -Norm.z;
				Collision_s.z = ( (float)Math.Floor( In.Camera.location.m_origin.z / VoxelBlockSize ) + 1.0f ) * VoxelBlockSize;
				Collision_s.x = ( Collision_s.z - In.Camera.location.m_origin.z ) * Delta_s.x + In.Camera.location.m_origin.x;
				Collision_s.y = ( Collision_s.z - In.Camera.location.m_origin.z ) * Delta_s.y + In.Camera.location.m_origin.y;
				Collision_s.w = ( Collision_s.z - In.Camera.location.m_origin.z ) * Delta_s.w;

				Offset_s.z = VoxelBlockSize;
				Offset_s.x = Delta_s.x * VoxelBlockSize;
				Offset_s.y = Delta_s.y * VoxelBlockSize;
				Offset_s.w = Delta_s.w * VoxelBlockSize;
				Norm_s.x = Offset_s.x / ( VoxelBlockSize / 2 );
				Norm_s.y = Offset_s.y / ( VoxelBlockSize / 2 );
				Norm_s.z = Offset_s.z / ( VoxelBlockSize / 2 );
			}
			else if( In.Camera.location.m_basis.m_el2.z <= -0.01 )
			{
				Collide_Z = true;
				Delta_s.x = Norm.x / +Norm.z;
				Delta_s.y = Norm.y / -Norm.z;
				Delta_s.w = 1.0f / Norm.z;
				Collision_s.z = ( (float)Math.Floor( In.Camera.location.m_origin.z / VoxelBlockSize ) ) * VoxelBlockSize;
				Collision_s.x = ( In.Camera.location.m_origin.z - Collision_s.z ) * Delta_s.x + In.Camera.location.m_origin.x;
				Collision_s.y = ( In.Camera.location.m_origin.z - Collision_s.z ) * Delta_s.y + In.Camera.location.m_origin.y;
				Collision_s.w = ( In.Camera.location.m_origin.z - Collision_s.z ) * Delta_s.w;
				Offset_s.z = -VoxelBlockSize;
				Offset_s.x = Delta_s.x * VoxelBlockSize;
				Offset_s.y = Delta_s.y * VoxelBlockSize;
				Offset_s.w = Delta_s.w * VoxelBlockSize;
				Norm_s.x = Offset_s.x / ( VoxelBlockSize / 2 );
				Norm_s.y = Offset_s.y / ( VoxelBlockSize / 2 );
				Norm_s.z = Offset_s.z / ( VoxelBlockSize / 2 );
			}



			//  printf("yaw: %04lf pitch: %lf Offset_y:%lf Offset_z:%lf xyz:%lf %lf %lf NXYZ:%lf %lf %lf Dxyz:%lf %lf %lf", yaw,pitch, Delta_h.Y, Delta_h.Z,x,y,Z, Norm_h.X, Norm_h.Y, Norm_h.Z, Delta_h.X, Delta_h.Y, Delta_h.Z);
			//printf("Angle (y:%lf p:%lf) XYZ:(%lf %lf %lf) Off(%lf %lf %lf %lf) Coll(%lf %lf %lf %lf) Norm(%lg %lg %lf) :\n", yaw,pitch,x,y,Z, Offset_s.X, Offset_s.Y, Offset_s.Z, Offset_s.w, Collision_s.X, Collision_s.Y, Collision_s.Z, Collision_s.w, Norm_s.X,Norm_s.Y, Norm_s.Z);

			int Match_h = 0;
			int Match_s = 0;
			int Match_v = 0;
			int Cycle = 1;
			float MinW = 1000000.0f;

			for( i = 0; i < 150; i++ )
			{

				// Horizontal X axis.
				if( Collide_X )
				{
					if( Match_h == 0 && Collision_h.w < MinW )
					{
						ActualCube_x = (int)Math.Floor( ( Collision_h.x - Norm_h.x ) / VoxelBlockSize ); ActualCube_y = (int)Math.Floor( ( Collision_h.y - Norm_h.y ) / VoxelBlockSize ); ActualCube_z = (int)Math.Floor( ( Collision_h.z - Norm_h.z ) / VoxelBlockSize );
						NewCube_x = (int)Math.Floor( ( Collision_h.x + Norm_h.x ) / VoxelBlockSize ); NewCube_y = (int)Math.Floor( ( Collision_h.y + Norm_h.y ) / VoxelBlockSize ); NewCube_z = (int)Math.Floor( ( Collision_h.z + Norm_h.z ) / VoxelBlockSize );
						if( GetVoxel( NewCube_x, NewCube_y, NewCube_z ) > 0 )
						{
							Out.PredPointedVoxel.X = ActualCube_x; Out.PredPointedVoxel.Y = ActualCube_y; Out.PredPointedVoxel.Z = ActualCube_z;
							Out.PointedVoxel.X = NewCube_x; Out.PointedVoxel.Y = NewCube_y; Out.PointedVoxel.Z = NewCube_z;
							// printf(" MATCH_H: %lf\n",Collision_h.w);
							Match_h = Cycle;
							MinW = Collision_h.w;
						}
					}
				}

				// Horizontal z axis.

				if( Collide_Z )
				{
					if( Match_s == 0 && Collision_s.w < MinW )
					{
						ActualCube_x = (int)Math.Floor( ( Collision_s.x - Norm_s.x ) / VoxelBlockSize ); ActualCube_y = (int)Math.Floor( ( Collision_s.y - Norm_s.y ) / VoxelBlockSize ); ActualCube_z = (int)Math.Floor( ( Collision_s.z - Norm_s.z ) / VoxelBlockSize );
						NewCube_x = (int)Math.Floor( ( Collision_s.x + Norm_s.x ) / VoxelBlockSize ); NewCube_y = (int)Math.Floor( ( Collision_s.y + Norm_s.y ) / VoxelBlockSize ); NewCube_z = (int)Math.Floor( ( Collision_s.z + Norm_s.z ) / VoxelBlockSize );
						if( GetVoxel( NewCube_x, NewCube_y, NewCube_z ) > 0 )
						{
							Out.PredPointedVoxel.X = ActualCube_x; Out.PredPointedVoxel.Y = ActualCube_y; Out.PredPointedVoxel.Z = ActualCube_z;
							Out.PointedVoxel.X = NewCube_x; Out.PointedVoxel.Y = NewCube_y; Out.PointedVoxel.Z = NewCube_z;
							// printf(" MATCH_S: %lf\n",Collision_s.w);
							Match_s = Cycle;
							MinW = Collision_s.w;
						}
					}
				}

				// Vertical y axis.

				if( Collide_Y )
				{
					if( Match_v == 0 && Collision_v.w < MinW )
					{
						ActualCube_x = (int)Math.Floor( ( Collision_v.x - Norm_v.x ) / VoxelBlockSize ); ActualCube_y = (int)Math.Floor( ( Collision_v.y - Norm_v.y ) / VoxelBlockSize ); ActualCube_z = (int)Math.Floor( ( Collision_v.z - Norm_v.z ) / VoxelBlockSize );
						NewCube_x = (int)Math.Floor( ( Collision_v.x + Norm_v.x ) / VoxelBlockSize ); NewCube_y = (int)Math.Floor( ( Collision_v.y + Norm_v.y ) / VoxelBlockSize ); NewCube_z = (int)Math.Floor( ( Collision_v.z + Norm_v.z ) / VoxelBlockSize );
						if( GetVoxel( NewCube_x, NewCube_y, NewCube_z ) > 0 )
						{
							Out.PredPointedVoxel.X = ActualCube_x; Out.PredPointedVoxel.Y = ActualCube_y; Out.PredPointedVoxel.Z = ActualCube_z;
							Out.PointedVoxel.X = NewCube_x; Out.PointedVoxel.Y = NewCube_y; Out.PointedVoxel.Z = NewCube_z;
							// printf(" MATCH_V: %lf\n",Collision_v.w);
							Match_v = Cycle;
							MinW = Collision_v.w;
						}
					}
				}

				//printf(" Match (H:%lf S:%lf V:%lf) \n", Collision_h.w, Collision_s.w, Collision_v.w);
				if( Match_h > 0 && ( Match_h - Cycle ) < -100 ) return ( true );
				if( Match_s > 0 && ( Match_s - Cycle ) < -100 ) return ( true );
				if( Match_v > 0 && ( Match_v - Cycle ) < -100 ) return ( true );

				Collision_h.x += Offset_h.x; Collision_h.y += Offset_h.y; Collision_h.z += Offset_h.z; Collision_h.w += Offset_h.w;
				Collision_v.x += Offset_v.x; Collision_v.y += Offset_v.y; Collision_v.z += Offset_v.z; Collision_v.w += Offset_v.w;
				Collision_s.x += Offset_s.x; Collision_s.y += Offset_s.y; Collision_s.z += Offset_s.z; Collision_s.w += Offset_s.w;
				Cycle++;
			}
			// printf("\n");
			return ( false );
			//printf("first_h_x : %lf first_h_y %lf\n",first_h_x,first_h_y);
		}

		bool RayCast_Vector( ref btTransform Pos, ref btVector3 Vector, ref RayCast_in In, ref RayCast_out Out, bool InvertCollision )
		{
			return RayCast_Vector( ref Pos.m_origin, ref Vector, ref In, ref Out, InvertCollision );
		}

		bool RayCast_Vector( ref btVector3 Pos, ref btVector3 Vector, ref RayCast_in In, ref RayCast_out Out, bool InvertCollision )
		{
			btVector3 Delta_h, Delta_v, Delta_s;
			btVector3 Offset_h = btVector3.Zero, Offset_v = btVector3.Zero, Offset_s = btVector3.Zero;
			btVector3 Norm_h = btVector3.Zero, Norm_v = btVector3.Zero, Norm_s = btVector3.Zero;
			btVector3 Collision_h = btVector3.Zero, Collision_v = btVector3.Zero, Collision_s = btVector3.Zero;
			int ActualCube_x_h, ActualCube_y_h, ActualCube_z_h;
			int ActualCube_x_v, ActualCube_y_v, ActualCube_z_v;
			int ActualCube_x_s, ActualCube_y_s, ActualCube_z_s;
			int NewCube_x_h, NewCube_y_h, NewCube_z_h;
			int NewCube_x_v, NewCube_y_v, NewCube_z_v;
			int NewCube_x_s, NewCube_y_s, NewCube_z_s;
			bool Collide_X, Collide_Y, Collide_Z;
			int i;
			byte Face_h, Face_s, Face_v;
			ushort VoxelType;

			btVector3 Norm;
			btVector3 Tmp;

			float Vector_Len;

			// Normalize input vector.
			//Vector_Len = Math.Sqrt( Vector.x * Vector.x + Vector.y * Vector.y + Vector.z * Vector.z);
			Norm = Vector;// / Vector_Len;

			// Norm = Vector;

			//printf("Norm:%lf %lf %lf\n",Norm.x,Norm.y,Norm.z);
			Collide_X = Collide_Y = Collide_Z = false;
			Face_h = Face_s = Face_v = 0;

			if( Norm.x > 0.00000001 )
			{
				Face_h = 2;
				Collide_X = true;
				Delta_h.y = Norm.y / Norm.x;
				Delta_h.z = Norm.z / Norm.x;
				Delta_h.w = 1.0f / Norm.x;
				Collision_h.x = (float)( Math.Floor( Pos.x / VoxelBlockSize ) + 1.0f ) * VoxelBlockSize;
				Collision_h.y = ( Collision_h.x - Pos.x ) * Delta_h.y + Pos.y;
				Collision_h.z = ( Collision_h.x - Pos.x ) * Delta_h.z + Pos.z;
				Collision_h.w = ( Collision_h.x - Pos.x ) * Delta_h.w;

				Offset_h.x = VoxelBlockSize;
				Offset_h.y = Delta_h.y * VoxelBlockSize;
				Offset_h.z = Delta_h.z * VoxelBlockSize;
				Offset_h.w = Delta_h.w * VoxelBlockSize;
				Norm_h.x = Offset_h.x / ( VoxelBlockSize / 2 );
				Norm_h.y = Offset_h.y / ( VoxelBlockSize / 2 );
				Norm_h.z = Offset_h.z / ( VoxelBlockSize / 2 );
			}
			else if( Norm.x < -0.00000001 )
			{
				Face_h = 3;
				Collide_X = true;
				Delta_h.y = Norm.y / -Norm.x;
				Delta_h.z = Norm.z / -Norm.x;
				Delta_h.w = 1.0f / Math.Abs( Norm.x );

				Collision_h.x = (float)( Math.Floor( Pos.x / VoxelBlockSize ) ) * VoxelBlockSize; // - 1.0
				Collision_h.y = ( Pos.x - Collision_h.x ) * Delta_h.y + Pos.y;
				Collision_h.z = ( Pos.x - Collision_h.x ) * Delta_h.z + Pos.z;
				Collision_h.w = ( Pos.x - Collision_h.x ) * Delta_h.w;
				Offset_h.x = -VoxelBlockSize;
				Offset_h.y = Delta_h.y * VoxelBlockSize;
				Offset_h.z = Delta_h.z * VoxelBlockSize;
				Offset_h.w = Delta_h.w * VoxelBlockSize;
				Norm_h.x = Offset_h.x / ( VoxelBlockSize / 2 );
				Norm_h.y = Offset_h.y / ( VoxelBlockSize / 2 );
				Norm_h.z = Offset_h.z / ( VoxelBlockSize / 2 );
			}


			if( Norm.y > 0.00000001 )
			{
				Face_v = 5;
				Collide_Y = true;
				Delta_v.x = Norm.x / Norm.y;
				Delta_v.z = Norm.z / Norm.y;
				Delta_v.w = 1 / Norm.y;
				Collision_v.y = (float)( Math.Floor( Pos.y / VoxelBlockSize ) + 1.0 ) * VoxelBlockSize;
				Collision_v.x = ( Collision_v.y - Pos.y ) * Delta_v.x + Pos.x;
				Collision_v.z = ( Collision_v.y - Pos.y ) * Delta_v.z + Pos.z;
				Collision_v.w = ( Collision_v.y - Pos.y ) * Delta_v.w;
				Offset_v.y = VoxelBlockSize;
				Offset_v.x = Delta_v.x * VoxelBlockSize;
				Offset_v.z = Delta_v.z * VoxelBlockSize;
				Offset_v.w = Delta_v.w * VoxelBlockSize;
				Norm_v.x = Offset_v.x / ( VoxelBlockSize / 2 );
				Norm_v.y = Offset_v.y / ( VoxelBlockSize / 2 );
				Norm_v.z = Offset_v.z / ( VoxelBlockSize / 2 );
			}
			else if( Norm.y < -0.00000001 )
			{
				Face_v = 4;
				Collide_Y = true;
				Delta_v.x = Norm.x / -Norm.y;
				Delta_v.z = Norm.z / -Norm.y;
				Delta_v.w = 1.0f / -Norm.y;
				Collision_v.y = ( (float)Math.Floor( Pos.y / VoxelBlockSize ) ) * VoxelBlockSize; // - 1.0
				Collision_v.x = ( Pos.y - Collision_v.y ) * Delta_v.x + Pos.x;
				Collision_v.z = ( Pos.y - Collision_v.y ) * Delta_v.z + Pos.z;
				Collision_v.w = Math.Abs( ( Collision_v.y - Pos.y ) * Delta_v.w );

				Offset_v.y = -VoxelBlockSize;
				Offset_v.x = Delta_v.x * VoxelBlockSize;
				Offset_v.z = Delta_v.z * VoxelBlockSize;
				Offset_v.w = Delta_v.w * VoxelBlockSize;
				Norm_v.x = Offset_v.x / ( VoxelBlockSize / 2 );
				Norm_v.y = Offset_v.y / ( VoxelBlockSize / 2 );
				Norm_v.z = Offset_v.z / ( VoxelBlockSize / 2 );
			}

			if( Norm.z > 0.00000001 )
			{
				Face_s = 0;
				Collide_Z = true;
				Delta_s.x = Norm.x / Norm.z;
				Delta_s.y = Norm.y / Norm.z;
				Delta_s.w = 1.0f / Norm.z;
				Collision_s.z = ( (float)Math.Floor( Pos.z / VoxelBlockSize + 1.0 ) ) * VoxelBlockSize;
				Collision_s.x = ( Collision_s.z - Pos.z ) * Delta_s.x + Pos.x;
				Collision_s.y = ( Collision_s.z - Pos.z ) * Delta_s.y + Pos.y;
				Collision_s.w = ( Collision_s.z - Pos.z ) * Delta_s.w;

				Offset_s.z = VoxelBlockSize;
				Offset_s.x = Delta_s.x * VoxelBlockSize;
				Offset_s.y = Delta_s.y * VoxelBlockSize;
				Offset_s.w = Delta_s.w * VoxelBlockSize;
				Norm_s.x = Offset_s.x / ( VoxelBlockSize / 2 );
				Norm_s.y = Offset_s.y / ( VoxelBlockSize / 2 );
				Norm_s.z = Offset_s.z / ( VoxelBlockSize / 2 );
			}
			else if( Norm.z < -0.00000001 )
			{
				Face_s = 1;
				Collide_Z = true;
				Delta_s.x = Norm.x / -Norm.z;
				Delta_s.y = Norm.y / -Norm.z;
				Delta_s.w = 1.0f / -Norm.z;
				Collision_s.z = ( (float)Math.Floor( Pos.z / VoxelBlockSize ) ) * VoxelBlockSize; // - 1.0
				Collision_s.x = ( Pos.z - Collision_s.z ) * Delta_s.x + Pos.x;
				Collision_s.y = ( Pos.z - Collision_s.z ) * Delta_s.y + Pos.y;
				Collision_s.w = ( Pos.z - Collision_s.z ) * Delta_s.w;
				Offset_s.z = -VoxelBlockSize;
				Offset_s.x = Delta_s.x * VoxelBlockSize;
				Offset_s.y = Delta_s.y * VoxelBlockSize;
				Offset_s.w = Delta_s.w * VoxelBlockSize;
				Norm_s.x = Offset_s.x / ( VoxelBlockSize / 2 );
				Norm_s.y = Offset_s.y / ( VoxelBlockSize / 2 );
				Norm_s.z = Offset_s.z / ( VoxelBlockSize / 2 );
			}
			/*
			 printf ("Loc(%lf %lf %lf) Norm(%lf %lf %lf) Col(%lf %lf %lf %lf) Off(%lf %lf %lf %lf) C(%d,%d,%d)\n", Pos.x, Pos.y, Pos.z, Norm.x,Norm.y, Norm.z, Collision_s.x, Collision_s.y, Collision_s.z, Collision_s.w, Offset_s.x,Offset_s.y, Offset_s.z,Offset_s.w
				 ,(ushort)((Collide_X==true)? 1:0) ,(ushort)((Collide_Y==true)? 1:0), (ushort)((Collide_Z==true)? 1:0));
		  */
			//  printf("yaw: %04lf pitch: %lf Offset_y:%lf Offset_z:%lf xyz:%lf %lf %lf NXYZ:%lf %lf %lf Dxyz:%lf %lf %lf", yaw,pitch, Delta_h.y, Delta_h.z,x,y,z, Norm_h.x, Norm_h.y, Norm_h.z, Delta_h.x, Delta_h.y, Delta_h.z);
			//printf("Angle (y:%lf p:%lf) XYZ:(%lf %lf %lf) Off(%lf %lf %lf %lf) Coll(%lf %lf %lf %lf) Norm(%lg %lg %lf) :\n", yaw,pitch,x,y,z, Offset_s.x, Offset_s.y, Offset_s.z, Offset_s.w, Collision_s.x, Collision_s.y, Collision_s.z, Collision_s.w, Norm_s.x,Norm_s.y, Norm_s.z);

			int Match_h = 0;
			int Match_s = 0;
			int Match_v = 0;
			int Cycle = 1;
			float MinW = 10000000.0f;

			for( i = 0; i < In.MaxCubeIterations; i++ )
			{
				// if  ( (Collision_h.w > In.MaxDetectionDistance) || (Collision_s.w > In.MaxDetectionDistance) || (Collision_v.w > In.MaxDetectionDistance)) { Out.Collided = false; return(false); }
				// Horizontal x axis.
				if( Collide_X )
				{
					if( Match_h == 0 && Collision_h.w < MinW )
					{
						ActualCube_x_h = (int)Math.Floor( ( Collision_h.x - Norm_h.x ) / VoxelBlockSize );
						ActualCube_y_h = (int)Math.Floor( ( Collision_h.y - Norm_h.y ) / VoxelBlockSize );
						ActualCube_z_h = (int)Math.Floor( ( Collision_h.z - Norm_h.z ) / VoxelBlockSize );
						NewCube_x_h = (int)Math.Floor( ( Collision_h.x ) / VoxelBlockSize );
						NewCube_y_h = (int)Math.Floor( ( Collision_h.y ) / VoxelBlockSize );
						NewCube_z_h = (int)Math.Floor( ( Collision_h.z ) / VoxelBlockSize );
						//ActualCube_x = (int)Math.Floor((Collision_h.x - Norm_h.x) / VoxelBlockSize); ActualCube_y = (int)Math.Floor((Collision_h.y - Norm_h.y) / VoxelBlockSize); ActualCube_z = (int)Math.Floor((Collision_h.z - Norm_h.z) / VoxelBlockSize);
						//NewCube_x = (int)Math.Floor((Collision_h.x + Norm_h.x) / VoxelBlockSize); NewCube_y = (int)Math.Floor((Collision_h.y + Norm_h.y) / VoxelBlockSize); NewCube_z = (int)Math.Floor((Collision_h.z + Norm_h.z) / VoxelBlockSize);
						if( Face_h == 3 ) NewCube_x_h--;

						VoxelType = GetVoxel( NewCube_x_h, NewCube_y_h, NewCube_z_h );

						if( !VoxelTypeManager.VoxelTable[VoxelType].properties.Is_PlayerCanPassThrough ^ InvertCollision )
						{
							Out.PredPointedVoxel.X = ActualCube_x_h; Out.PredPointedVoxel.Y = ActualCube_y_h; Out.PredPointedVoxel.Z = ActualCube_z_h;
							Out.PointedVoxel.X = NewCube_x_h; Out.PointedVoxel.Y = NewCube_y_h; Out.PointedVoxel.Z = NewCube_z_h;
							Out.CollisionPoint.x = Collision_h.x; Out.CollisionPoint.y = Collision_h.y; Out.CollisionPoint.z = Collision_h.z; Out.CollisionDistance = Collision_h.w;
							Out.CollisionAxe = 0; Out.CollisionFace = Face_h;
							Out.PointInCubeFace.X = ( Out.CollisionPoint.z % VoxelBlockSize ); Out.PointInCubeFace.Y = ( Out.CollisionPoint.y % VoxelBlockSize );
							//printf(" MATCH_H: %lf (%ld %ld %ld) C:%ld\n",Collision_h.w, NewCube_x_h, NewCube_y_h, NewCube_z_h, Cycle);
							Match_h = Cycle;
							MinW = Collision_h.w;
						}
					}
				}

				// Horizontal z axis.

				if( Collide_Z )
				{
					if( Match_s == 0 && Collision_s.w < MinW )
					{
						ActualCube_x_s = (int)Math.Floor( ( Collision_s.x - Norm_s.x ) / VoxelBlockSize ); ActualCube_y_s = (int)Math.Floor( ( Collision_s.y - Norm_s.y ) / VoxelBlockSize ); ActualCube_z_s = (int)Math.Floor( ( Collision_s.z - Norm_s.z ) / VoxelBlockSize );
						NewCube_x_s = (int)Math.Floor( ( Collision_s.x ) / VoxelBlockSize ); NewCube_y_s = (int)Math.Floor( ( Collision_s.y ) / VoxelBlockSize ); NewCube_z_s = (int)Math.Floor( ( Collision_s.z ) / VoxelBlockSize );
						//ActualCube_x_s = (int)Math.Floor((Collision_s.x - Norm_s.x) / VoxelBlockSize); ActualCube_y_s = (int)Math.Floor((Collision_s.y - Norm_s.y) / VoxelBlockSize); ActualCube_z_s = (int)Math.Floor((Collision_s.z - Norm_s.z) / VoxelBlockSize);
						//NewCube_x_s = (int)Math.Floor((Collision_s.x + Norm_s.x) / VoxelBlockSize); NewCube_y_s = (int)Math.Floor((Collision_s.y + Norm_s.y) / VoxelBlockSize); NewCube_z_s = (int)Math.Floor((Collision_s.z + Norm_s.z) / VoxelBlockSize);
						if( Face_s == 1 ) NewCube_z_s--;

						VoxelType = GetVoxel( NewCube_x_s, NewCube_y_s, NewCube_z_s );

						if( !VoxelTypeManager.VoxelTable[VoxelType].properties.Is_PlayerCanPassThrough ^ InvertCollision )
						{
							Out.PredPointedVoxel.X = ActualCube_x_s; Out.PredPointedVoxel.Y = ActualCube_y_s; Out.PredPointedVoxel.Z = ActualCube_z_s;
							Out.PointedVoxel.X = NewCube_x_s; Out.PointedVoxel.Y = NewCube_y_s; Out.PointedVoxel.Z = NewCube_z_s;
							Out.CollisionPoint.x = Collision_s.x; Out.CollisionPoint.y = Collision_s.y; Out.CollisionPoint.z = Collision_s.z; Out.CollisionDistance = Collision_s.w;
							Out.CollisionAxe = 2; Out.CollisionFace = Face_s;
							Out.PointInCubeFace.X = ( Out.CollisionPoint.x % VoxelBlockSize ); Out.PointInCubeFace.Y = ( Out.CollisionPoint.y % VoxelBlockSize );

							//printf(" MATCH_S: %lf (%ld %ld %ld) C:%ld\n",Collision_s.w, NewCube_x_s, NewCube_y_s, NewCube_z_s, Cycle);
							Match_s = Cycle;
							MinW = Collision_s.w;
						}
					}
				}

				// Vertical y axis.

				if( Collide_Y )
				{
					if( Match_v == 0 && Collision_v.w < MinW )
					{
						ActualCube_x_v = (int)Math.Floor( ( Collision_v.x - Norm_v.x ) / VoxelBlockSize ); ActualCube_y_v = (int)Math.Floor( ( Collision_v.y - Norm_v.y ) / VoxelBlockSize ); ActualCube_z_v = (int)Math.Floor( ( Collision_v.z - Norm_v.z ) / VoxelBlockSize );
						NewCube_x_v = (int)Math.Floor( ( Collision_v.x ) / VoxelBlockSize ); NewCube_y_v = (int)Math.Floor( ( Collision_v.y ) / VoxelBlockSize ); NewCube_z_v = (int)Math.Floor( ( Collision_v.z ) / VoxelBlockSize );
						//ActualCube_x_v = (int)Math.Floor((Collision_v.x - Norm_v.x) / VoxelBlockSize);   ActualCube_y_v = (int)Math.Floor((Collision_v.y - Norm_v.y) / VoxelBlockSize);   ActualCube_z_v = (int)Math.Floor((Collision_v.z - Norm_v.z) / VoxelBlockSize);
						//NewCube_x_v = (int)Math.Floor((Collision_v.x + Norm_v.x) / VoxelBlockSize); NewCube_y_v = (int)Math.Floor((Collision_v.y + Norm_v.y) / VoxelBlockSize); NewCube_z_v = (int)Math.Floor((Collision_v.z + Norm_v.z) / VoxelBlockSize);
						if( Face_v == 4 ) NewCube_y_v--;

						VoxelType = GetVoxel( NewCube_x_v, NewCube_y_v, NewCube_z_v );

						if( !VoxelTypeManager.VoxelTable[VoxelType].properties.Is_PlayerCanPassThrough ^ InvertCollision )
						{
							Out.PointedVoxel.X = NewCube_x_v; Out.PointedVoxel.Y = NewCube_y_v; Out.PointedVoxel.Z = NewCube_z_v;
							Out.CollisionPoint.x = Collision_v.x; Out.CollisionPoint.y = Collision_v.y; Out.CollisionPoint.z = Collision_v.z; Out.CollisionDistance = Collision_v.w;
							Out.PredPointedVoxel.X = ActualCube_x_v; Out.PredPointedVoxel.Y = ActualCube_y_v; Out.PredPointedVoxel.Z = ActualCube_z_v;
							Out.CollisionAxe = 1; Out.CollisionFace = Face_v;
							Out.PointInCubeFace.X = ( Out.CollisionPoint.x % VoxelBlockSize ); Out.PointInCubeFace.Y = ( Out.CollisionPoint.z % VoxelBlockSize );

							//printf(" MATCH_V: %lf (%ld %ld %ld) C:%ld\n",Collision_v.w, NewCube_x_v, NewCube_y_v, NewCube_z_v,Cycle );
							Match_v = Cycle;
							MinW = Collision_v.w;
						}
					}
				}

				//printf(" Match (H:%lf S:%lf V:%lf) \n", Collision_h.w, Collision_s.w, Collision_v.w);
				if( Match_h > 0 && ( Cycle - Match_h ) > In.PlaneCubeDiff ) { Out.Collided = true; return ( true ); }
				if( Match_s > 0 && ( Cycle - Match_s ) > In.PlaneCubeDiff ) { Out.Collided = true; return ( true ); }
				if( Match_v > 0 && ( Cycle - Match_v ) > In.PlaneCubeDiff ) { Out.Collided = true; return ( true ); }

				if( Collide_X )
					Collision_h.x += Offset_h.x; Collision_h.y += Offset_h.y; Collision_h.z += Offset_h.z; Collision_h.w += Offset_h.w;
				if( Collide_Y )
					Collision_v.x += Offset_v.x; Collision_v.y += Offset_v.y; Collision_v.z += Offset_v.z; Collision_v.w += Offset_v.w;
				if( Collide_Z )
					Collision_s.x += Offset_s.x; Collision_s.y += Offset_s.y; Collision_s.z += Offset_s.z; Collision_s.w += Offset_s.w;
				Cycle++;
			}
			// compute it as a delta and revert this change :/
			if( false )
			{
				if( ( !Collide_Y && Collide_X ) || ( Collide_Y && Collide_X && ( Collision_h.w < Collision_v.w ) ) )
				{
					if( Collide_Z && Collision_h.w < Collision_s.w )
					{
						Out.PredPointedVoxel.X = ActualCube_x_h; Out.PredPointedVoxel.Y = ActualCube_y_h; Out.PredPointedVoxel.Z = ActualCube_z_h;
						Out.PointedVoxel.X = NewCube_x_h; Out.PointedVoxel.Y = NewCube_y_h; Out.PointedVoxel.Z = NewCube_z_h;
					}
					else
					{
						if( ( Collide_Z && !Collide_Y ) || ( Collide_Z && Collide_Y && Collision_s.w < Collision_v.w ) )
						{
							Out.PredPointedVoxel.X = ActualCube_x_s; Out.PredPointedVoxel.Y = ActualCube_y_s; Out.PredPointedVoxel.Z = ActualCube_z_s;
							Out.PointedVoxel.X = NewCube_x_s; Out.PointedVoxel.Y = NewCube_y_s; Out.PointedVoxel.Z = NewCube_z_s;
						}
						else
						{
							Out.PredPointedVoxel.X = ActualCube_x_h; Out.PredPointedVoxel.Y = ActualCube_y_h; Out.PredPointedVoxel.Z = ActualCube_z_h;
							Out.PointedVoxel.X = NewCube_x_h; Out.PointedVoxel.Y = NewCube_y_h; Out.PointedVoxel.Z = NewCube_z_h;
						}
					}
				}
				else
				{
					if( ( Collide_Z && !Collide_Y ) || ( Collide_Y && Collide_Z && Collision_s.w < Collision_v.w ) )
					{
						Out.PredPointedVoxel.X = ActualCube_x_s; Out.PredPointedVoxel.Y = ActualCube_y_s; Out.PredPointedVoxel.Z = ActualCube_z_s;
						Out.PointedVoxel.X = NewCube_x_s; Out.PointedVoxel.Y = NewCube_y_s; Out.PointedVoxel.Z = NewCube_z_s;
					}
					else if( Collide_Y )
					{
						Out.PredPointedVoxel.X = ActualCube_x_v; Out.PredPointedVoxel.Y = ActualCube_y_v; Out.PredPointedVoxel.Z = ActualCube_z_v;
						Out.PointedVoxel.X = NewCube_x_v; Out.PointedVoxel.Y = NewCube_y_v; Out.PointedVoxel.Z = NewCube_z_v;
					}
				}
			}
			Out.Collided = false;
			Out.CollisionAxe = 0;
			Out.CollisionFace = 0;
			Out.CollisionDistance = 0.0f;
			Out.CollisionPoint = btVector3.Zero;
			return ( false );
			//printf("first_h_x : %lf first_h_y %lf\n",first_h_x,first_h_y);
		}

		bool RayCast_Vector2( ref btVector3 Pos, ref btVector3 Vector, ref RayCast_in In, ref RayCast_out Out, bool InvertCollision )
		{
			btVector3 Delta_h, Delta_v, Delta_s;
			btVector3 Offset_h = btVector3.Zero, Offset_v = btVector3.Zero, Offset_s = btVector3.Zero;
			btVector3 Norm_h = btVector3.Zero, Norm_v = btVector3.Zero, Norm_s = btVector3.Zero;
			btVector3 Collision_h = btVector3.Zero, Collision_v = btVector3.Zero, Collision_s = btVector3.Zero;
			int ActualCube_x, ActualCube_y, ActualCube_z;
			int NewCube_x, NewCube_y, NewCube_z;
			bool Collide_X, Collide_Y, Collide_Z;
			int i;
			byte Face_h, Face_s, Face_v;
			ushort VoxelType;

			btVector3 Norm;
			btVector3 Tmp;

			float Vector_Len;

			// Normalize input vector.
			Vector_Len = (float)Math.Sqrt( Vector.x * Vector.x + Vector.y * Vector.y + Vector.z * Vector.z );
			Vector.Div( Vector_Len, out Norm );

			// Norm = Vector;

			//printf("Norm:%lf %lf %lf\n",Norm.x,Norm.y,Norm.z);
			Collide_X = Collide_Y = Collide_Z = false;
			Face_h = Face_s = Face_v = 0;

			if( Norm.x > 0.00000001 )
			{
				Face_h = 2;
				Collide_X = true;
				Delta_h.y = Norm.y / Norm.x;
				Delta_h.z = Norm.z / Norm.x;
				Delta_h.w = 1.0f / Norm.x;
				Collision_h.x = ( (float)Math.Floor( Pos.x / VoxelBlockSize ) + 1.0f ) * VoxelBlockSize;
				Collision_h.y = ( Collision_h.x - Pos.x ) * Delta_h.y + Pos.y;
				Collision_h.z = ( Collision_h.x - Pos.x ) * Delta_h.z + Pos.z;
				Collision_h.w = ( Collision_h.x - Pos.x ) * Delta_h.w;

				Offset_h.x = VoxelBlockSize;
				Offset_h.y = Delta_h.y * VoxelBlockSize;
				Offset_h.z = Delta_h.z * VoxelBlockSize;
				Offset_h.w = Delta_h.w * VoxelBlockSize;
				Norm_h.x = Offset_h.x / ( VoxelBlockSize / 2 );
				Norm_h.y = Offset_h.y / ( VoxelBlockSize / 2 );
				Norm_h.z = Offset_h.z / ( VoxelBlockSize / 2 );
			}
			else if( Norm.x < -0.00000001 )
			{
				Face_h = 3;
				Collide_X = true;
				Delta_h.y = Norm.y / -Norm.x;
				Delta_h.z = Norm.z / -Norm.x;
				Delta_h.w = 1.0f / Math.Abs( Norm.x );

				Collision_h.x = ( (float)Math.Floor( Pos.x / VoxelBlockSize ) ) * VoxelBlockSize; // - 1.0
				Collision_h.y = ( Pos.x - Collision_h.x ) * Delta_h.y + Pos.y;
				Collision_h.z = ( Pos.x - Collision_h.x ) * Delta_h.z + Pos.z;
				Collision_h.w = ( Pos.x - Collision_h.x ) * Delta_h.w;
				Offset_h.x = -VoxelBlockSize;
				Offset_h.y = Delta_h.y * VoxelBlockSize;
				Offset_h.z = Delta_h.z * VoxelBlockSize;
				Offset_h.w = Delta_h.w * VoxelBlockSize;
				Norm_h.x = Offset_h.x / ( VoxelBlockSize / 2 );
				Norm_h.y = Offset_h.y / ( VoxelBlockSize / 2 );
				Norm_h.z = Offset_h.z / ( VoxelBlockSize / 2 );
			}


			if( Norm.y > 0.00000001 )
			{
				Face_v = 5;
				Collide_Y = true;
				Delta_v.x = Norm.x / Norm.y;
				Delta_v.z = Norm.z / Norm.y;
				Delta_v.w = 1 / Norm.y;
				Collision_v.y = ( (float)Math.Floor( Pos.y / VoxelBlockSize ) + 1.0f ) * VoxelBlockSize;
				Collision_v.x = ( Collision_v.y - Pos.y ) * Delta_v.x + Pos.x;
				Collision_v.z = ( Collision_v.y - Pos.y ) * Delta_v.z + Pos.z;
				Collision_v.w = ( Collision_v.y - Pos.y ) * Delta_v.w;
				Offset_v.y = VoxelBlockSize;
				Offset_v.x = Delta_v.x * VoxelBlockSize;
				Offset_v.z = Delta_v.z * VoxelBlockSize;
				Offset_v.w = Delta_v.w * VoxelBlockSize;
				Norm_v.x = Offset_v.x / ( VoxelBlockSize / 2 );
				Norm_v.y = Offset_v.y / ( VoxelBlockSize / 2 );
				Norm_v.z = Offset_v.z / ( VoxelBlockSize / 2 );
			}
			else if( Norm.y < -0.00000001 )
			{
				Face_v = 4;
				Collide_Y = true;
				Delta_v.x = Norm.x / -Norm.y;
				Delta_v.z = Norm.z / -Norm.y;
				Delta_v.w = 1.0f / -Norm.y;
				Collision_v.y = ( (float)Math.Floor( Pos.y / VoxelBlockSize ) ) * VoxelBlockSize; // - 1.0
				Collision_v.x = ( Pos.y - Collision_v.y ) * Delta_v.x + Pos.x;
				Collision_v.z = ( Pos.y - Collision_v.y ) * Delta_v.z + Pos.z;
				Collision_v.w = Math.Abs( ( Collision_v.y - Pos.y ) * Delta_v.w );

				Offset_v.y = -VoxelBlockSize;
				Offset_v.x = Delta_v.x * VoxelBlockSize;
				Offset_v.z = Delta_v.z * VoxelBlockSize;
				Offset_v.w = Delta_v.w * VoxelBlockSize;
				Norm_v.x = Offset_v.x / ( VoxelBlockSize / 2 );
				Norm_v.y = Offset_v.y / ( VoxelBlockSize / 2 );
				Norm_v.z = Offset_v.z / ( VoxelBlockSize / 2 );
			}

			if( Norm.z > 0.00000001 )
			{
				Face_s = 0;
				Collide_Z = true;
				Delta_s.x = Norm.x / Norm.z;
				Delta_s.y = Norm.y / Norm.z;
				Delta_s.w = 1.0f / Norm.z;
				Collision_s.z = ( (float)Math.Floor( Pos.z / VoxelBlockSize + 1.0f ) ) * VoxelBlockSize;
				Collision_s.x = ( Collision_s.z - Pos.z ) * Delta_s.x + Pos.x;
				Collision_s.y = ( Collision_s.z - Pos.z ) * Delta_s.y + Pos.y;
				Collision_s.w = ( Collision_s.z - Pos.z ) * Delta_s.w;

				Offset_s.z = VoxelBlockSize;
				Offset_s.x = Delta_s.x * VoxelBlockSize;
				Offset_s.y = Delta_s.y * VoxelBlockSize;
				Offset_s.w = Delta_s.w * VoxelBlockSize;
				Norm_s.x = Offset_s.x / ( VoxelBlockSize / 2 );
				Norm_s.y = Offset_s.y / ( VoxelBlockSize / 2 );
				Norm_s.z = Offset_s.z / ( VoxelBlockSize / 2 );
			}
			else if( Norm.z < -0.00000001 )
			{
				Face_s = 1;
				Collide_Z = true;
				Delta_s.x = Norm.x / -Norm.z;
				Delta_s.y = Norm.y / -Norm.z;
				Delta_s.w = 1.0f / -Norm.z;
				Collision_s.z = ( (float)Math.Floor( Pos.z / VoxelBlockSize ) ) * VoxelBlockSize; // - 1.0
				Collision_s.x = ( Pos.z - Collision_s.z ) * Delta_s.x + Pos.x;
				Collision_s.y = ( Pos.z - Collision_s.z ) * Delta_s.y + Pos.y;
				Collision_s.w = ( Pos.z - Collision_s.z ) * Delta_s.w;
				Offset_s.z = -VoxelBlockSize;
				Offset_s.x = Delta_s.x * VoxelBlockSize;
				Offset_s.y = Delta_s.y * VoxelBlockSize;
				Offset_s.w = Delta_s.w * VoxelBlockSize;
				Norm_s.x = Offset_s.x / ( VoxelBlockSize / 2 );
				Norm_s.y = Offset_s.y / ( VoxelBlockSize / 2 );
				Norm_s.z = Offset_s.z / ( VoxelBlockSize / 2 );
			}
			/*
			 printf ("Loc(%lf %lf %lf) Norm(%lf %lf %lf) Col(%lf %lf %lf %lf) Off(%lf %lf %lf %lf) C(%d,%d,%d)\n", Pos.x, Pos.y, Pos.z, Norm.x,Norm.y, Norm.z, Collision_s.x, Collision_s.y, Collision_s.z, Collision_s.w, Offset_s.x,Offset_s.y, Offset_s.z,Offset_s.w
				 ,(ushort)((Collide_X==true)? 1:0) ,(ushort)((Collide_Y==true)? 1:0), (ushort)((Collide_Z==true)? 1:0));
		  */
			//  printf("yaw: %04lf pitch: %lf Offset_y:%lf Offset_z:%lf xyz:%lf %lf %lf NXYZ:%lf %lf %lf Dxyz:%lf %lf %lf", yaw,pitch, Delta_h.y, Delta_h.z,x,y,z, Norm_h.x, Norm_h.y, Norm_h.z, Delta_h.x, Delta_h.y, Delta_h.z);
			//printf("Angle (y:%lf p:%lf) XYZ:(%lf %lf %lf) Off(%lf %lf %lf %lf) Coll(%lf %lf %lf %lf) Norm(%lg %lg %lf) :\n", yaw,pitch,x,y,z, Offset_s.x, Offset_s.y, Offset_s.z, Offset_s.w, Collision_s.x, Collision_s.y, Collision_s.z, Collision_s.w, Norm_s.x,Norm_s.y, Norm_s.z);

			int Match_h = 0;
			int Match_s = 0;
			int Match_v = 0;
			int Cycle = 1;
			float MinW = 10000000.0f;

			for( i = 0; i < In.MaxCubeIterations; i++ )
			{
				// if  ( (Collision_h.w > In.MaxDetectionDistance) || (Collision_s.w > In.MaxDetectionDistance) || (Collision_v.w > In.MaxDetectionDistance)) { Out.Collided = false; return(false); }
				// Horizontal x axis.
				if( Collide_X )
				{
					if( Match_h == 0 && Collision_h.w < MinW )
					{
						ActualCube_x = (int)Math.Floor( ( Collision_h.x - Norm_h.x ) / VoxelBlockSize ); ActualCube_y = (int)Math.Floor( ( Collision_h.y - Norm_h.y ) / VoxelBlockSize ); ActualCube_z = (int)Math.Floor( ( Collision_h.z - Norm_h.z ) / VoxelBlockSize );
						NewCube_x = (int)Math.Floor( ( Collision_h.x ) / VoxelBlockSize ); NewCube_y = (int)Math.Floor( ( Collision_h.y ) / VoxelBlockSize ); NewCube_z = (int)Math.Floor( ( Collision_h.z ) / VoxelBlockSize );
						//ActualCube_x = (int)Math.Floor((Collision_h.x - Norm_h.x) / VoxelBlockSize); ActualCube_y = (int)Math.Floor((Collision_h.y - Norm_h.y) / VoxelBlockSize); ActualCube_z = (int)Math.Floor((Collision_h.z - Norm_h.z) / VoxelBlockSize);
						//NewCube_x = (int)Math.Floor((Collision_h.x + Norm_h.x) / VoxelBlockSize); NewCube_y = (int)Math.Floor((Collision_h.y + Norm_h.y) / VoxelBlockSize); NewCube_z = (int)Math.Floor((Collision_h.z + Norm_h.z) / VoxelBlockSize);
						if( Face_h == 3 ) NewCube_x--;

						VoxelType = GetVoxel( NewCube_x, NewCube_y, NewCube_z );

						if( !VoxelTypeManager.VoxelTable[VoxelType].properties.Is_PlayerCanPassThrough ^ InvertCollision )
						{
							Out.PredPointedVoxel.X = ActualCube_x; Out.PredPointedVoxel.Y = ActualCube_y; Out.PredPointedVoxel.Z = ActualCube_z;
							Out.PointedVoxel.X = NewCube_x; Out.PointedVoxel.Y = NewCube_y; Out.PointedVoxel.Z = NewCube_z;
							Out.CollisionPoint.x = Collision_h.x; Out.CollisionPoint.y = Collision_h.y; Out.CollisionPoint.z = Collision_h.z; Out.CollisionDistance = Collision_h.w;
							Out.CollisionAxe = 0; Out.CollisionFace = Face_h;
							Out.PointInCubeFace.X = ( Out.CollisionPoint.z % VoxelBlockSize ); Out.PointInCubeFace.Y = ( Out.CollisionPoint.y % VoxelBlockSize );
							//printf(" MATCH_H: %lf (%ld %ld %ld) C:%ld\n",Collision_h.w, NewCube_x, NewCube_y, NewCube_z, Cycle);
							Match_h = Cycle;
							MinW = Collision_h.w;
						}
					}
				}

				// Horizontal z axis.

				if( Collide_Z )
				{
					if( Match_s == 0 && Collision_s.w < MinW )
					{
						ActualCube_x = (int)Math.Floor( ( Collision_s.x - Norm_s.x ) / VoxelBlockSize ); ActualCube_y = (int)Math.Floor( ( Collision_s.y - Norm_s.y ) / VoxelBlockSize ); ActualCube_z = (int)Math.Floor( ( Collision_s.z - Norm_s.z ) / VoxelBlockSize );
						NewCube_x = (int)Math.Floor( ( Collision_s.x ) / VoxelBlockSize ); NewCube_y = (int)Math.Floor( ( Collision_s.y ) / VoxelBlockSize ); NewCube_z = (int)Math.Floor( ( Collision_s.z ) / VoxelBlockSize );
						//ActualCube_x = (int)Math.Floor((Collision_s.x - Norm_s.x) / VoxelBlockSize); ActualCube_y = (int)Math.Floor((Collision_s.y - Norm_s.y) / VoxelBlockSize); ActualCube_z = (int)Math.Floor((Collision_s.z - Norm_s.z) / VoxelBlockSize);
						//NewCube_x = (int)Math.Floor((Collision_s.x + Norm_s.x) / VoxelBlockSize); NewCube_y = (int)Math.Floor((Collision_s.y + Norm_s.y) / VoxelBlockSize); NewCube_z = (int)Math.Floor((Collision_s.z + Norm_s.z) / VoxelBlockSize);
						if( Face_s == 1 ) NewCube_z--;

						VoxelType = GetVoxel( NewCube_x, NewCube_y, NewCube_z );

						if( !VoxelTypeManager.VoxelTable[VoxelType].properties.Is_PlayerCanPassThrough ^ InvertCollision )
						{
							Out.PredPointedVoxel.X = ActualCube_x; Out.PredPointedVoxel.Y = ActualCube_y; Out.PredPointedVoxel.Z = ActualCube_z;
							Out.PointedVoxel.X = NewCube_x; Out.PointedVoxel.Y = NewCube_y; Out.PointedVoxel.Z = NewCube_z;
							Out.CollisionPoint.x = Collision_s.x; Out.CollisionPoint.y = Collision_s.y; Out.CollisionPoint.z = Collision_s.z; Out.CollisionDistance = Collision_s.w;
							Out.CollisionAxe = 2; Out.CollisionFace = Face_s;
							Out.PointInCubeFace.X = ( Out.CollisionPoint.x % VoxelBlockSize ); Out.PointInCubeFace.Y = ( Out.CollisionPoint.y % VoxelBlockSize );

							//printf(" MATCH_S: %lf (%ld %ld %ld) C:%ld\n",Collision_s.w, NewCube_x, NewCube_y, NewCube_z, Cycle);
							Match_s = Cycle;
							MinW = Collision_s.w;
						}
					}
				}

				// Vertical y axis.

				if( Collide_Y )
				{
					if( Match_v == 0 && Collision_v.w < MinW )
					{
						ActualCube_x = (int)Math.Floor( ( Collision_v.x - Norm_v.x ) / VoxelBlockSize ); ActualCube_y = (int)Math.Floor( ( Collision_v.y - Norm_v.y ) / VoxelBlockSize ); ActualCube_z = (int)Math.Floor( ( Collision_v.z - Norm_v.z ) / VoxelBlockSize );
						NewCube_x = (int)Math.Floor( ( Collision_v.x ) / VoxelBlockSize ); NewCube_y = (int)Math.Floor( ( Collision_v.y ) / VoxelBlockSize ); NewCube_z = (int)Math.Floor( ( Collision_v.z ) / VoxelBlockSize );
						//ActualCube_x = (int)Math.Floor((Collision_v.x - Norm_v.x) / VoxelBlockSize);   ActualCube_y = (int)Math.Floor((Collision_v.y - Norm_v.y) / VoxelBlockSize);   ActualCube_z = (int)Math.Floor((Collision_v.z - Norm_v.z) / VoxelBlockSize);
						//NewCube_x = (int)Math.Floor((Collision_v.x + Norm_v.x) / VoxelBlockSize); NewCube_y = (int)Math.Floor((Collision_v.y + Norm_v.y) / VoxelBlockSize); NewCube_z = (int)Math.Floor((Collision_v.z + Norm_v.z) / VoxelBlockSize);
						if( Face_v == 4 ) NewCube_y--;

						VoxelType = GetVoxel( NewCube_x, NewCube_y, NewCube_z );

						if( !VoxelTypeManager.VoxelTable[VoxelType].properties.Is_PlayerCanPassThrough ^ InvertCollision )
						{
							Out.PredPointedVoxel.X = ActualCube_x; Out.PredPointedVoxel.Y = ActualCube_y; Out.PredPointedVoxel.Z = ActualCube_z;
							Out.PointedVoxel.X = NewCube_x; Out.PointedVoxel.Y = NewCube_y; Out.PointedVoxel.Z = NewCube_z;
							Out.CollisionPoint.x = Collision_v.x; Out.CollisionPoint.y = Collision_v.y; Out.CollisionPoint.z = Collision_v.z; Out.CollisionDistance = Collision_v.w;
							Out.CollisionAxe = 1; Out.CollisionFace = Face_v;
							Out.PointInCubeFace.X = ( Out.CollisionPoint.x % VoxelBlockSize ); Out.PointInCubeFace.Y = ( Out.CollisionPoint.z % VoxelBlockSize );

							//printf(" MATCH_V: %lf (%ld %ld %ld) C:%ld\n",Collision_v.w, NewCube_x, NewCube_y, NewCube_z,Cycle );
							Match_v = Cycle;
							MinW = Collision_v.w;
						}
					}
				}

				//printf(" Match (H:%lf S:%lf V:%lf) \n", Collision_h.w, Collision_s.w, Collision_v.w);
				if( Match_h > 0 && ( Cycle - Match_h ) > In.PlaneCubeDiff ) { Out.Collided = true; return ( true ); }
				if( Match_s > 0 && ( Cycle - Match_s ) > In.PlaneCubeDiff ) { Out.Collided = true; return ( true ); }
				if( Match_v > 0 && ( Cycle - Match_v ) > In.PlaneCubeDiff ) { Out.Collided = true; return ( true ); }

				if( Collide_X )
					Collision_h.x += Offset_h.x; Collision_h.y += Offset_h.y; Collision_h.z += Offset_h.z; Collision_h.w += Offset_h.w;
				if( Collide_Y )
					Collision_v.x += Offset_v.x; Collision_v.y += Offset_v.y; Collision_v.z += Offset_v.z; Collision_v.w += Offset_v.w;
				if( Collide_Z )
					Collision_s.x += Offset_s.x; Collision_s.y += Offset_s.y; Collision_s.z += Offset_s.z; Collision_s.w += Offset_s.w;
				Cycle++;
			}
			Out.Collided = false;
			Out.CollisionAxe = 0;
			Out.CollisionFace = 0;
			Out.CollisionDistance = 0.0f;
			Out.CollisionPoint = btVector3.Zero;
			return ( false );
			//printf("first_h_x : %lf first_h_y %lf\n",first_h_x,first_h_y);
		}
		bool RayCast_Vector_special( ref btVector3 Pos, ref btVector3 Vector, ref RayCast_in In, ref RayCast_out Out, bool InvertCollision )
		{
			btVector3 Delta_h, Delta_v, Delta_s;
			btVector3 Offset_h = btVector3.Zero, Offset_v = btVector3.Zero, Offset_s = btVector3.Zero;
			btVector3 Norm_h = btVector3.Zero, Norm_v = btVector3.Zero, Norm_s = btVector3.Zero;
			btVector3 Collision_h = btVector3.Zero, Collision_v = btVector3.Zero, Collision_s = btVector3.Zero;

			int ActualCube_x, ActualCube_y, ActualCube_z;
			int NewCube_x, NewCube_y, NewCube_z;
			bool Collide_X, Collide_Y, Collide_Z;
			int i;
			byte Face_h, Face_s, Face_v;
			ushort VoxelType;

			btVector3 Norm;
			btVector3 Tmp;

			float Vector_Len;

			bool Armed, DeferArmed;

			Armed = false; DeferArmed = false;


			NewCube_x = (int)Math.Floor( ( Pos.x ) / VoxelBlockSize ); NewCube_y = (int)Math.Floor( ( Pos.y ) / VoxelBlockSize ); NewCube_z = (int)Math.Floor( ( Pos.z ) / VoxelBlockSize );
			VoxelType = GetVoxel( NewCube_x, NewCube_y, NewCube_z );
			if( VoxelTypeManager.VoxelTable[VoxelType].properties.Is_PlayerCanPassThrough != InvertCollision ) Armed = true;



			// Normalize input vector.
			Vector_Len = (float)Math.Sqrt( Vector.x * Vector.x + Vector.y * Vector.y + Vector.z * Vector.z );
			Vector.Div( Vector_Len, out Norm );

			// Norm = Vector;

			//printf("Norm:%lf %lf %lf\n",Norm.x,Norm.y,Norm.z);
			Collide_X = Collide_Y = Collide_Z = false;
			Face_h = Face_s = Face_v = 0;

			if( Norm.x > 0.00000001 )
			{
				Face_h = 2;
				Collide_X = true;
				Delta_h.y = Norm.y / Norm.x;
				Delta_h.z = Norm.z / Norm.x;
				Delta_h.w = 1.0f / Norm.x;
				Collision_h.x = ( (float)Math.Floor( Pos.x / VoxelBlockSize ) + 1.0f ) * VoxelBlockSize;
				Collision_h.y = ( Collision_h.x - Pos.x ) * Delta_h.y + Pos.y;
				Collision_h.z = ( Collision_h.x - Pos.x ) * Delta_h.z + Pos.z;
				Collision_h.w = ( Collision_h.x - Pos.x ) * Delta_h.w;

				Offset_h.x = VoxelBlockSize;
				Offset_h.y = Delta_h.y * VoxelBlockSize;
				Offset_h.z = Delta_h.z * VoxelBlockSize;
				Offset_h.w = Delta_h.w * VoxelBlockSize;
				Norm_h.x = Offset_h.x / ( VoxelBlockSize / 2 );
				Norm_h.y = Offset_h.y / ( VoxelBlockSize / 2 );
				Norm_h.z = Offset_h.z / ( VoxelBlockSize / 2 );
			}
			else if( Norm.x < -0.00000001 )
			{
				Face_h = 3;
				Collide_X = true;
				Delta_h.y = Norm.y / -Norm.x;
				Delta_h.z = Norm.z / -Norm.x;
				Delta_h.w = 1.0f / Math.Abs( Norm.x );

				Collision_h.x = ( (float)Math.Floor( Pos.x / VoxelBlockSize ) ) * VoxelBlockSize; // - 1.0
				Collision_h.y = ( Pos.x - Collision_h.x ) * Delta_h.y + Pos.y;
				Collision_h.z = ( Pos.x - Collision_h.x ) * Delta_h.z + Pos.z;
				Collision_h.w = ( Pos.x - Collision_h.x ) * Delta_h.w;
				Offset_h.x = -VoxelBlockSize;
				Offset_h.y = Delta_h.y * VoxelBlockSize;
				Offset_h.z = Delta_h.z * VoxelBlockSize;
				Offset_h.w = Delta_h.w * VoxelBlockSize;
				Norm_h.x = Offset_h.x / ( VoxelBlockSize / 2 );
				Norm_h.y = Offset_h.y / ( VoxelBlockSize / 2 );
				Norm_h.z = Offset_h.z / ( VoxelBlockSize / 2 );
			}

			if( Norm.y > 0.00000001 )
			{
				Face_v = 5;
				Collide_Y = true;
				Delta_v.x = Norm.x / Norm.y;
				Delta_v.z = Norm.z / Norm.y;
				Delta_v.w = 1 / Norm.y;
				Collision_v.y = ( (float)Math.Floor( Pos.y / VoxelBlockSize ) + 1.0f ) * VoxelBlockSize;
				Collision_v.x = ( Collision_v.y - Pos.y ) * Delta_v.x + Pos.x;
				Collision_v.z = ( Collision_v.y - Pos.y ) * Delta_v.z + Pos.z;
				Collision_v.w = ( Collision_v.y - Pos.y ) * Delta_v.w;
				Offset_v.y = VoxelBlockSize;
				Offset_v.x = Delta_v.x * VoxelBlockSize;
				Offset_v.z = Delta_v.z * VoxelBlockSize;
				Offset_v.w = Delta_v.w * VoxelBlockSize;
				Norm_v.x = Offset_v.x / ( VoxelBlockSize / 2 );
				Norm_v.y = Offset_v.y / ( VoxelBlockSize / 2 );
				Norm_v.z = Offset_v.z / ( VoxelBlockSize / 2 );
			}
			else if( Norm.y < -0.00000001 )
			{
				Face_v = 4;
				Collide_Y = true;
				Delta_v.x = Norm.x / -Norm.y;
				Delta_v.z = Norm.z / -Norm.y;
				Delta_v.w = 1.0f / -Norm.y;
				Collision_v.y = ( (float)Math.Floor( Pos.y / VoxelBlockSize ) ) * VoxelBlockSize; // - 1.0
				Collision_v.x = ( Pos.y - Collision_v.y ) * Delta_v.x + Pos.x;
				Collision_v.z = ( Pos.y - Collision_v.y ) * Delta_v.z + Pos.z;
				Collision_v.w = Math.Abs( ( Collision_v.y - Pos.y ) * Delta_v.w );

				Offset_v.y = -VoxelBlockSize;
				Offset_v.x = Delta_v.x * VoxelBlockSize;
				Offset_v.z = Delta_v.z * VoxelBlockSize;
				Offset_v.w = Delta_v.w * VoxelBlockSize;
				Norm_v.x = Offset_v.x / ( VoxelBlockSize / 2 );
				Norm_v.y = Offset_v.y / ( VoxelBlockSize / 2 );
				Norm_v.z = Offset_v.z / ( VoxelBlockSize / 2 );
			}

			if( Norm.z > 0.00000001 )
			{
				Face_s = 0;
				Collide_Z = true;
				Delta_s.x = Norm.x / Norm.z;
				Delta_s.y = Norm.y / Norm.z;
				Delta_s.w = 1.0f / Norm.z;
				Collision_s.z = ( (float)Math.Floor( Pos.z / VoxelBlockSize + 1.0f ) ) * VoxelBlockSize;
				Collision_s.x = ( Collision_s.z - Pos.z ) * Delta_s.x + Pos.x;
				Collision_s.y = ( Collision_s.z - Pos.z ) * Delta_s.y + Pos.y;
				Collision_s.w = ( Collision_s.z - Pos.z ) * Delta_s.w;

				Offset_s.z = VoxelBlockSize;
				Offset_s.x = Delta_s.x * VoxelBlockSize;
				Offset_s.y = Delta_s.y * VoxelBlockSize;
				Offset_s.w = Delta_s.w * VoxelBlockSize;
				Norm_s.x = Offset_s.x / ( VoxelBlockSize / 2 );
				Norm_s.y = Offset_s.y / ( VoxelBlockSize / 2 );
				Norm_s.z = Offset_s.z / ( VoxelBlockSize / 2 );
			}
			else if( Norm.z < -0.00000001 )
			{
				Face_s = 1;
				Collide_Z = true;
				Delta_s.x = Norm.x / -Norm.z;
				Delta_s.y = Norm.y / -Norm.z;
				Delta_s.w = 1.0f / -Norm.z;
				Collision_s.z = ( (float)Math.Floor( Pos.z / VoxelBlockSize ) ) * VoxelBlockSize; // - 1.0
				Collision_s.x = ( Pos.z - Collision_s.z ) * Delta_s.x + Pos.x;
				Collision_s.y = ( Pos.z - Collision_s.z ) * Delta_s.y + Pos.y;
				Collision_s.w = ( Pos.z - Collision_s.z ) * Delta_s.w;
				Offset_s.z = -VoxelBlockSize;
				Offset_s.x = Delta_s.x * VoxelBlockSize;
				Offset_s.y = Delta_s.y * VoxelBlockSize;
				Offset_s.w = Delta_s.w * VoxelBlockSize;
				Norm_s.x = Offset_s.x / ( VoxelBlockSize / 2 );
				Norm_s.y = Offset_s.y / ( VoxelBlockSize / 2 );
				Norm_s.z = Offset_s.z / ( VoxelBlockSize / 2 );
			}
			/*
			   printf ("Loc(%lf %lf %lf) Norm(%lf %lf %lf) Col(%lf %lf %lf %lf) Off(%lf %lf %lf %lf) C(%d,%d,%d)\n", Pos.x, Pos.y, Pos.z, Norm.x,Norm.y, Norm.z, Collision_s.x, Collision_s.y, Collision_s.z, Collision_s.w, Offset_s.x,Offset_s.y, Offset_s.z,Offset_s.w
				   ,(ushort)((Collide_X==true)? 1:0) ,(ushort)((Collide_Y==true)? 1:0), (ushort)((Collide_Z==true)? 1:0));
			*/
			//  printf("yaw: %04lf pitch: %lf Offset_y:%lf Offset_z:%lf xyz:%lf %lf %lf NXYZ:%lf %lf %lf Dxyz:%lf %lf %lf", yaw,pitch, Delta_h.y, Delta_h.z,x,y,z, Norm_h.x, Norm_h.y, Norm_h.z, Delta_h.x, Delta_h.y, Delta_h.z);
			//printf("Angle (y:%lf p:%lf) XYZ:(%lf %lf %lf) Off(%lf %lf %lf %lf) Coll(%lf %lf %lf %lf) Norm(%lg %lg %lf) :\n", yaw,pitch,x,y,z, Offset_s.x, Offset_s.y, Offset_s.z, Offset_s.w, Collision_s.x, Collision_s.y, Collision_s.z, Collision_s.w, Norm_s.x,Norm_s.y, Norm_s.z);

			int Match_h = 0;
			int Match_s = 0;
			int Match_v = 0;
			int Cycle = 1;
			float MinW = 10000000.0f;

			for( i = 0; i < In.MaxCubeIterations; i++ )
			{
				// if  ( (Collision_h.w > In.MaxDetectionDistance) || (Collision_s.w > In.MaxDetectionDistance) || (Collision_v.w > In.MaxDetectionDistance)) { Out.Collided = false; return(false); }
				// Horizontal x axis.
				if( Collide_X )
				{
					if( Match_h == 0 && Collision_h.w < MinW )
					{
						ActualCube_x = (int)Math.Floor( ( Collision_h.x - Norm_h.x ) / VoxelBlockSize ); ActualCube_y = (int)Math.Floor( ( Collision_h.y - Norm_h.y ) / VoxelBlockSize ); ActualCube_z = (int)Math.Floor( ( Collision_h.z - Norm_h.z ) / VoxelBlockSize );
						NewCube_x = (int)Math.Floor( ( Collision_h.x ) / VoxelBlockSize ); NewCube_y = (int)Math.Floor( ( Collision_h.y ) / VoxelBlockSize ); NewCube_z = (int)Math.Floor( ( Collision_h.z ) / VoxelBlockSize );
						//ActualCube_x = (int)Math.Floor((Collision_h.x - Norm_h.x) / VoxelBlockSize); ActualCube_y = (int)Math.Floor((Collision_h.y - Norm_h.y) / VoxelBlockSize); ActualCube_z = (int)Math.Floor((Collision_h.z - Norm_h.z) / VoxelBlockSize);
						//NewCube_x = (int)Math.Floor((Collision_h.x + Norm_h.x) / VoxelBlockSize); NewCube_y = (int)Math.Floor((Collision_h.y + Norm_h.y) / VoxelBlockSize); NewCube_z = (int)Math.Floor((Collision_h.z + Norm_h.z) / VoxelBlockSize);
						if( Face_h == 3 ) NewCube_x--;
						VoxelType = GetVoxel( NewCube_x, NewCube_y, NewCube_z );
						if( !VoxelTypeManager.VoxelTable[VoxelType].properties.Is_PlayerCanPassThrough ^ InvertCollision )
						{
							if( Armed )
							{
								Out.PredPointedVoxel.X = ActualCube_x; Out.PredPointedVoxel.Y = ActualCube_y; Out.PredPointedVoxel.Z = ActualCube_z;
								Out.PointedVoxel.X = NewCube_x; Out.PointedVoxel.Y = NewCube_y; Out.PointedVoxel.Z = NewCube_z;
								Out.CollisionPoint.x = Collision_h.x; Out.CollisionPoint.y = Collision_h.y; Out.CollisionPoint.z = Collision_h.z; Out.CollisionDistance = Collision_h.w;
								Out.CollisionAxe = 0; Out.CollisionFace = Face_h;
								Out.PointInCubeFace.X = ( Out.CollisionPoint.z % VoxelBlockSize ); Out.PointInCubeFace.Y = ( Out.CollisionPoint.y % VoxelBlockSize );
								//printf(" MATCH_H: %lf (%ld %ld %ld) C:%ld\n",Collision_h.w, NewCube_x, NewCube_y, NewCube_z, Cycle);
								Match_h = Cycle;
								MinW = Collision_h.w;
							}
							else DeferArmed = true;
						}
					}
				}

				// Horizontal z axis.

				if( Collide_Z )
				{
					if( Match_s == 0 && Collision_s.w < MinW )
					{
						ActualCube_x = (int)Math.Floor( ( Collision_s.x - Norm_s.x ) / VoxelBlockSize ); ActualCube_y = (int)Math.Floor( ( Collision_s.y - Norm_s.y ) / VoxelBlockSize ); ActualCube_z = (int)Math.Floor( ( Collision_s.z - Norm_s.z ) / VoxelBlockSize );
						NewCube_x = (int)Math.Floor( ( Collision_s.x ) / VoxelBlockSize ); NewCube_y = (int)Math.Floor( ( Collision_s.y ) / VoxelBlockSize ); NewCube_z = (int)Math.Floor( ( Collision_s.z ) / VoxelBlockSize );
						//ActualCube_x = (int)Math.Floor((Collision_s.x - Norm_s.x) / VoxelBlockSize); ActualCube_y = (int)Math.Floor((Collision_s.y - Norm_s.y) / VoxelBlockSize); ActualCube_z = (int)Math.Floor((Collision_s.z - Norm_s.z) / VoxelBlockSize);
						//NewCube_x = (int)Math.Floor((Collision_s.x + Norm_s.x) / VoxelBlockSize); NewCube_y = (int)Math.Floor((Collision_s.y + Norm_s.y) / VoxelBlockSize); NewCube_z = (int)Math.Floor((Collision_s.z + Norm_s.z) / VoxelBlockSize);
						if( Face_s == 1 ) NewCube_z--;
						VoxelType = GetVoxel( NewCube_x, NewCube_y, NewCube_z );
						if( !VoxelTypeManager.VoxelTable[VoxelType].properties.Is_PlayerCanPassThrough ^ InvertCollision )
						{
							if( Armed )
							{
								Out.PredPointedVoxel.X = ActualCube_x; Out.PredPointedVoxel.Y = ActualCube_y; Out.PredPointedVoxel.Z = ActualCube_z;
								Out.PointedVoxel.X = NewCube_x; Out.PointedVoxel.Y = NewCube_y; Out.PointedVoxel.Z = NewCube_z;
								Out.CollisionPoint.x = Collision_s.x; Out.CollisionPoint.y = Collision_s.y; Out.CollisionPoint.z = Collision_s.z; Out.CollisionDistance = Collision_s.w;
								Out.CollisionAxe = 2; Out.CollisionFace = Face_s;
								Out.PointInCubeFace.X = ( Out.CollisionPoint.x % VoxelBlockSize ); Out.PointInCubeFace.Y = ( Out.CollisionPoint.y % VoxelBlockSize );

								//printf(" MATCH_S: %lf (%ld %ld %ld) C:%ld\n",Collision_s.w, NewCube_x, NewCube_y, NewCube_z, Cycle);
								Match_s = Cycle;
								MinW = Collision_s.w;
							}
							else DeferArmed = true;
						}
					}
				}

				// Vertical y axis.

				if( Collide_Y )
				{
					if( Match_v == 0 && Collision_v.w < MinW )
					{
						ActualCube_x = (int)Math.Floor( ( Collision_v.x - Norm_v.x ) / VoxelBlockSize ); ActualCube_y = (int)Math.Floor( ( Collision_v.y - Norm_v.y ) / VoxelBlockSize ); ActualCube_z = (int)Math.Floor( ( Collision_v.z - Norm_v.z ) / VoxelBlockSize );
						NewCube_x = (int)Math.Floor( ( Collision_v.x ) / VoxelBlockSize ); NewCube_y = (int)Math.Floor( ( Collision_v.y ) / VoxelBlockSize ); NewCube_z = (int)Math.Floor( ( Collision_v.z ) / VoxelBlockSize );
						//ActualCube_x = (int)Math.Floor((Collision_v.x - Norm_v.x) / VoxelBlockSize);   ActualCube_y = (int)Math.Floor((Collision_v.y - Norm_v.y) / VoxelBlockSize);   ActualCube_z = (int)Math.Floor((Collision_v.z - Norm_v.z) / VoxelBlockSize);
						//NewCube_x = (int)Math.Floor((Collision_v.x + Norm_v.x) / VoxelBlockSize); NewCube_y = (int)Math.Floor((Collision_v.y + Norm_v.y) / VoxelBlockSize); NewCube_z = (int)Math.Floor((Collision_v.z + Norm_v.z) / VoxelBlockSize);
						if( Face_v == 4 ) NewCube_y--;
						VoxelType = GetVoxel( NewCube_x, NewCube_y, NewCube_z );
						if( !VoxelTypeManager.VoxelTable[VoxelType].properties.Is_PlayerCanPassThrough ^ InvertCollision )
						{
							if( Armed )
							{
								Out.PredPointedVoxel.X = ActualCube_x; Out.PredPointedVoxel.Y = ActualCube_y; Out.PredPointedVoxel.Z = ActualCube_z;
								Out.PointedVoxel.X = NewCube_x; Out.PointedVoxel.Y = NewCube_y; Out.PointedVoxel.Z = NewCube_z;
								Out.CollisionPoint.x = Collision_v.x; Out.CollisionPoint.y = Collision_v.y; Out.CollisionPoint.z = Collision_v.z; Out.CollisionDistance = Collision_v.w;
								Out.CollisionAxe = 1; Out.CollisionFace = Face_v;
								Out.PointInCubeFace.X = ( Out.CollisionPoint.x % VoxelBlockSize ); Out.PointInCubeFace.Y = ( Out.CollisionPoint.z % VoxelBlockSize );

								//printf(" MATCH_V: %lf (%ld %ld %ld) C:%ld\n",Collision_v.w, NewCube_x, NewCube_y, NewCube_z,Cycle );
								Match_v = Cycle;
								MinW = Collision_v.w;
							}
							else DeferArmed = true;
						}
					}
				}

				//printf(" Match (H:%lf S:%lf V:%lf) \n", Collision_h.w, Collision_s.w, Collision_v.w);
				if( Match_h > 0 && ( Cycle - Match_h ) > In.PlaneCubeDiff ) { Out.Collided = true; return ( true ); }
				if( Match_s > 0 && ( Cycle - Match_s ) > In.PlaneCubeDiff ) { Out.Collided = true; return ( true ); }
				if( Match_v > 0 && ( Cycle - Match_v ) > In.PlaneCubeDiff ) { Out.Collided = true; return ( true ); }

				if( Collide_X )
					Collision_h.x += Offset_h.x; Collision_h.y += Offset_h.y; Collision_h.z += Offset_h.z; Collision_h.w += Offset_h.w;
				if( Collide_Y )
					Collision_v.x += Offset_v.x; Collision_v.y += Offset_v.y; Collision_v.z += Offset_v.z; Collision_v.w += Offset_v.w;
				if( Collide_Z )
					Collision_s.x += Offset_s.x; Collision_s.y += Offset_s.y; Collision_s.z += Offset_s.z; Collision_s.w += Offset_s.w;
				Cycle++;
				if( DeferArmed ) Armed = true;
			}
			Out.Collided = false;
			Out.CollisionAxe = 0;
			Out.CollisionFace = 0;
			Out.CollisionDistance = 0.0f;
			Out.CollisionPoint = btVector3.Zero;
			return ( false );
			//printf("first_h_x : %lf first_h_y %lf\n",first_h_x,first_h_y);
		}

	}
}
