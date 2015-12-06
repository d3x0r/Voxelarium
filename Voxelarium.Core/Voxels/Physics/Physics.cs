/*
 * This file is part of Voxelarium.
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

using BEPUphysics;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUutilities.Threading;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels;

namespace Voxelarium.Core.Voxels.Physics
{
	internal enum VoxelShape : byte
	{
		Empty, Solid
	}


	public class PhysicsEngine : IDisposable
	{
		internal Space world;
		internal Entity[] test_entitites = new Entity[10];
		ParallelLooper parallelLooper = new ParallelLooper();
		public PhysicsEngine()
		{
			Display.AtExit += Display_AtExit;
			for( int i = 0; i < ( Environment.ProcessorCount - 3 ); i++ )
			{
				parallelLooper.AddThread();
			}
			VoxelGridConvexPairHandler.EnsurePairsAreRegistered();
			world = new Space();
			world.ForceUpdater.Gravity = new Vector3( 0, -9.81f, 0 );

			{
				int n;
				for( n = 0; n < 10; n++ )
				{
					Vector3 origin = new Vector3( 1800, n * 8, 0 );
					test_entitites[n] = new Box( origin, 4, 4, 4, 1 );
					world.Add( test_entitites[n] );
				}
			}

		}

		private void Display_AtExit()
		{
			while( parallelLooper.ThreadCount > 0 )
				parallelLooper.RemoveThread();
		}

		public void Dispose()
		{
			foreach( Sector sector in active_sectors )
				world.Remove( sector.grid );
			active_sectors = null;
			world = null;
		}

		internal List<Sector> active_sectors = new List<Sector>();

		internal void Add( Sector sector )
		{
			sector.PhysicsSpace = world;
			lock ( active_sectors )
			{
				active_sectors.Add( sector );
			}
			world.Add( sector.grid );
		}

		internal void Remove( Sector sector )
		{
			sector.PhysicsSpace = null;
			lock ( active_sectors )
			{
				active_sectors.Remove( sector );
			}
			world.Remove( sector.grid );
		}

		internal void Step( float delta )
		{
			world.Update( delta );
		}

		public class Sector
		{
			internal VoxelShape[] content;
			VoxelGridShape shape;
			internal VoxelGrid grid;
			internal VoxelSector sector;

			internal Space PhysicsSpace;

			internal bool Empty { get { return shape.Empty; } set {
					if( value == true && !shape.Empty )
					{
						int len = content.Length;
						for( int i = 0; i < len; i++ )
							content[i] = VoxelShape.Empty;
					}
					shape.Empty = value; } }

			internal Sector( PhysicsEngine pe, VoxelWorld world, VoxelSector sector )
			{
				this.sector = sector;
				content = new VoxelShape[sector.Size_x * sector.Size_y * sector.Size_z];
				shape = new VoxelGridShape( content, world.VoxelBlockSize );
				grid = new VoxelGrid( shape, new Vector3( sector.Pos_x * sector.Size_x * world.VoxelBlockSize
													, sector.Pos_y * sector.Size_y * world.VoxelBlockSize
													, sector.Pos_z * sector.Size_z * world.VoxelBlockSize ) );
				PhysicsSpace = pe.world;
			}

			/// <summary>
			/// Set a voxel as solid cube
			/// </summary>
			/// <param name="offset"></param>
			internal void SetVoxel( uint offset )
			{
				shape.Empty = false;
				//shape.Shapes
				//Log.log( "Set cell solid : " + offset + " at " + grid.Position.X + " " + grid.Position.Y + " " + grid.Position.Z  );
				shape.Cells[offset] = VoxelShape.Solid;
			}

			/// <summary>
			/// set voxel as an empty cube
			/// </summary>
			/// <param name="offset"></param>
			internal void ClearVoxel( uint offset )
			{
				//shape.Shapes
				shape.Cells[offset] = VoxelShape.Empty;
			}

			internal void SetPos( int x, int y, int z )
			{
				grid.Position = new Vector3( x, y, z );
				grid.UpdateBoundingBox();
			}


			internal void UpdateBoundsSlow()
			{
				int xmin = int.MaxValue, xmax = int.MinValue;
				int ymin = int.MaxValue, ymax = int.MinValue;
				int zmin = int.MaxValue, zmax = int.MinValue;
				for( int x = 0; x < sector.Size_x; x++ )
					for( int y = 0; y < sector.Size_y; y++ )
						for( int z = 0; y < sector.Size_z; z++ )
						{
							int ofs = (int)(x * sector.Size_y + y + z * sector.Size_y * sector.Size_x);
							if( content[ofs] != VoxelShape.Empty )
							{
								if( x < xmin ) xmin = x;
								if( x > xmax ) xmax = x;
								if( y < ymin ) ymin = y;
								if( y > ymax ) ymax = y;
								if( z < zmin ) zmin = z;
								if( z > zmax ) zmax = z;
							}
						}
				grid.UpdateBoundingBox();
			}
		}
	}
}
