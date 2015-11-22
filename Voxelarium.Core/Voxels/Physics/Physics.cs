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
using BEPUutilities;
using BEPUutilities.Threading;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Voxels;

namespace Voxelarium.Core.Voxels.Physics
{
	internal enum VoxelShape : byte
	{
		Empty, Solid
	}

	public class PhysicsEngine
	{
		Space world;
		public PhysicsEngine()
		{
			ParallelLooper parallelLooper = new ParallelLooper();
			for( int i = 0; i < ( Environment.ProcessorCount - 3 ); i++ )
			{
				parallelLooper.AddThread();
			}

			world = new Space();
			world.ForceUpdater.Gravity = new Vector3( 0, -9.81f, 0 );

		}

		internal void Add( Sector sector )
		{
			world.Add( sector.grid );
		}

		public class Sector
		{
			VoxelShape[] content;
			VoxelGridShape shape;
			internal VoxelGrid grid;
			internal Sector( VoxelWorld world, int x, int y, int z )
			{

				shape = new VoxelGridShape( content, world.VoxelBlockSize );
				grid = new VoxelGrid( shape, new Vector3( -x * VoxelSector.ZVOXELBLOCSIZE_X * world.VoxelBlockSize * 0.5f
													, -y * VoxelSector.ZVOXELBLOCSIZE_X * world.VoxelBlockSize * 0.5f
													, -z * VoxelSector.ZVOXELBLOCSIZE_X * world.VoxelBlockSize * 0.5f ) );
				
			}

			internal void SetVoxel( int x, int y, int z )
			{
				int offset = y 
					+ ( x * (int)VoxelSector.ZVOXELBLOCSIZE_Y ) 
					+ ( z * (int)VoxelSector.ZVOXELBLOCSIZE_Y * (int)VoxelSector.ZVOXELBLOCSIZE_X );
				//shape.Shapes
			}
			internal void SetVoxel( int offset )
			{
				//shape.Shapes
			}
			internal void ClearVoxel( int x, int y, int z )
			{
				int offset = y
					+ ( x * (int)VoxelSector.ZVOXELBLOCSIZE_Y )
					+ ( z * (int)VoxelSector.ZVOXELBLOCSIZE_Y * (int)VoxelSector.ZVOXELBLOCSIZE_X );
				//shape.Shapes
			}
			internal void ClearVoxel( int offset )
			{
				//shape.Shapes
			}
		}


	}
}
