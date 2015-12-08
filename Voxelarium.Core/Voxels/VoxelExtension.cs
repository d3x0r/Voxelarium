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
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public interface IVoxelExtension
	{
		bool IsSleeping();
		ushort Tempurature { get; set; }
	}
	public abstract class VoxelExtension : IDisposable
	{
		public ExtensionTypes ExtensionType;
		public ushort tempurature;
		public bool sleeping;
		public enum ExtensionTypes
		{
			Extension_None = 0
			, Extension_Storage = 1
			, Extension_UserTextureTransformer = 2
			, Extension_PlaneZ1 = 3
			, Extension_TransformationMachine = 4
			, Extension_Programmable = 5
			, Extension_FusionElement = 6
			, Extension_BlastFurnace = 7
			, Extension_MiningRobot_xr1 = 8
			  , Extension_Sequencer = 9
			  , Extension_Egmy_T1 = 10
			  , Extension_FertileGround
			  , Extension_Food
			  , Extension_Animal
			  , Extension_Aroma
			  , Extension_AromaGenerator
			  , Extension_Count
		};

		//public static ULong[] ExtensionCharCodes;

		//protected:
		//	bool _ThrowExtension( ZStream_SpecialRamStream* Stream, ZMemSize ExtensionSize );

		public virtual uint GetExtensionID() { return ( VoxelUtils.MulticharConst( 'N', 'S', 'P', 'C' ) ); }
		public virtual bool Save( BinaryWriter Stream ) { return true; }
		public virtual bool Load( BinaryReader Stream ) { return true; }
		//public virtual void SetGameEnv( VoxelGameEnvironment GameEnv ) { }
		public virtual VoxelExtension GetNewCopy() { return ( null ); }
		public void Dispose() { }

		bool _ThrowExtension( Stream Stream, uint ExtensionSize )
		{
			ExtensionSize -= 2;
			try
			{
				for( uint i = 0; i < ExtensionSize; i++ ) Stream.ReadByte();
			}
			catch( Exception e )
			{
				return false;
			}

			return true;
		}

		public static uint[] ExtensionCharCodes = {
			 0
			, VoxelUtils.MulticharConst( 'S', 'T', 'O', 'R' )
			, VoxelUtils.MulticharConst( 'U', 'T', 'T', 'R' )
			, VoxelUtils.MulticharConst( 'P', 'L', 'Z', '1' )
			, VoxelUtils.MulticharConst( 'F', 'M', 'C', 'H' )
			, VoxelUtils.MulticharConst( 'P', 'R', 'O', 'G' )
			, VoxelUtils.MulticharConst( 'F', 'U', 'S', 'E' )
			, VoxelUtils.MulticharConst( 'B', 'F', 'U', 'R' ) // 7 blst furnace
			, VoxelUtils.MulticharConst( 'M', 'R', 'X', '1' )  //8
			, VoxelUtils.MulticharConst( 'S', 'E', 'Q', 'U' )
			, VoxelUtils.MulticharConst( 'E', 'M', 'Y', '1' ) // 10 egmy
			, VoxelUtils.MulticharConst( 'B', 'F', 'G', 'R' ) // fertile ground
			, VoxelUtils.MulticharConst( 'F', 'O', 'O', 'D' )
			, VoxelUtils.MulticharConst( 'B', 'A', 'N', 'I' )  // animal
			, VoxelUtils.MulticharConst( 'A', 'R', 'M', 'A' )  // aroma
			, VoxelUtils.MulticharConst( 'A', 'R', 'M', 'G' )   // aroma generator
		//, // count unused
	};

	}
}
