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
#define USE_EXTERNAL_COMPILER
using System;
using System.CodeDom.Compiler;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Reflection.Emit;
using System.Text;
using Voxelarium.Common;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Voxels
{
	public class VoxelTypeManager
	{
		internal VoxelType[] VoxelTable;
		VoxelGameEnvironment GameEnv;

		int LoadedTexturesCount;

		//internal FastBit_Array_64k ActiveTable;

		internal VoxelType this[int n]
		{
			get
			{
				return VoxelTable[n];
			}
		}

		internal int GetTexturesCount() { return ( LoadedTexturesCount ); }

		internal VoxelTypeManager()
		{
			GameEnv = null;
			LoadedTexturesCount = 0;
			VoxelTable = new VoxelType[65536];
		}

		~VoxelTypeManager()
		{
			int i;
			for( i = 0; i < 65536; i++ )
			{
				VoxelTable[i] = null;
			}
			VoxelTable = null;
		}


		void AddVoxelType( int TypeNum, VoxelType VoxelType )
		{
			VoxelTable[TypeNum] = VoxelType;
			VoxelType.VoxelTypeManager = this;
			//ActiveTable.Set( TypeNum, VoxelType.properties.Is_Active );
		}

		internal void SetGameEnv( VoxelGameEnvironment GameEnv )
		{
			this.GameEnv = GameEnv;
		}



		internal void FillZeroSlots( ushort VoxelTypeUsedToFill )
		{
			int i;

			// Mark this voxeltype as a null voxeltype

			VoxelTable[VoxelTypeUsedToFill].properties.Is_NoType = true;

			// Fill the empty slots to point to this voxeltype;
			for( i = 0; i < 65536; i++ )
			{
				if( VoxelTable[i] == null )
					VoxelTable[i] = VoxelTable[VoxelTypeUsedToFill];
			}
		}

		internal VoxelType GetVoxelType( ushort TypeNum )
		{
			return ( VoxelTable[TypeNum] );
		}

		internal void LoadTexturesToAtlas( TextureAtlas atlas, ref int percent, ref int step, ref int steps )
		{
			steps += VoxelTable.Length;
			foreach( VoxelType vt in VoxelTable )
			{
				vt.LoadTextureToAtlas( atlas );
				percent = ( ++step * 100 ) / steps;
			}
		}

		internal bool LoadVoxelTypes( bool nogui )
		{
			Log.log( "Here we start to do magic to get voxel types..." );
			ushort i;
			VoxelType VoxelType;

			i = 0;
			VoxelType = new VoxelType_Void();
			VoxelType.SetProperties( new VoxelProperties( 0 ) );
			AddVoxelType( i++, VoxelType );

			for( ; i < 300; i++ )
			{
				VoxelProperties props = VoxelProperties.Load( i );
				if( Compiler.LoadVoxelCode( props, props.Type ) )
				{
					VoxelType = Compiler.LoadExtendedVoxelType( props, props.Type );
					VoxelType.ExtensionType = Compiler.LoadExtendedVoxelExtension( props.Type );
					if( VoxelType.ExtensionType != null )
						props.Is_HasAllocatedMemoryExtension = true;
				}
				else
					VoxelType = null;
				if( VoxelType == null )
					VoxelType = new VoxelType();

				VoxelType.SetProperties( props );

				VoxelType.SetGameEnv( GameEnv );
				VoxelType.SetManager( this );
				bool success = false;
				if( ( ( VoxelType.properties.DrawInfo & (byte)VoxelGlobalSettings.ZVOXEL_DRAWINFO_SHADER ) != 0 ) )
				{
					if( ( ( VoxelType.properties.DrawInfo & (byte)VoxelGlobalSettings.ZVOXEL_DRAWINFO_DECAL ) != 0 ) )
						success = VoxelType.LoadTexture();
					else
						success = true;
				}
				else if( !nogui )
				{
					success = VoxelType.LoadTexture();
					if( !success )
						Log.log( "Fatal error : Missing texture for " + i );
				}
				if( success )
				{
					AddVoxelType( i, VoxelType );
					LoadedTexturesCount++;
				}
			}
			return true;
		}

		VoxelExtension CreateVoxelExtension( int VoxelType )
		{
			VoxelType VoxelTypeEntry;

			VoxelTypeEntry = VoxelTable[VoxelType]; if( VoxelTypeEntry ==null ) return ( null );
			return ( VoxelTypeEntry.CreateVoxelExtension() );
		}

		internal void DeleteVoxelExtension( ushort VoxelType, VoxelExtension VoxelExtension )
		{
			VoxelType VoxelTypeEntry;

			VoxelTypeEntry = VoxelTable[VoxelType]; if( VoxelTypeEntry == null ) return;
			if( VoxelTypeEntry.properties.ExtensionType == 0 ) return;
			VoxelTypeEntry.DeleteVoxelExtension( VoxelExtension );
		}

		//void DumpInfos();

		//void OutFabInfos();

		//bool _Internal_CompareTransTables( ZString & Message, ZFabInfos* Fab, ZFabInfos::ZTransformation* Tr1, ZFabInfos::ZTransformation* Tr2 );

		//void FindFabConflics();

	}
}
