#define USE_EXTERNAL_COMPILER
using System;
using System.CodeDom.Compiler;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Reflection.Emit;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Voxels
{
	internal class VoxelTypeManager
	{
		internal VoxelType[] VoxelTable;
		VoxelGameEnvironment GameEnv;

		int LoadedTexturesCount;

		public FastBit_Array_64k ActiveTable;


		internal int GetTexturesCount() { return ( LoadedTexturesCount ); }

		internal VoxelTypeManager()
		{
			int i;
			GameEnv = null;
			LoadedTexturesCount = 0;
			ActiveTable = new FastBit_Array_64k();
			ActiveTable.Clear();
			VoxelTable = new VoxelType[65536];
		}

		~VoxelTypeManager()
		{
			int i;
			for( i = 0; i < 65536; i++ )
			{
				VoxelTable[i] = null;
			}
			ActiveTable = null;
			VoxelTable = null;
		}


		void AddVoxelType( int TypeNum, VoxelType VoxelType )
		{
			VoxelTable[TypeNum] = VoxelType;
			VoxelType.VoxelTypeManager = this;
			ActiveTable.Set( TypeNum, VoxelType.properties.Is_Active );
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

		internal bool LoadVoxelTypes()
		{
			Log.log( "Here we start to do magic to get voxel types..." );
			int i;
			VoxelType VoxelType;

			i = 0;
			VoxelType = new VoxelType_Void();
			VoxelType.SetProperties( new VoxelProperties( 0 ) );
			AddVoxelType( i++, VoxelType );

			for( ; i < 100; i++ )
			{
				VoxelProperties props = VoxelProperties.Load( i );
				if( Compiler.LoadVoxelCode( props.Type ) )
					VoxelType = Compiler.LoadExtendedVoxelType( props.Type );

				if( VoxelType == null )
					VoxelType = new VoxelType();
				VoxelType.SetProperties( props );

				VoxelType.SetGameEnv( GameEnv );
				VoxelType.SetManager( this );
				if( VoxelType.LoadTexture() )
				{
					AddVoxelType( i, VoxelType );
					//VoxelType.LoadVoxelInformations();
					ActiveTable.Set( i, VoxelType.properties.Is_Active );
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
