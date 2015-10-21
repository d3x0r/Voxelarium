using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public class VoxelType
	{
		public struct VoxelLocation
		{
			VoxelSector Sector;
			ushort Offset;
		};

		protected VoxelGameEnvironment GameEnv;
		public VoxelTypeManager VoxelTypeManager;
		public string VoxelTypeName;
		//byte CanPassThrough;
		// Obsoleted
		public bool Draw_TransparentRendering;
		public bool Draw_FullVoxelOpacity;
		// New
		public byte DrawInfo;
		//
		public ushort Type;
		//public ZBitmapImage* MainTexture;
		public int[] OpenGl_TextureRef = new int[6];

		public bool Is_NoType; // Defined if this is a default "No type" Entry;
		public bool Is_UserTypeTransformable;     // Can be used to make user blocks. Very rare blocs must avoid that.
		public bool Is_PlayerCanPassThrough;      // The player can pass through it without colision.
		public bool Is_CanBeReplacedBy_Water;     // Does water can flow and replace it.
		public bool Is_CanBeReplacedBy_GreenAcid; // Does green acid can flow and replace it.
		public bool Is_CanBeReplacedBy_MustardGaz;// The mustard gaz can flow and replace it.
		public bool Is_CanTriggerBomb;            // Vincinity of this voxel can trigger nearby bomb
		public bool Is_Pumpable_ByPump_T1;        // Can be pumped by pump T1.
		public bool Is_Pumpable_ByPump_T2;        // Can be pumped by pump T1.
		public bool Is_Loadable_ByLoader_L1;      // Can be loaded by loader L1.
		public bool BvProp_MoveableByTreadmill;   // Can be moved by threadmill.
		public bool BvProp_CanBePickedUpByRobot;  // User programmable robot can pick it up.
		public bool BvProp_AtomicFireResistant;   // Z Fire
		public bool BvProp_EgmyT1Resistant;
		public byte BvProp_XrRobotPickMinLevel;       // xr1 extraction robot can pick it up.
		public byte BvProp_PrRobotReplaceMinLevel;       // Pr  programmable robot level to destroy it.
		public byte BvProp_PrRobotPickMinLevel;          // Pr  programmable robot level to pick it up.
		public byte BvProp_PrRobotMoveMinLevel;          // Pr  programmable robot level to pick it up.

		public int BlastResistance;              // Blast resistance. 1 = air or no modification. 2 = Very soft solid block. -1..-X = Blast amplification.

		// Control and

		public bool Is_SpaceGripType;
		public bool Is_KeepControlOnJumping;

		//

		public bool Is_Harming;
		public bool Is_Active; // Active voxels trigger special functions and transformations.
		public bool Is_CombinableWith_GreenAcid;


		// Type d'extension.

		public uint ExtensionType;
		public bool Is_VoxelExtension;              // Does this voxeltype has specific extension to init.
		public bool Is_HasAllocatedMemoryExtension; // Has allocated memory extension

		// Interfaces logicielles

		public bool Is_Interface_StoreBlock;
		public bool Is_Interface_PushBlock;
		public bool Is_Interface_PullBlock;
		public bool Is_Interface_GetInfo;
		public bool Is_Interface_SetInfo;

		// Material Caracteristics

		public double MiningHardness;
		public uint MiningType;
		public double FrictionCoef;
		public double HarmingLifePointsPerSecond;
		public double LiquidDensity;

		public double Grip_Horizontal;
		public double Grip_Vertical;
		public bool Is_Liquid;
		public bool Is_Gaz;

		// Autres flags

		public bool Is_Rideable; // This voxel is a vehicle where player can board in.
		public bool Is_HasHelpingMessage;
		public string HelpingMessage;
		public bool BvProp_FastMoving; // Fast moving voxels can override the modification tracker system in some cases.
		public FabMachineInfo FabInfo;
		public uint Documentation_PageNum;

		~VoxelType()
		{
			//MainTexture = null;
			FabInfo = null;
		}

		public VoxelType( ushort VoxelType )
		{
			VoxelTypeManager = null;
			//MainTexture = null;
			Is_PlayerCanPassThrough = false;
			this.Type = VoxelType;
			Draw_TransparentRendering = false;
			Draw_FullVoxelOpacity = true;
			DrawInfo = VoxelGlobalSettings.ZVOXEL_DRAWINFO_DRAWFULLVOXELOPACITY;
			ExtensionType = 0;
			Is_VoxelExtension = false;
			Is_HasAllocatedMemoryExtension = false;
			MiningHardness = 1000.0;
			MiningType = 2;
			Is_NoType = false;
			Is_UserTypeTransformable = true;
			Is_Harming = false;
			FrictionCoef = 0.001;
			Grip_Horizontal = 0.9;
			Grip_Vertical = 0.0;
			Is_SpaceGripType = false;
			Is_KeepControlOnJumping = true;
			HarmingLifePointsPerSecond = 0.0;
			Is_Active = false;
			Is_CanBeReplacedBy_Water = false;
			Is_CanBeReplacedBy_GreenAcid = false;
			Is_CanBeReplacedBy_MustardGaz = false;
			Is_CombinableWith_GreenAcid = true;
			Is_CanTriggerBomb = false;
			Is_Liquid = false;
			Is_Gaz = false;
			Is_Interface_StoreBlock = false;
			Is_Interface_PushBlock = false;
			Is_Interface_PullBlock = false;
			Is_Interface_GetInfo = false;
			Is_Interface_SetInfo = false;
			Is_Pumpable_ByPump_T1 = false;
			Is_Pumpable_ByPump_T2 = false;
			Is_Loadable_ByLoader_L1 = true;
			BvProp_MoveableByTreadmill = true;
			BvProp_FastMoving = false;
			Is_Rideable = false;
			Is_HasHelpingMessage = false;
			BvProp_CanBePickedUpByRobot = true;
			BvProp_XrRobotPickMinLevel = 1;
			BvProp_PrRobotReplaceMinLevel = 0;
			BvProp_PrRobotPickMinLevel = 0;
			BvProp_PrRobotMoveMinLevel = 0;
			BvProp_AtomicFireResistant = false;
			BvProp_EgmyT1Resistant = false;
			LiquidDensity = 0.0;
			BlastResistance = 1;
			for( int i = 0; i < 6; i++ )
				OpenGl_TextureRef[i] = 0;
			GameEnv = null;
			FabInfo = null;
			Documentation_PageNum = 0;
		}
		public void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }
		public void SetManager( VoxelTypeManager VoxelTypeManager ) { this.VoxelTypeManager = VoxelTypeManager; }
		public virtual void SpecialRender( float x, float y, float z ) { }

		public virtual bool LoadTexture()
		{
#if FINISHED_GRAPHICS
			//ZBitmapImage* Image;
			int attempt;
			// Get the right folder path
			Image = new ZBitmapImage();
			for( attempt = 0; attempt < 2; attempt++ )
			{
				ZString FileSpec, FileName;
				if( VoxelType < 32768 )
				{
					FileName << "voxeltexture_" << (ULong)VoxelType << ( attempt == 1 ? ".bmp" : ".png" );
					FileSpec.AddToPath( COMPILEOPTION_DATAFILESPATH ).AddToPath( "VoxelTypes" ).AddToPath( FileName );
				}
				else
				{
					FileName << "voxeltexture_" << (ULong)( VoxelType - 32767 ) << ( attempt == 1 ? ".bmp" : ".png" );
					if( COMPILEOPTION_USEHOMEDIRSTORAGE ) { FileSpec = ZStream_File::Get_Directory_UserData(); FileSpec.AddToPath( COMPILEOPTION_SAVEFOLDERNAME ); }
					FileSpec.AddToPath( "UserTextures" );
					FileSpec.AddToPath( FileName );
				}

				//  if (VoxelType<32768) sprintf(Buffer, "VoxelTypes/voxeltexture_%u.bmp", VoxelType);
				//  else                 sprintf(Buffer, "UserTextures/voxeltexture_%u.bmp", VoxelType-32767);

				if( Image->LoadBMP( FileSpec.String ) ) { break; }
			}
			if( attempt == 2 ) { delete Image; return false; }

#if COMPILEOPTION_LOWRESTEXTURING
			if( Image->Width > 128 ) Image->ReduceSize();
#endif
			MainTexture = Image;
			if( Image->BytesPerPixel != 4 )
			{
				if( VoxelType < 32768 ) printf( "Warning : Image %d has no alpha channel, this can cause crashes.\n", VoxelType );
				else printf( "Warning : User defined image %d has no alpha channel, this can cause crashes.\n", VoxelType - 32767 );
			}
#endif
			return ( true );
		}

		public virtual bool LoadVoxelInformations()
		{
			//char Buffer[1024];
			string File, Line, Token;

			// Get the right folder path

			string FileSpec, FileName;
			if( Type < 32768 )
			{
				FileName = String.Format( "voxelinfo_" + (ulong)Type + ".txt" );
				//FileSpec.AddToPath( COMPILEOPTION_DATAFILESPATH ).AddToPath( "VoxelTypes" ).AddToPath( "voxelinfo" ).AddToPath( FileName );

			}
			else
			{
				//FileName << "voxelinfo_" << (ULong)( VoxelType - 32767 ) << ".txt";
				//if( COMPILEOPTION_USEHOMEDIRSTORAGE ) { FileSpec = ZStream_File::Get_Directory_UserData(); FileSpec.AddToPath( COMPILEOPTION_SAVEFOLDERNAME ); }
				//FileSpec.AddToPath( "UserTextures" ).AddToPath( "voxelinfo" ).AddToPath( FileName );
			}


#if asfsdfasdf
			if( !File.LoadFromFile( FileSpec.String ) ) return ( false );

			DrawInfo = 0;
			while( File.Split( 0x0a, Line ) )
			{
				Line.StripAll( 0x0d );
				if( Line.Split( '=', Token ) )
				{
					Token.StripAll( ' ' );
					// if (Token=="Draw_TransparentRendering") Draw_TransparentRendering = Line.GetULong()>0 ? true:false;
					// if (Token=="Draw_FullVoxelOpacity")     Draw_FullVoxelOpacity     = Line.GetULong()>0 ? true:false;

					if( Token == "Draw_TransparentRendering" )
					{
						if( Line.GetULong() > 0 ) { DrawInfo |= ZVOXEL_DRAWINFO_DRAWTRANSPARENTRENDERING; Draw_TransparentRendering = true; }
						else { DrawInfo &= 0xFF ^ ZVOXEL_DRAWINFO_DRAWTRANSPARENTRENDERING; Draw_TransparentRendering = false; }
					}
					if( Token == "Draw_FullVoxelOpacity" )
					{
						if( Line.GetULong() > 0 ) { DrawInfo |= ZVOXEL_DRAWINFO_DRAWFULLVOXELOPACITY; Draw_FullVoxelOpacity = true; }
						else { DrawInfo &= 0xFF ^ ZVOXEL_DRAWINFO_DRAWFULLVOXELOPACITY; Draw_FullVoxelOpacity = false; }
					}
					if( Token == "Draw_SpecialRender" ) { if( Line.GetULong() > 0 ) DrawInfo |= ZVOXEL_DRAWINFO_SPECIALRENDERING; else DrawInfo &= 0xFF ^ ZVOXEL_DRAWINFO_SPECIALRENDERING; }

					if( Token == "VoxelTypeName" ) VoxelTypeName = Line;

					if( Token == "ExtensionType" ) { ExtensionType = Line.GetULong(); }
					if( Token == "MiningHardness" ) { MiningHardness = (double)Line.GetULong(); }
					if( Token == "MiningType" ) { MiningType = Line.GetULong(); }
					if( Token == "BvProp_PlayerCanPassThrough" ) { Is_PlayerCanPassThrough = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_Harming" ) { Is_Harming = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_Active" ) { Is_Active = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "FrictionCoef" ) { FrictionCoef = Line.GetDouble(); if( FrictionCoef == 0.0 ) FrictionCoef = 1.0; }
					if( Token == "Grip_Horizontal" ) { Grip_Horizontal = Line.GetDouble(); if( Grip_Horizontal == 0.0 ) Grip_Horizontal = 1.0; }
					if( Token == "Grip_Vertical" ) { Grip_Vertical = Line.GetDouble(); if( Grip_Vertical == 0.0 ) Grip_Vertical = 1.0; }
					if( Token == "BvProp_KeepControlOnJumping" ) { Is_KeepControlOnJumping = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "HarmingLifePointsPerSecond" ) { HarmingLifePointsPerSecond = Line.GetDouble(); }
					if( Token == "BvProp_CombinableWith_GreenAcid" ) { Is_CombinableWith_GreenAcid = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_CanBeReplacedBy_MustardGaz" ) { Is_CanBeReplacedBy_MustardGaz = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_CanTriggerBomb" ) { Is_CanTriggerBomb = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BlastResistance" ) { BlastResistance = ( Line.GetLong() ); }
					if( Token == "BvProp_Liquid" ) { Is_Liquid = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "LiquidDensity" ) { LiquidDensity = Line.GetDouble(); }
					if( Token == "BvProp_Gaz" ) { Is_Gaz = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_Pumpable_ByPump_T1" ) { Is_Pumpable_ByPump_T1 = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_Pumpable_ByPump_T2" ) { Is_Pumpable_ByPump_T2 = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_Loadable_ByLoader_L1" ) { Is_Loadable_ByLoader_L1 = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_FastMoving" ) { BvProp_FastMoving = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_CanBePickedUpByRobot" ) { BvProp_CanBePickedUpByRobot = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_XrRobotPickMinLevel" ) { BvProp_XrRobotPickMinLevel = (UByte)Line.GetULong(); }
					if( Token == "Documentation_PageNum" ) { Documentation_PageNum = Line.GetULong(); }
					if( Token == "BvProp_AtomicFireResistant" ) { BvProp_AtomicFireResistant = ( Line.GetULong() != 0 ) ? true : false; }
					if( Token == "BvProp_EgmyT1Resistant" ) { BvProp_EgmyT1Resistant = ( Line.GetULong() != 0 ) ? true : false; }


					// if (Token == "Is_Interface_StoreBlock") {Is_Interface_StoreBlock = (Line.GetULong()!=0) ? true:false; }

				}
			}
#endif
			return ( true );
		}

		// Voxel Extensions;
		public virtual VoxelExtension CreateVoxelExtension( bool IsLoadingPhase = false ) { return null; }
		public virtual void DeleteVoxelExtension( VoxelExtension VoxelExtension, bool IsUnloadingPhase = false )
		{
			if( Is_HasAllocatedMemoryExtension && VoxelExtension != null )
			{
				VoxelExtension.Dispose();
				VoxelExtension = null;
			}
		}

		//
		public virtual void UserAction_Activate( uint VoxelInfo, int x, int y, int z ) { }

		public virtual bool Interface_StoreBlock_Store( ushort VoxelType, uint Count ) { return ( false ); }
		public virtual uint Interface_PushBlock_Push( VoxelLocation DestLocation, ushort VoxelType, uint Count ) { return ( 0 ); }
		public virtual uint Interface_PushBlock_PushTest( VoxelLocation DestLocation, ushort VoxelType, uint Count ) { return ( Count ); }
		public virtual uint Interface_PushBlock_Pull( VoxelLocation DestLocation, out ushort VoxelType, uint Count ) { VoxelType = 0; return ( 0 ); }
		public virtual uint Interface_PushBlock_PullTest( VoxelLocation DestLocation, out ushort VoxelType, uint Count ) { VoxelType = 0; return ( 0 ); }

		// Squirrel interface
		//public virtual bool Interface_GetInfo( VoxelLocation VLoc, uint InfoNum, ZVar Out ) { return ( false ); }
		//public virtual bool Interface_GetInfoDoc( uint InfoNum, uint DocType, ZVar Out ) { return ( false ); }
		//public virtual bool Interface_SetInfo( VoxelLocation VLoc, uint InfoNum, ZVar In ) { return ( false ); }
		public virtual void GetBlockInformations( VoxelLocation DestLocation, string Infos ) { return; }

		// When an active voxel should be processed. Note some voxels use "direct" faster way.
		//public virtual void ActiveProcess( ActiveVoxelInterface AvData ) { };

		public virtual bool React( VoxelRef self, double tick ) { return false; }

	}
}
