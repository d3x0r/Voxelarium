/*
 * This file is part of Voxelarium. Split properties from VoxelSectorType
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
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using Voxelarium.Core.Types;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.Core.UI;

namespace Voxelarium.Core.Voxels
{
	public class VoxelProperties
	{
		public string VoxelTypeName;
		public ushort Type;

		public string VoxelClassName;
		//Type VoxelExtension;
		//Type VoxelType;

		//byte CanPassThrough;
		// Obsoleted
		public bool Draw_TransparentRendering;
		public bool Draw_FullVoxelOpacity;
		// New
		public byte DrawInfo;
		public Color FaceColor;
		public Color EdgeColor;
		public short EdgePower; 
		//
		public bool Is_NoType;                    // Defined if this is a default "No type" Entry;
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

		public VoxelProperties( ushort Type )
		{
			this.Type = Type;

			Is_PlayerCanPassThrough = false;
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
			FabInfo = null;
			Documentation_PageNum = 0;
		}

		public bool LoadVoxelProperties( )
		{
			StreamReader sr;

			// Get the right folder path

			string FileName;
			if( Type < 32768 )
			{
				FileName = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "VoxelTypes/voxelinfo/" + String.Format( "voxelinfo_" + (ulong)Type + ".txt" );
			}
			else
			{
				FileName = VStreamFile.Get_Directory_UserData()
					+ "/" + VoxelGlobalSettings.COMPILEOPTION_SAVEFOLDERNAME
					+ "/VoxelTypes/voxelinfo/" + String.Format( "voxelinfo_" + (ulong)Type + ".txt" );
			}
			int location;
			if( Display.FileExists( FileName, out location ) )
			{
				using( sr = Display.FileOpenText( location, FileName ) )
				{
					string input;
					while( ( input = sr.ReadLine() ) != null )
					{
						if( input.Length == 0 )
							continue;
						XString line = XString.Burst( input );
						string Token = line.firstseg.Text;
						XStringSeg value = line.firstseg.Next.Next;
						String sValue;
						int intValue;
						double dblValue;
						if( !Double.TryParse( value.Text, out dblValue ) )
						{
							dblValue = 0;
						}
						if( !Int32.TryParse( value.Text, out intValue ) )
						{
							intValue = 0;
						}
						sValue = value.Expand();

						if( Token == "Draw_TransparentRendering" )
						{

							if( intValue > 0 ) { DrawInfo |= VoxelGlobalSettings.ZVOXEL_DRAWINFO_DRAWTRANSPARENTRENDERING; Draw_TransparentRendering = true; }
							else { DrawInfo &= 0xFF ^ VoxelGlobalSettings.ZVOXEL_DRAWINFO_DRAWTRANSPARENTRENDERING; Draw_TransparentRendering = false; }
						}
						if( Token == "Draw_FullVoxelOpacity" )
						{
							if( intValue > 0 ) { DrawInfo |= VoxelGlobalSettings.ZVOXEL_DRAWINFO_DRAWFULLVOXELOPACITY; Draw_FullVoxelOpacity = true; }
							else { DrawInfo &= 0xFF ^ VoxelGlobalSettings.ZVOXEL_DRAWINFO_DRAWFULLVOXELOPACITY; Draw_FullVoxelOpacity = false; }
						}
						if( Token == "Draw_SpecialRender" ) { if( intValue > 0 ) DrawInfo |= VoxelGlobalSettings.ZVOXEL_DRAWINFO_SPECIALRENDERING; else DrawInfo &= 0xFF ^ VoxelGlobalSettings.ZVOXEL_DRAWINFO_SPECIALRENDERING; }

						if( Token == "VoxelTypeName" ) VoxelTypeName = sValue;

						if( Token == "ExtensionType" ) { ExtensionType = (uint)intValue; }
						if( Token == "MiningHardness" ) { MiningHardness = (double)intValue; }
						if( Token == "MiningType" ) { MiningType = (uint)intValue; }
						if( Token == "BvProp_PlayerCanPassThrough" ) { Is_PlayerCanPassThrough = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_Harming" ) { Is_Harming = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_Active" ) { Is_Active = ( intValue != 0 ) ? true : false; }
						if( Token == "FrictionCoef" ) { FrictionCoef = dblValue; if( FrictionCoef == 0.0 ) FrictionCoef = 1.0; }
						if( Token == "Grip_Horizontal" ) { Grip_Horizontal = dblValue; if( Grip_Horizontal == 0.0 ) Grip_Horizontal = 1.0; }
						if( Token == "Grip_Vertical" ) { Grip_Vertical = dblValue; if( Grip_Vertical == 0.0 ) Grip_Vertical = 1.0; }
						if( Token == "BvProp_KeepControlOnJumping" ) { Is_KeepControlOnJumping = ( intValue != 0 ) ? true : false; }
						if( Token == "HarmingLifePointsPerSecond" ) { HarmingLifePointsPerSecond = dblValue; }
						if( Token == "BvProp_CombinableWith_GreenAcid" ) { Is_CombinableWith_GreenAcid = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_CanBeReplacedBy_MustardGaz" ) { Is_CanBeReplacedBy_MustardGaz = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_CanTriggerBomb" ) { Is_CanTriggerBomb = ( intValue != 0 ) ? true : false; }
						if( Token == "BlastResistance" ) { BlastResistance = ( intValue ); }
						if( Token == "BvProp_Liquid" ) { Is_Liquid = ( intValue != 0 ) ? true : false; }
						if( Token == "LiquidDensity" ) { LiquidDensity = dblValue; }
						if( Token == "BvProp_Gaz" ) { Is_Gaz = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_Pumpable_ByPump_T1" ) { Is_Pumpable_ByPump_T1 = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_Pumpable_ByPump_T2" ) { Is_Pumpable_ByPump_T2 = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_Loadable_ByLoader_L1" ) { Is_Loadable_ByLoader_L1 = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_FastMoving" ) { BvProp_FastMoving = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_CanBePickedUpByRobot" ) { BvProp_CanBePickedUpByRobot = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_XrRobotPickMinLevel" ) { BvProp_XrRobotPickMinLevel = (byte)intValue; }
						if( Token == "Documentation_PageNum" ) { Documentation_PageNum = (uint)intValue; }
						if( Token == "BvProp_AtomicFireResistant" ) { BvProp_AtomicFireResistant = ( intValue != 0 ) ? true : false; }
						if( Token == "BvProp_EgmyT1Resistant" ) { BvProp_EgmyT1Resistant = ( intValue != 0 ) ? true : false; }
						if( Token == "VoxelClassName" ) { VoxelClassName = sValue; }
						if( Token == "EdgeColor" )
						{
							DrawInfo |= VoxelGlobalSettings.ZVOXEL_DRAWINFO_SHADER; EdgeColor = XColor.DeserializeColor( sValue );
						}
						if( Token == "FaceColor" )
						{
							DrawInfo |= VoxelGlobalSettings.ZVOXEL_DRAWINFO_SHADER; FaceColor = XColor.DeserializeColor( sValue );
						}
						if( Token == "EdgePower" )
						{
							DrawInfo |= VoxelGlobalSettings.ZVOXEL_DRAWINFO_SHADER; EdgePower = (short)intValue;
						}
					}
				}
			}
			return ( true );
		}

		public static VoxelProperties Load( ushort type )
		{
			VoxelProperties props = new VoxelProperties( type );
			props.LoadVoxelProperties();
			return props;
		}
	}
}
