using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	internal class VoxelType_Void : VoxelType
	{
		public VoxelType_Void()
		{
			PropertiesSet += UpdateProperties;
		}

		void UpdateProperties()
		{
			properties.Is_PlayerCanPassThrough = true;
			properties.Draw_TransparentRendering = false;
			properties.Draw_FullVoxelOpacity = false;
			properties.DrawInfo = VoxelGlobalSettings.ZVOXEL_DRAWINFO_VOID;
			properties.Is_Harming = false;
			properties.FrictionCoef = 0.0001;
			properties.Grip_Vertical = 0.0;
			properties.Grip_Horizontal = 0.8;
			properties.Is_SpaceGripType = false;
			properties.Is_KeepControlOnJumping = true;
			properties.Is_Active = false;
			properties.Is_CanBeReplacedBy_Water = true;
			properties.Is_CombinableWith_GreenAcid = false;
			properties.Is_CanBeReplacedBy_GreenAcid = true;
			properties.Is_CanBeReplacedBy_MustardGaz = true;
			properties.BvProp_CanBePickedUpByRobot = false;
			properties.BvProp_XrRobotPickMinLevel = 255;
			properties.BvProp_PrRobotReplaceMinLevel = 0;
			properties.BvProp_PrRobotPickMinLevel = 255;
			properties.BvProp_PrRobotMoveMinLevel = 0;
			properties.BvProp_AtomicFireResistant = true;
			properties.Is_Liquid = false;
			properties.Is_Gaz = true;
			properties.Is_Loadable_ByLoader_L1 = false;
			properties.BvProp_MoveableByTreadmill = false;
			properties.BvProp_EgmyT1Resistant = false;
			properties.LiquidDensity = 0.0;
			properties.VoxelTypeName = "VOID";
		}
	}
}
