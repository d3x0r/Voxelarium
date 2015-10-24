using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public interface IWorldGenesis
	{
		void GenerateSector( VoxelSector VoxelSector );
		bool LoadTemplateImages();
	}
}
