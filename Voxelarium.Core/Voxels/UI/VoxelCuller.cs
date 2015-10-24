using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels.UI
{
	internal abstract class VoxelCuller
	{
		internal abstract int getFaceCulling( VoxelSector Sector, int offset );
		internal abstract void setFaceCulling( VoxelSector Sector, int offset, VoxelSector.FACEDRAW_Operations value );

		internal abstract void InitFaceCullData( VoxelSector Sector );
		internal abstract byte[] GetData();

	}
}
