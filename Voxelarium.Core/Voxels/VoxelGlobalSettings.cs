using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public class VoxelGlobalSettings
	{
		public const bool COMPILEOPTION_USEHOMEDIRSTORAGE = false;
		public const bool COMPILEOPTION_BOUNDCHECKINGSLOW = false;
        public const string COMPILEOPTION_SAVEFOLDERNAME = "Voxelarium";

		public const int ZVOXEL_DRAWINFO_VOID = 0;
		public const int ZVOXEL_DRAWINFO_NOTVOID = 1;
		public const int ZVOXEL_DRAWINFO_DRAWFULLVOXELOPACITY = 2;
		public const int ZVOXEL_DRAWINFO_DRAWTRANSPARENTRENDERING = 4;
		public const int ZVOXEL_DRAWINFO_SPECIALRENDERING = 8;

		public const int VoxelBlockSizeBits = 5;
		public const int VoxelBlockSize = 1 << VoxelBlockSizeBits;

	}
}
