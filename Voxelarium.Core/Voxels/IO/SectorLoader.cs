using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels.IO
{
	internal interface SectorLoader
	{
		bool Init();
		void Cleanup();
		void Request_Sector( int x, int y, int z, int Priority );
		bool Is_EjectFileNotFull();
		void Eject_Sector( VoxelSector Sector );
		VoxelSector GetRequested();
	}
}
