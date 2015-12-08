using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Voxelarium.Core.Game;

namespace Voxelarium.Core.Voxels.Types
{

	public interface IVoxelExtension_Storage
	{
		Inventory.Entry GetStorageEntry( int index );
		ushort GetVoxelType( int index );
		void SetVoxelType( int index, ushort type );
		int GetVoxelQuantity( int index );
		void SetVoxelQuantity( int index, int quantity );
	}
}
