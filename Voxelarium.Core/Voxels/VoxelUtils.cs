using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	class VoxelUtils
	{
		public static uint MulticharConst( char a, char b, char c, char d ) { return ( (uint)a ) | ( (uint)b << 9 ) | ( (uint)c << 16 ) | ( (uint)d << 24 ); }
	}
}
