using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels.UI
{
	internal struct Camera
	{
		internal btTransform location;
		internal struct VisionColor
	    {
			internal bool Activate;
			internal byte Red, Green, Blue, Opacity;
		}
		internal VisionColor ColoredVision;
	}
}
