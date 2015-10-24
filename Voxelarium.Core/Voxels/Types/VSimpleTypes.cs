using OpenTK;
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels.Types
{
	public struct ZPolar3f { public float yaw, pitch, roll, Len; }

	//public struct ZVector4d { double x, y, z, w;}
	public struct ZVector2 { public float x, y; };
	//public struct { float x, y; }   ZVector2f;
	//public struct { Long x, y; } ZVector2L;
	//public struct { float Position_x, Position_y, Position_z, Width, Height, Depth; } ZBox3f;
	//public struct { float r, v, b;} ZColor3f;
	public partial struct ZVector3L { public int x, y, z; }


	public struct ZRect3L{ public ZVector3L Start, End; } 
	public struct ZRect3L_2 { public int sx, sy, sz, ex, ey, ez; }
	public struct ZRect3UL_2 { public uint sx, sy, sz, ex, ey, ez; }
	public struct ZRect1d { public double Start, End; };
	public struct ZRect1f { public float Start, End; };
	public struct ZLineCoords { public ZVector2 Start, End; };
	public struct Box
	{
		internal Vector3 Position; // Size.Z too
		internal Vector3 Size;
	}
	public struct Box2D
	{
		internal Vector2 Position; // Size.Z too
		internal Vector2 Size;
	}

}
