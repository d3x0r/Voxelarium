/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel. (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
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
using OpenTK;
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels.Types
{
	public struct ZPolar3f { public float yaw, pitch, roll, Len; }
	public struct ZVector3f
	{
		public float x, y, z;
		public static ZVector3f operator +( ZVector3f a, ZPolar3f Polar )
		{
			ZVector3f Result;
			Result.x = (float)(a.x + Polar.Len * Math.Sin( Polar.yaw / 57.295779506 ) * Math.Cos( Polar.pitch / 57.295779506 ));
			Result.y = (float)( a.y + Polar.Len * Math.Sin( Polar.yaw / 57.295779506 ) * Math.Sin( Polar.pitch / 57.295779506 ));
			Result.z = (float)( a.z + Polar.Len * Math.Cos( Polar.yaw / 57.295779506 ));
			return Result;
		}
		public static ZVector3f operator +( ZVector3f a, float Scalar )
		{
			ZVector3f Result; Result.x = a.x + Scalar; Result.y = a.y + Scalar; Result.z = a.z + Scalar; return Result;
		}
		public static implicit operator ZVector3L( ZVector3f a )
		{
			ZVector3L Result;
			Result.x = (int)a.x;
			Result.y = (int)a.y;
			Result.z = (int)a.z;
			return Result;
		}
	}
	//public struct ZVector4d { double x, y, z, w;}
	public struct ZVector2 { public float x, y; };
	//public struct { Long x, y; } ZVector2L;
	//public struct { float Position_x, Position_y, Position_z, Width, Height, Depth; } ZBox3f;
	//public struct { float r, v, b;} ZColor3f;
	public partial struct ZVector3L { public int x, y, z; }


	public struct ZRect3L{ public ZVector3L Start, End; } 
	public struct ZRect3L_2 { public int sx, sy, sz, ex, ey, ez; }
	public struct ZRect3UL_2 { public uint sx, sy, sz, ex, ey, ez; }
	//public struct ZRect1d { public double Start, End; };
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
