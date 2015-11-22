/*
 * Vision Source Implementation uses same 'Activate', Red,green,blue,opacity variables.
 * 
 * This file is part of Voxelarium
 *
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

//#define COLUMN_MAJOR_EXPECTED

using Voxelarium.LinearMath;

namespace Voxelarium.Core.Voxels.UI
{
	internal class Camera
	{
		internal btTransform location;
		internal struct VisionColor
	    {
			internal bool Activate;
			internal byte Red, Green, Blue, Opacity;
		}
		internal VisionColor ColoredVision;

		internal Camera()
		{
			location = btTransform.Identity;
		}
		internal void MoveTo( float x, float y, float z )
		{
			location.Translate( x, y, z );
		}
		internal void RotateYaw( float yaw )
		{
			location.m_basis.Rotate( 1, yaw );
		}
		internal void RotatePitch( float pitch )
		{
			location.m_basis.Rotate( 0, pitch );
		}
		internal void RotateRoll( float roll )
		{
			location.m_basis.Rotate( 2, roll );
		}
		internal void MoveForward( float delta )
		{
			btVector3 dir;
#if COLUMN_MAJOR_EXPECTED
			location.m_basis.getColumn( 2 ).Mult( delta, out dir );
#else
			location.m_basis.m_el2.Mult( delta, out dir );
#endif
			location.Move( dir.x, dir.y, dir.z );
		}
		internal void MoveRight( float delta )
		{
			btVector3 dir;
#if COLUMN_MAJOR_EXPECTED
			location.m_basis.getColumn( 0 ).Mult( delta, out dir );
#else
			location.m_basis.m_el0.Mult( delta, out dir );
#endif
			location.Move( dir.x, dir.y, dir.z );
		}
		internal void MoveUp( float delta )
		{
			btVector3 dir;
#if COLUMN_MAJOR_EXPECTED
			location.m_basis.getColumn( 1 ).Mult( delta, out dir );
#else
			location.m_basis.m_el1.Mult( delta, out dir );
#endif
			location.Move( dir.x, dir.y, dir.z );
		}
	}
}
