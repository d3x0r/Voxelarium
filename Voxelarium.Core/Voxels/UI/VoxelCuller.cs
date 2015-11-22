/*
 * This file is part of Voxelarium.
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

		internal abstract void CullSector( VoxelSector Sector, bool internal_faces, VoxelSector.FACEDRAW_Operations interesting_faces );
	}
}
