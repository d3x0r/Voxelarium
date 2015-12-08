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
//#define ALLOW_UNSAFE
#define ALLOW_INLINE
using System;
using System.Collections.Generic;
#if ALLOW_INLINE
using System.Runtime.CompilerServices;
#endif
using System.Text;

namespace Voxelarium.Core.Voxels.Types
{
	public class FastBit_Array_32k
	{
		internal uint[] Storage = new uint[1024];

		~FastBit_Array_32k()
		{
			Storage = null;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal void Clear()
		{
			ushort i;
			for( i = 0; i < 1024; i++ ) Storage[i] = 0;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void Set( int Index, bool Data )
		{
			int Remain;
			int Offset;
			Remain = (ushort)(Index & 0x1f);
			Offset = Index >> 5;
			//Remain &= 0x1f;
			if( Data ) Storage[Offset] |= (uint)1 << Remain;
			else Storage[Offset] &= ~( (uint)1 << Remain );
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		 public bool Get( ushort Index )
		{
			ushort Remain;
			int Offset;
			Remain = (ushort)(Index & 0x1f);
			Offset = Index >> 5;
#if ALLOW_UNSAFE
			unsafe
			{
				fixed( uint*pStorage = Storage )
				{
					return ( *( pStorage + Offset ) & ( (uint)1 << Remain ) ) != 0;
				}
			}
#else
			return ( Storage[Offset] & ( (uint)1 << Remain ) ) != 0;
#endif
		}
	}
}
