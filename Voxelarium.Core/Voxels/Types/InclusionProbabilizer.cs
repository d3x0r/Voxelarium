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
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels.Types
{
	public class InclusionProbabilizer
	{
		const int MaxEntryCount = 32;
		const int MaxValue = 2048;

		internal struct EntryInfo
		{
			internal ushort VoxelType;
			internal float RepeatChance;
			internal float Percent;
		};

		internal struct Bnd
		{
			internal uint Low;
			internal uint High;
		};

		uint EntryCount;
		Bnd[] Bounds = new Bnd[MaxEntryCount];
		EntryInfo[] Infos = new EntryInfo[MaxEntryCount];
		float CombinedPercent;
		uint FirstFenceBound;

		public void Clear()
		{
			EntryCount = 0;
			CombinedPercent = 0;
		}

		public InclusionProbabilizer()
		{
			Clear();
		}

		public void AddEntry( ushort VoxelType, float RepeatChance, float Percent )
		{
			if( EntryCount >= MaxEntryCount ) while( true ) { }

			Infos[EntryCount].VoxelType = VoxelType;
			Infos[EntryCount].RepeatChance = MaxValue * RepeatChance / 100;
			Infos[EntryCount].Percent = Percent;
			CombinedPercent += Percent;
			EntryCount++;
		}

		public uint ComputeProbabilities( float MainInclusionCoef = 1.0f )
		{
			uint i;
			float PercentValue = ( (float)MaxValue ) / 100.0f;
			float Bound = 0;
			float CombinedCoef = 100.0f / CombinedPercent;

			for( i = 0; i < EntryCount; i++ )
			{
				Bounds[i].Low = (uint)Bound;
				Bound += ( CombinedCoef * Infos[i].Percent * PercentValue );
				Bounds[i].High = (uint)Bound;
			}
			Bounds[0].Low = 0;
			Bounds[EntryCount - 1].High = MaxValue;
			FirstFenceBound = (uint)( PercentValue * CombinedPercent * MainInclusionCoef );

			return ( FirstFenceBound );
		}

		public bool AllowRepeat( uint EntryNum, SaltyRandomGenerator RandomGen )
		{
			uint RandomNumber = (uint)RandomGen.GetEntropy( 11, false );
			return RandomNumber < Infos[EntryNum].RepeatChance;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public uint GetTypeNum( SaltyRandomGenerator RandomGen )
		{
			uint Pivot, Step;
			uint RandomNumber = (uint)RandomGen.GetEntropy( 11, false );
				
			//RandomNumber &= 0x7fffffff;
			Pivot = EntryCount >> 1;
			Step = Pivot >> 1;

			while( true )
			{

				if( RandomNumber < Bounds[Pivot].Low ) Pivot -= Step;
				else if( RandomNumber > Bounds[Pivot].High ) Pivot += Step;
				else return ( Pivot );
				Step = Step >> 1;
				if( Step == 0 ) Step = 1;

				// Bound checking enabled to track errors.
				if( VoxelGlobalSettings.COMPILEOPTION_BOUNDCHECKING )
					if( Pivot >= EntryCount ) throw new Exception( "Out Of Bounds" );
			}
			return ( 0 );
		}

		public bool IsBelowFence( SaltyRandomGenerator RandomGen )
		{
			uint num = (uint)RandomGen.GetEntropy( 11, false );
			if( num < FirstFenceBound )
				return true;
			return false;
        }

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public ushort GetVoxelType( uint EntryNum ) { return ( Infos[EntryNum].VoxelType ); }
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		//public uint  GetRepeatChance( uint EntryNum ) { return ( Infos[EntryNum].RepeatChance ); }

		public void PrintInfos() { }
	}
}
