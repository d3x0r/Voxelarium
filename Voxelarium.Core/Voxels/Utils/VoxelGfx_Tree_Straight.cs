﻿/*
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
using Voxelarium.Core.Types;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Voxels.Utils
{
	public class VoxelGfx_Tree_Straight
	{
		VoxelSector Sector;
		LightSpeedRandom Random;
		LinearInterpolator ThicknessInterpolator = new LinearInterpolator();
		public ZPolar3f StartingDirection;
		public float TruncHeight;
		public uint MinSubDivisionsPerNode;
		public uint MaxSubDivisionsPerNode;
		public bool RandomSubdivs;
		public float BrancheLen;
		public float SecundaryTruncSegmentLenght;
		public float MaxBranchLenght;
		public bool HasFolliage;
		public float FolliageLenght;

		//uint ZVoxelGfx_Tree_Straight::Seed = 0;
		public uint Seed;

		public void DrawBranch( ref ZVector3f Position, ref ZPolar3f BranchVector, float TotalBranchLen )
		{
			ZVector3f NewPosition;
			ZPolar3f NewBranchVector;
			ZRect3L Rect;
			ZRect1f Thickness;
			uint nSubDivisions;
			float Angle1, Angle2, NewTotalBranchLen;


			NewTotalBranchLen = TotalBranchLen + BranchVector.Len;
			if( NewTotalBranchLen > MaxBranchLenght ) return;

			NewPosition = Position + BranchVector;
			Rect.Start = Position + 0.5f;
			Rect.End = NewPosition + 0.5f;
			Thickness.Start = ThicknessInterpolator.Interpolate( TotalBranchLen ); Thickness.End = ThicknessInterpolator.Interpolate( NewTotalBranchLen );
			// Thickness.Start = 0.5; Thickness.End = 0.5;
			// printf("S:%lu (%ld,%ld,%ld = %ld,%ld,%ld)\n", nStep, Rect.Start.x, Rect.Start.y, Rect.Start.z, Rect.End.x, Rect.End.y, Rect.End.z );
			if( TotalBranchLen > 0 ) Sector.Draw_safe_VoxelLine( ref Rect, ref Thickness, 72 );
			if( HasFolliage && NewTotalBranchLen > FolliageLenght ) Sector.Draw_safe_Sphere( Rect.End.x, Rect.End.y, Rect.End.z, 10, 70, false );


			if( RandomSubdivs ) nSubDivisions = MinSubDivisionsPerNode + (uint)Random.GetNumber() % MaxSubDivisionsPerNode;
			else nSubDivisions = MinSubDivisionsPerNode;
			if( TotalBranchLen > ( MaxBranchLenght / 10 ) && nSubDivisions > 2 ) nSubDivisions = 2;
			//if (nSubDivisions>2) nSubDivisions = 2;

			nSubDivisions = 1;


			switch( nSubDivisions )
			{

				case 1:
					NewBranchVector = BranchVector;
					NewBranchVector.Len = BrancheLen;
					Angle1 = (float)( Random.GetNumber() ) / 23860929 / 6;
					Angle2 = (float)( Random.GetNumber() ) / 23860929 / 6;
					//NewBranchVector.pitch += Angle1; NewBranchVector.yaw   += Angle2;
					DrawBranch( ref NewPosition, ref NewBranchVector, NewTotalBranchLen );
					break;

				case 2:
				case 3:
				case 4:
				default:
					NewBranchVector = BranchVector;
					NewBranchVector.Len = BrancheLen;
					do
					{
						Angle1 = (float)( Random.GetNumber() ) / 23860929 / 3;
						Angle2 = (float)( Random.GetNumber() ) / 23860929 / 3;
					} while( Math.Abs( Angle1 ) < 15 && Math.Abs( Angle2 ) < 15 );


					NewBranchVector.pitch += Angle1; NewBranchVector.yaw += Angle2;
					DrawBranch( ref NewPosition, ref NewBranchVector, NewTotalBranchLen );
					NewBranchVector = BranchVector;
					NewBranchVector.pitch -= Angle1; NewBranchVector.yaw -= Angle2;
					DrawBranch( ref NewPosition, ref NewBranchVector, NewTotalBranchLen );
					break;
			}


		}

		void DrawSecundaryTrunc( ref ZVector3f Position, ref ZPolar3f BranchVector, float TotalBranchLen )
		{
			ZVector3f NewPosition;
			ZPolar3f NewBranchVector;
			ZRect3L Rect;
			ZRect1f Thickness;
			//uint nSubDivisions;
			float Angle1, Angle2, NewTotalBranchLen;


			NewTotalBranchLen = TotalBranchLen + BranchVector.Len;
			if( NewTotalBranchLen > MaxBranchLenght ) return;

			NewPosition = Position + BranchVector;
			Rect.Start = Position + 0.5f;
			Rect.End = NewPosition + 0.5f;
			Thickness.Start = ThicknessInterpolator.Interpolate( TotalBranchLen ); Thickness.End = ThicknessInterpolator.Interpolate( NewTotalBranchLen );
			if( TotalBranchLen > 0 ) Sector.Draw_safe_VoxelLine( ref Rect, ref Thickness, 72 );

			// la branche

			NewBranchVector = BranchVector;
			NewBranchVector.Len = BrancheLen;
			Angle1 = (float)( Random.GetNumber() ) / 23860929 / 3;
			Angle2 = (float)( Random.GetNumber() ) / 23860929 / 3;
			if( Angle1 > 0 ) Angle1 += 15.0f;
			if( Angle1 < 0 ) Angle1 -= 15.0f;
			if( Angle2 > 0 ) Angle2 += 15.0f;
			if( Angle2 < 0 ) Angle2 -= 15.0f;
			NewBranchVector.pitch += Angle1;
			NewBranchVector.yaw += Angle2;

			DrawBranch( ref NewPosition, ref NewBranchVector, NewTotalBranchLen / 2 + 50 );

			// Le tronc central


			NewBranchVector = BranchVector;
			NewBranchVector.Len = SecundaryTruncSegmentLenght;
			DrawSecundaryTrunc( ref NewPosition, ref NewBranchVector, NewTotalBranchLen );


		}


		void DrawTrunc( ref ZVector3f Position, ref ZPolar3f Direction )
		{
			ZRect3L Rect;
			//ZRect1f Thickness;
			uint Number;

			float[] ThickTable = { 5, 4, 3, 3, 2, 2, 2 };

			//Thickness.Start = 5;
			//Thickness.End   = 5;

			Number = (uint)Random.GetNumber();
			TruncHeight = 10 + ( Number / 71582788 );

			Rect.Start = Position + 0.5f;
			float savelen = Direction.Len;
            Direction.Len *= TruncHeight;
			Position = Position + Direction;
			Direction.Len = savelen;
			Rect.End = Position + 0.5f;

			//Sector.Draw_safe_VoxelLine(&Rect,&Thickness,67);
			Sector.Draw_safe_VoxelLine_TickCtl( ref Rect, ThickTable, 7, 72 );

		}


		public void DrawTree( VoxelSector Sector, ref ZVector3f BasePosition )
		{
			ZVector3f Position;
			float[] ThicknessTable = { 2, 1 };

			//printf("-----------------------Draw Tree---------------------------\n");
			//printf("Seed : %lu",Seed);
			Random.Init( Seed );

			ThicknessInterpolator.SetValuesTable( ThicknessTable, 2 );
			ThicknessInterpolator.SetBounds( 0, MaxBranchLenght );

			this.Sector = Sector;
			Position = BasePosition;
			DrawTrunc( ref Position, ref StartingDirection );
			DrawSecundaryTrunc( ref Position, ref StartingDirection, 0 );
		}

		public VoxelGfx_Tree_Straight()
		{
			Random = new LightSpeedRandom();
			// Trunc starting direction : Vertical absolute.
			StartingDirection.Len = 1;
			StartingDirection.pitch = 90;
			StartingDirection.yaw = 90;
			StartingDirection.roll = 0;
			TruncHeight = 20; // 40
			MinSubDivisionsPerNode = 1;
			MaxSubDivisionsPerNode = 3;
			RandomSubdivs = true;
			BrancheLen = 10;
			SecundaryTruncSegmentLenght = 2.5f;
			MaxBranchLenght = 120;
			FolliageLenght = 50; // 60
			HasFolliage = true;
		}


		private void Random_getsalt( SaltyRandomGenerator.SaltData add_data_here )
		{
			add_data_here.Clear();
			add_data_here.salt_data.Add( BitConverter.GetBytes( Seed ) );
		}
	}
}
