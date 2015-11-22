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
using System.IO;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	internal class GameStats
	{
		internal class Buffer 
		{
			internal uint[] data;
			internal Buffer( uint count )
			{
				data = new uint[count];
			}
		}
		Buffer[] Log_Buffer = new Buffer[64];
		uint BufferOffset;

		internal uint FrameTime; // Frame time in milliseconds

		internal uint SectorRefresh_Count;
		internal uint SectorRefresh_TotalTime;
		internal uint SectorRefresh_MaxTime;
		internal uint SectorRefresh_MinTime;
		internal uint SectorRefresh_Waiting;

		internal uint SectorRender_Count;
		internal uint SectorRender_TotalTime;

		internal GameStats()
		{
			uint i;

			FrameTime = 0;
			SectorRefresh_Count = 0;
			SectorRefresh_TotalTime = 0;
			SectorRefresh_MaxTime = 0;
			SectorRefresh_MinTime = 0;
			SectorRender_Count = 0;
			SectorRender_TotalTime = 0;
			SectorRefresh_Waiting = 0;
			BufferOffset = 0;

			for( i = 0; i < 64; i++ ) Log_Buffer[i] = null;
			if( VoxelGlobalSettings.COMPILEOPTION_FINETIMINGTRACKING )
				for( i = 0; i < 8; i++ ) { Log_Buffer[i] = new Buffer(1000000); }
		}

		~GameStats()
		{
			uint i;

			for( i = 0; i < 64; i++ ) if( Log_Buffer[i] != null) { Log_Buffer[i].data = null; Log_Buffer[i] = null; }
		}

		internal bool Start()
		{
			return ( true );
		}

		internal bool End()
		{
			if( VoxelGlobalSettings. COMPILEOPTION_FINETIMINGTRACKING  )
				SaveLogToFile( "./log_framerate.txt" );
			return ( true );
		}

		internal void DoLogRecord()
		{
			if( VoxelGlobalSettings.COMPILEOPTION_FINETIMINGTRACKING )
			{
				Log_Buffer[0].data[BufferOffset] = FrameTime;
				Log_Buffer[1].data[BufferOffset] = SectorRefresh_Count;
				Log_Buffer[2].data[BufferOffset] = SectorRefresh_TotalTime;
				Log_Buffer[3].data[BufferOffset] = SectorRefresh_MaxTime;
				Log_Buffer[4].data[BufferOffset] = SectorRefresh_MinTime;
				Log_Buffer[5].data[BufferOffset] = SectorRender_Count;
				Log_Buffer[6].data[BufferOffset] = SectorRender_TotalTime;
				Log_Buffer[7].data[BufferOffset] = SectorRefresh_Waiting;
				BufferOffset++;
			}
		}

		bool SaveLogToFile( string FileSpec )
		{
			StreamWriter sw = new StreamWriter( FileSpec );
			uint i;
			string As;

			{
				for( i = 0; i < BufferOffset; i++ )
				{
					As = Log_Buffer[0].data[i].ToString();
					As += "," + Log_Buffer[1].data[i];
					As += "," + Log_Buffer[2].data[i];
					As += "," + Log_Buffer[3].data[i];
					As += "," + Log_Buffer[4].data[i];
					As += "," + Log_Buffer[5].data[i];
					As += "," + Log_Buffer[6].data[i];
					As += "," + Log_Buffer[7].data[i];
					sw.WriteLine( As );
				}
				sw.Close();
			}
			return ( true );
		}



	}
}
