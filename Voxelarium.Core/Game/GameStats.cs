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
		VoxelGameEnvironment GameEnv;
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
			GameEnv = null;
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
