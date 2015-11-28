using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Voxelarium.Core.Networking;

namespace Voxelarium.Core.Voxels.IO
{
	internal class NetworkSectorLoader : SectorLoader
	{
		VoxelGameEnvironment game;
		bool ThreadContinue;
		AutoResetEvent SleepEvent = new AutoResetEvent( false );
		Thread loader_thread;
		ClientConnection connection;
		VoxelTypeManager VoxelTypeManager;
		int UniverseNum;

        internal NetworkSectorLoader( VoxelGameEnvironment game, ClientConnection connection )
		{
			this.game = game;
		}

		public void SetVoxelTypeManager( VoxelTypeManager VoxelTypeManager ) { this.VoxelTypeManager = VoxelTypeManager; }

		public void SetUniverseNum( int UniverseNum ) { this.UniverseNum = UniverseNum; }

		public void Cleanup()
		{
			ThreadContinue = false;
			while( loader_thread.ThreadState == ThreadState.Running )
			{
				SleepEvent.Set();
				System.Threading.Thread.Sleep( 10 );
			}
		}

		public void Eject_Sector( VoxelSector Sector )
		{
			throw new NotImplementedException();
		}

		public VoxelSector GetRequested()
		{
			throw new NotImplementedException();
		}

		public bool Init( ref int start_percent, ref int step, ref int steps )
		{
			throw new NotImplementedException();
		}

		public bool Is_EjectFileNotFull()
		{
			throw new NotImplementedException();
		}

		public void Request_Sector( int x, int y, int z, int Priority, EventWaitHandle wait_event = null )
		{
			throw new NotImplementedException();
		}
	}
}
