using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;

namespace Voxelarium.Server
{
	class Program
	{
		static AutoResetEvent sleep_event;
		static bool done;

		internal static GameServer game_server;
		internal static MasterServerConnector master;

		internal static void Exit()
		{
			done = true;
			sleep_event.Set();
		}


		static void Main( string[] args )
		{
			game_server = new GameServer();
			master = new MasterServerConnector();
			sleep_event = new AutoResetEvent( false );
			while( !done )
				sleep_event.WaitOne();
		}
	}
}
