using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using Voxelarium.Common;
using Voxelarium.Core.UI;

namespace Voxelarium.Server
{
	class Program
	{
		static AutoResetEvent sleep_event;
		static bool done;

		internal static GameServer game_server;
		internal static MasterServerConnector master;
		internal delegate void SimpleAtExit();
		internal static event SimpleAtExit AtExit;

		internal static void Exit()
		{
			if( AtExit != null )
				AtExit();
			done = true;
			sleep_event.Set();
		}


		static void Main( string[] args )
		{
			Log.log( "Game Server Starting.." );
			game_server = new GameServer();
			master = new MasterServerConnector();
			sleep_event = new AutoResetEvent( false );
			while( !done )
				sleep_event.WaitOne();
			// Yes, technically there is no display; but existing threads registered shutdown event
			// with AtExit event list in Display; so use that to gracefully exit.
			Display.Shutdown();
		}
	}
}
