using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using Voxelarium.Common;

namespace Voxelarium.MasterServer
{
	internal class Program
	{
		static AutoResetEvent sleep_event;
		static bool done;
		static RegistryServer server;

		internal static void Exit()
		{
			done = true;
			sleep_event.Set();
		}

		static void Main( string[] args )
		{
			Log.log( "Master Server Starting.." );
			server = new RegistryServer();
			sleep_event = new AutoResetEvent( false );
			while( !done )
				sleep_event.WaitOne();
		}
	}
}
