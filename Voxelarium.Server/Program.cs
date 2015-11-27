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
		static Server server;

		internal static void Exit()
		{
			done = true;
			sleep_event.Set();
		}


		static void Main( string[] args )
		{
			server = new Server();
			sleep_event = new AutoResetEvent( false );
			while( !done )
				sleep_event.WaitOne();
		}
	}
}
