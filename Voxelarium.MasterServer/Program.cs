using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;

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
			server = new RegistryServer();
			while( !done )
				sleep_event.WaitOne();
		}
	}
}
