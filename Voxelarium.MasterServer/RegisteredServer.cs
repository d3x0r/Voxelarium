using System;
using System.Collections.Generic;
using System.Net;
using System.Text;

namespace Voxelarium.MasterServer
{
	internal class RegisteredServer
	{
		internal string name;
		internal IPAddress[] Addresses;
		internal int max_connections;
		internal int pending_connections;
		internal int active_connections;
	}
}
