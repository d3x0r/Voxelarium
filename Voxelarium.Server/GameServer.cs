using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using Voxelarium.Common;

namespace Voxelarium.Server
{
	internal class GameServer
	{
		static int serving_port;
		TcpListener server;
		TcpListener server_v6;

		internal GameServer()
		{
			serving_port = Settings.Read( "Serve on Port", 31733 );

        }
	}
}
