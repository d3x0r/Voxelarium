using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Net;
using System.Text;

namespace Voxelarium.MasterServer
{
	internal class Protocol
	{
		internal enum Message
		{
			Hello, // register a new server
			ListServers,  // list all registered servers 
			ConnectionUpdate, // server reports new client count
		}

		[ProtoContract]
		public class Hello
		{
			[ProtoMember( 1 )]
			public string ServerName { get; set; }
			[ProtoMember( 2 )]
			public int Connections { get; set; }
			[ProtoMember( 3 )]
			IPAddress address;
			public Hello()
			{
			}
		}
		[ProtoContract]
		public class Server
		{
			[ProtoMember( 1 )]
			public string ServerName { get; set; }
			[ProtoMember( 2 )]
			public int Connections { get; set; }
			[ProtoMember( 3 )]
			public int MaxConnections { get; set; }
			[ProtoMember( 3 )]
			public int PendingConnections { get; set; }

			public Server()
			{
			}
		}
	}
}
