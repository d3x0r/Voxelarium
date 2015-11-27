using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Text;

namespace Voxelarium.Protocol
{
		internal enum Message
		{
			Hello, // register a new server
			ListServers,  // list all registered servers 
			ConnectionUpdate, // server reports new client count
			Servers, // some servers  from the list 
			Ping,
			PingReply,
		}

		[ProtoContract]
		public class Hello
		{
			[ProtoMember( 1 )]
			public string ServerName { get; set; }
			[ProtoMember( 2 )]
			public int Connections { get; set; }
			[ProtoMember( 3 )]
			IPAddress[] addresses;
			public Hello()
			{
			}
		}

	[ProtoContract]
	public class ListServers
	{
		[ProtoMember( 1 )]
		internal int start_offset { get; set; }
		public ListServers()
		{
		}
	}

	[ProtoContract]
	public class ConnectionUpdate
	{
		[ProtoMember( 1 )]
		internal int Connections { get; set; }
		public ConnectionUpdate() { }
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
