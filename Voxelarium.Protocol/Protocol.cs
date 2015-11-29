using ProtoBuf;
using System;
using System.Collections.Generic;

using System.Net;
using System.Text;

namespace Voxelarium.Protocol
{
	public enum Message
	{
		ServerHello, // register a new server
		ClientHello, // register a new server
		ListServers,  // list all registered servers 
		ConnectionUpdate, // server reports new client count
		Servers, // some servers  from the list 
		Ping,
		PingReply,
	}

	[ProtoContract]
	public class ClientHello
	{
		[ProtoMember( 1 )]
		public Guid ClientID { get; set; }
		public ClientHello()
		{
		}
	}

	[ProtoContract]
	public class ServerHello
	{
		[ProtoMember( 1 )]
		public string ServerName { get; set; }
		[ProtoMember( 2 )]
		public int Connections { get; set; }
		[ProtoMember( 4 )]
		public Guid ServerID { get; set; }
		[ProtoMember( 5 )]
		public int Port;
		public ServerHello()
		{
		}
	}

	[ProtoContract]
	public class ListServers
	{
		[ProtoMember( 1 )]
		public int start_offset { get; set; }
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
		[ProtoMember( 5 )]
		public int Port { get; set; }
		[ProtoMember( 2 )]
		public int Connections { get; set; }
		[ProtoMember( 3 )]
		public int MaxConnections { get; set; }
		[ProtoMember( 4 )]
		public int PendingConnections { get; set; }

		public Server()
		{
		}
	}
}
