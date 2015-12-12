﻿using ProtoBuf;
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
		ConnectToClient,
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
	public class ConnectToClient
	{
		[ProtoMember( 1 )]
		public Guid ClientID { get; set; }
		[ProtoMember( 2 )]
		public byte[] byAddressV4;
		[ProtoMember( 3 )]
		public byte[] byAddressV6;
		[ProtoMember( 4 )]
		public int port;

		public IPEndPoint ClientAddressV4;
		public IPEndPoint ClientAddressV6;
		public IPEndPoint ServerAddressV4;
		public IPEndPoint ServerAddressV6;
		public ConnectToClient()
		{
		}

		public void SetupAddresses()
		{
			if( byAddressV4 != null )
			{
				IPAddress ipaddr = new IPAddress( byAddressV4 );
				ClientAddressV4 = new IPEndPoint( ipaddr, port );
			}
			if( byAddressV6 != null )
			{
				IPAddress ipaddr = new IPAddress( byAddressV6 );
				ClientAddressV6 = new IPEndPoint( ipaddr, port );
			}
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
		public int Port { get; set; }
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
		public int Connections { get; set; }
		public ConnectionUpdate() { }
	}


	/// <summary>
	/// Received by clients from master server
	/// </summary>
	[ProtoContract]
	public class RegisteredGameServer
	{
		[ProtoMember( 1 )]
		public string name { get; set; }
		//[ProtoMember( 2 )]
		public List<byte[]> AddressBytes { get; set; }
		public List<IPAddress> Addresses;
		[ProtoMember( 3 )]
		public int max_connections { get; set; }
		[ProtoMember( 4 )]
		public int pending_connections { get; set; }
		[ProtoMember( 5 )]
		public int active_connections { get; set; }
		[ProtoMember( 6 )]
		public Guid ID { get; set; }
		[ProtoMember( 7 )]
		public int Port { get; set; }
		public RegisteredGameServer(){}
	}
}
