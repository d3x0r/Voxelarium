using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Net;
using System.Text;
using Voxelarium.Protocol;

namespace Voxelarium.MasterServer
{
	[ProtoContract]
	internal class RegisteredServer
	{
		[ProtoMember( 1 )]
		internal string name { get; set; }
		[ProtoMember( 2 )]
		internal List<byte[]> AddressBytes { get; set; }
		internal List<IPAddress> Addresses;
		[ProtoMember( 3 )]
		internal int max_connections { get; set; }
		[ProtoMember( 4 )]
		internal int pending_connections { get; set; }
		[ProtoMember( 5 )]
		internal int active_connections { get; set; }
		[ProtoMember( 6 )]
		internal Guid ID;
		[ProtoMember( 7 )]
		internal int Port;

		internal void RemoveAddress( IPAddress host_address )
		{
			AddressBytes.Remove( host_address.GetAddressBytes() );
			Addresses.Remove( host_address );
		}
		internal void AddAddress( IPAddress host_address )
		{
			AddressBytes.Add( host_address.GetAddressBytes() );
			Addresses.Add( host_address );
		}
		internal RegisteredServer()
		{
			AddressBytes = new List<byte[]>();
			Addresses = new List<IPAddress>();
			//new IPAddress( AddressBytes );
		}
	}
	internal class RegisteredServers : List<RegisteredServer>
	{
		internal RegisteredServer AddServer( ServerHello hello, IPAddress address )
		{
			foreach( RegisteredServer server in this )
			{
				if( server.name == hello.ServerName && server.ID == hello.ServerID )
				{
					foreach( IPAddress test in server.Addresses )
					{
						if( test.Equals( address ) )
							return server;
					}
					server.AddAddress( address );
					return server;
				}
			}
			RegisteredServer new_server = new RegisteredServer();
			new_server.name = hello.ServerName;
			new_server.AddAddress( address );
			new_server.max_connections = hello.Connections;
			new_server.pending_connections = 0;
			new_server.active_connections = 0;
			new_server.Port = hello.Port;
			return new_server;
		}
	}
}
