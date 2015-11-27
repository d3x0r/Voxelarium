using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Net;
using System.Text;

namespace Voxelarium.MasterServer
{
	[ProtoContract]
	internal class RegisteredServer
	{
		[ProtoMember( 1 )]
		internal string name { get; set; }
		[ProtoMember( 2 )]
		internal List<IPAddress> Addresses { get; set; }
		[ProtoMember( 3 )]
		internal int max_connections { get; set; }
		[ProtoMember( 4 )]
		internal int pending_connections { get; set; }
		[ProtoMember( 5 )]
		internal int active_connections { get; set; }

		internal void RemoveAddress( IPAddress host_address )
		{
			Addresses.Remove( host_address );
		}
	}
	internal class RegisteredServers : List<RegisteredServer>
	{
		internal RegisteredServer AddServer( string name, int max_connections, IPAddress address )
		{
			foreach( RegisteredServer server in this )
			{
				if( server.name == name )
				{
					foreach( IPAddress test in server.Addresses )
					{
						if( test.Equals( address ) )
							return server;
					}
					server.Addresses.Add( address );
					return server;
				}
			}
			RegisteredServer new_server = new RegisteredServer();
			new_server.name = name;
			new_server.Addresses.Add( address );
			new_server.max_connections = max_connections;
			new_server.pending_connections = 0;
			new_server.active_connections = 0;
			return new_server;
		}
	}
}
