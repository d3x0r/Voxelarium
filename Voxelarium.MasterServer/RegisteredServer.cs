/*
 * This file is part of Voxelarium.
 *
 * Copyright 2015-2016 James Buckeyne  *** Added 18/12/2015
 *
 * Voxelarium is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Voxelarium is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Net;
using System.Text;
using Voxelarium.Protocol;

namespace Voxelarium.MasterServer
{
	public class RegisteredServer:  Protocol.RegisteredGameServer
	{
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
		public RegisteredServer()
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
			new_server.ID = hello.ServerID;
			new_server.AddAddress( address );
			new_server.max_connections = hello.Connections;
			new_server.pending_connections = 0;
			new_server.active_connections = 0;
			new_server.Port = hello.Port;
			this.Add( new_server );
			return new_server;
		}
	}
}
