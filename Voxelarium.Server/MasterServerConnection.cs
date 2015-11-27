using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Voxelarium.Common;

namespace Voxelarium.Server
{
	internal class MasterServerConnection
	{
		int v4Address_attempt;
		List<IPAddress> v4Addresses = new List<IPAddress>();
		TcpClient v4Client;

		int v6Address_attempt;
		List<IPAddress> v6Addresses = new List<IPAddress>();
		TcpClient v6Client;

		int host_port;

		int connect_attempts;
		int connections;

		bool Connected
		{
			get
			{
				return connections > 0;
			}
		}


		void ConnectionComplete( IAsyncResult iar )
		{
			TcpClient client = iar.AsyncState as TcpClient;
			try
			{
				client.EndConnect( iar );
			}
			catch( Exception e )
			{
				Log.log( "failed to connect: {0}", e.Message );
				if( client == v4Client )
				{
					if( v4Address_attempt == v4Addresses.Count )
					{
						v4Address_attempt = 0;
					}
					client = new TcpClient();
				}
			}
			//iar.AsyncState

		}

		internal MasterServerConnection()
		{
			string host_address = Settings.Read( "Master Server Address", "d3x0r.org" );
			host_port = Settings.Read( "Master Server Port", 31732 );
			IPHostEntry he = Dns.GetHostEntry( host_address );
			IPAddress[] addresses = he.AddressList;
			foreach( IPAddress ip in addresses )
			{
				if( ip.AddressFamily == System.Net.Sockets.AddressFamily.InterNetworkV6 )
				{
					if( v6Client != null )
					{
						v6Client = new TcpClient();
						v6Client.BeginConnect( ip, host_port, ConnectionComplete, v6Client );
					}
					else
						v6Addresses.Add( ip );
				}
				if( ip.AddressFamily == AddressFamily.InterNetwork )
				{
					if( v4Client != null )
					{
						v4Client = new TcpClient();
						v4Client.BeginConnect( ip, host_port, ConnectionComplete, v4Client );
					}
					else
						v4Addresses.Add( ip );
				}
			}
		}
	}
}
