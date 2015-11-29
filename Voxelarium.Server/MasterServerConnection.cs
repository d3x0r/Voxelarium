using ProtoBuf;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Voxelarium.Common;
using Voxelarium.Protocol;

namespace Voxelarium.Server
{
	internal class MasterServerConnector
	{
		static byte[] ping_reply_message = { 4, 0, 0, 0, (byte)Protocol.Message.PingReply, 0, 0, 0 };

		int v4Address_attempt;
		List<IPAddress> v4Addresses = new List<IPAddress>();
		TcpClient v4Client;

		int v6Address_attempt;
		List<IPAddress> v6Addresses = new List<IPAddress>();
		TcpClient v6Client;
		MemoryStream buffer = new MemoryStream( 2048 );
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

		internal class MasterServerConnection
		{
			TcpClient tcpClient;
			Socket socket;
			enum ReadState
			{
				readLength, readData
			};
			ReadState state;
			MemoryStream buffer = new MemoryStream( 2048 );

			internal MasterServerConnection( TcpClient client)
			{
				this.tcpClient = client;
				socket = client.Client;
				state = ReadState.readLength;

				client.Client.BeginReceive( buffer.GetBuffer(), 0, 4, SocketFlags.None, ReadComplete, client );
			}

			void ReadComplete( IAsyncResult iar )
			{
				int toread = 4;
				int bytes = 0;
				try
				{
					bytes = socket.EndReceive( iar );
				}
				catch( Exception e )
				{
					Log.log( "Disconnected: {0}", e.Message );
					socket.Close();
					return;
				}
				switch( state )
				{
				case ReadState.readLength:
					toread = BitConverter.ToInt32( buffer.GetBuffer(), 0 );
					state = ReadState.readData;
					break;
				case ReadState.readData:
					state = ReadState.readLength;
					Protocol.Message msgId = (Protocol.Message)BitConverter.ToInt32( buffer.GetBuffer(), 0 );
					switch( msgId )
					{
					case Message.Ping:
						Log.log( "Received Ping." );
						socket.Send( ping_reply_message, 8, SocketFlags.None );
						break;
					default:
						Log.log( "Received unhandled message: {0}", msgId );
						break;
					}
					break;
				}
				buffer.SetLength( toread );
				socket.BeginReceive( buffer.GetBuffer(), 0, toread, SocketFlags.None, ReadComplete, null );

			}
		}


		void NewV4Connect()
		{
			if( v4Address_attempt < v4Addresses.Count )
			{
				TcpClient client = new TcpClient();
				client.BeginConnect( v4Addresses[v4Address_attempt++], host_port, ConnectionComplete, client );
			}
		}

		void NewV6Connect()
		{
			if( v6Address_attempt < v6Addresses.Count )
			{
				TcpClient client = new TcpClient( AddressFamily.InterNetworkV6 );
				//client.Client.SetSocketOption( SocketOptionLevel.Socket, SocketOptionName.
				client.BeginConnect( v6Addresses[v6Address_attempt++], host_port, ConnectionComplete, client );
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
					NewV4Connect();
					return;
				}
				if( client == v6Client )
				{
					NewV6Connect();
					return;
				}
			}

			buffer.SetLength( 8 );
			buffer.Seek( 8, SeekOrigin.Begin );
			ServerHello hello = new ServerHello();
			hello.ServerName = Settings.Read( "Server Name", "Change Me" );
			hello.Connections = Settings.Read( "Max Connections", 16 );
			hello.ServerID = GameServer.ServerID;
			Serializer.Serialize( buffer, hello );
			byte[] len = BitConverter.GetBytes( buffer.Length - 4 );
			byte[] msgId = BitConverter.GetBytes( (int)Protocol.Message.ServerHello );
			byte[] sendbuf = buffer.GetBuffer();
			for( int n = 0; n < 4; n++ ) sendbuf[n] = len[n];
			for( int n = 0; n < 4; n++ ) sendbuf[4+n] = msgId[n];
			client.Client.Send( sendbuf, (int)buffer.Length, SocketFlags.None );
			new MasterServerConnection( client );
		}

		internal MasterServerConnector()
		{
			string host_address = Settings.Read( "Master Server Address", "d3x0r.org" );
			host_port = Settings.Read( "Master Server Port", 31732 );
			IPHostEntry he = Dns.GetHostEntry( host_address );
			IPAddress[] addresses = he.AddressList;
			foreach( IPAddress ip in addresses )
			{
				if( ip.AddressFamily == System.Net.Sockets.AddressFamily.InterNetworkV6 )
				{
					v6Addresses.Add( ip );
				}
				if( ip.AddressFamily == AddressFamily.InterNetwork )
				{
					v4Addresses.Add( ip );
				}
			}
			NewV4Connect();
			NewV6Connect();
		}
	}
}
