using ProtoBuf;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Voxelarium.Common;
using Voxelarium.Protocol;
using Voxelarium.Core.UI;

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
		int host_port;
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
			MemoryStream buffer = new MemoryStream( 32 );
			MemoryStream send_buffer = new MemoryStream( 32 );

			internal MasterServerConnection( TcpClient client)
			{
				this.tcpClient = client;
				socket = client.Client;
				state = ReadState.readLength;
				socket.NoDelay = true;
				buffer.SetLength( 4 ); // makes ure buffer has a buffer
				socket.BeginReceive( buffer.GetBuffer(), 0, 4, SocketFlags.None, ReadComplete, client );
				SendHello();
			}

			void SendHello()
			{
				send_buffer.SetLength( 8 );
				send_buffer.Seek( 8, SeekOrigin.Begin );
				ServerHello hello = new ServerHello();
				hello.ServerName = GameServer.ServerName;
				hello.Connections = GameServer.MaxConnections;
				hello.ServerID = GameServer.ServerID;
				//Log.log( "Sending Hello {0} {1}", GameServer.ServerID, GameServer.ServerName );
				Serializer.Serialize( send_buffer, hello );
				byte[] len = BitConverter.GetBytes( (int)(send_buffer.Length - 4) );
				byte[] msgId = BitConverter.GetBytes( (int)Protocol.Message.ServerHello );
				byte[] sendbuf = send_buffer.GetBuffer();
				for( int n = 0; n < 4; n++ ) sendbuf[n] = len[n];
				for( int n = 0; n < 4; n++ ) sendbuf[4+n] = msgId[n];
				//Log.log( "Sending Server Hello {0}", send_buffer.Length );
				socket.Send( sendbuf, (int)send_buffer.Length, SocketFlags.None );
			}

			void ConnectToClient( Protocol.ConnectToClient client )
			{
				TcpClient connection = new TcpClient();
				client.SetupAddresses();

				//IPEndPoint local = new IPEndPoint( client.AddressV4
				//connection.Client.Bind( 
			}

			void ReadComplete( IAsyncResult iar )
			{
				int toread = 4;
				int bytes = 0;
				try
				{
					bytes = socket.EndReceive( iar );
					if( bytes == 0 )
					{
						// socket is closed.
						socket.Close();
						Program.Exit();
						return;
					}
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
					//Log.log( "Received length of {0}", toread, 0 );
					state = ReadState.readData;
					break;
				case ReadState.readData:
					state = ReadState.readLength;
					Protocol.Message msgId = (Protocol.Message)BitConverter.ToInt32( buffer.GetBuffer(), 0 );
					switch( msgId )
					{
					case Message.Ping:
						//Log.log( "Received Ping." );
						socket.Send( ping_reply_message, 8, SocketFlags.None );
						break;
					case Protocol.Message.ConnectToClient:
						Protocol.ConnectToClient client = Serializer.Deserialize<Protocol.ConnectToClient>( buffer );
						ConnectToClient( client );
						break;
					default:
						Log.log( "Received unhandled message: {0}", msgId );
						break;
					}
					break;
				}
				buffer.Position = 0;
				buffer.SetLength( toread );
				socket.BeginReceive( buffer.GetBuffer(), 0, toread, SocketFlags.None, ReadComplete, null );

			}
		}


		bool NewV4Connect()
		{
			if( v4Address_attempt < v4Addresses.Count )
			{
				TcpClient client = new TcpClient();
				v4Client = client;
				client.BeginConnect( v4Addresses[v4Address_attempt++], host_port, ConnectionComplete, client );
				return true;
			}
			return false;
		}

		bool NewV6Connect()
		{
			if( v6Address_attempt < v6Addresses.Count )
			{
				TcpClient client = new TcpClient( AddressFamily.InterNetworkV6 );
				//client.Client.SetSocketOption( SocketOptionLevel.Socket, SocketOptionName.
				v6Client = client;
				client.BeginConnect( v6Addresses[v6Address_attempt++], host_port, ConnectionComplete, client );
				return true;
			}
			return false;
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
					v4Client.Close();
					v4Client = null;
					if( !NewV4Connect() ) {
						if( v6Client == null )
						{
							Log.log( "Master Server is not responding; no more connections to try" );
							Program.Exit();
						}
					}
					return;
				}
				if( client == v6Client )
				{
					Log.log( "V6 failed... {0}", e.Message );
					v6Client.Close();
					v6Client = null;
					if( !NewV6Connect() ){
						if( v4Client == null )
						{
							Log.log( "Master Server is not responding; no more connections to try" );
							Program.Exit();
						}
					}
					return;
				}
			}
			connections++;
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
