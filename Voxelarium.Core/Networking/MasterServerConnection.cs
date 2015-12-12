using ProtoBuf;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Voxelarium.Common;
using Voxelarium.Core.Game;
using Voxelarium.Protocol;

namespace Voxelarium.Core.Networking
{
	class MasterServerConnection
	{
		Thread connect_thread;
		DateTime begin_connect_time;
		int total_time = 10000;
		bool connected;
		bool connecting; // initial connecting state.. haven't started doing connections yet
		static byte[] ping_reply_message = { 4, 0, 0, 0, (byte)Protocol.Message.PingReply, 0, 0, 0 };

		int v4Address_attempt;
		List<IPAddress> v4Addresses = new List<IPAddress>();
		TcpClient v4Client;
		Timer v4TimeoutTimer;

		int v6Address_attempt;
		List<IPAddress> v6Addresses = new List<IPAddress>();
		TcpClient v6Client;
		Timer v6TimeoutTimer;
		MemoryStream buffer;
		MemoryStream send_buffer;

		Socket socket;

		List<Server> ServerList = new List<Server>();

		enum ReadState
		{
			readLength, readData
		};
		ReadState state;

		internal int PercentToFail
		{
			get
			{
				TimeSpan ts = ( DateTime.Now - begin_connect_time );
				int time = ts.Seconds * 1000 + ts.Milliseconds;
				if( !connecting && v6TimeoutTimer == null && v4TimeoutTimer == null )
				{
					Log.log( "Both timers have expired..." );
					total_time = time;
				}
				if( time > total_time )
					time = total_time; // keep at 100%
				return ( time * 100 ) / total_time;
			}
		}
		internal bool Connected
		{
			get
			{
				return connected;
			}
		}


		bool NewV4Connect()
		{
			if( v4Address_attempt < v4Addresses.Count )
			{
				TcpClient client = new TcpClient();
				try
				{
					if( client == null )
					{
						Log.log( "Failed to make v4 client..." );
						return false;
					}
					v4Client = client;
					client.BeginConnect( v4Addresses[v4Address_attempt++], Settings_Hardware.MasterServerPort, ConnectionComplete, client );
				}
				catch( SocketException )
				{
					client.Close();
					v4Client = null;
					Log.log( "Failed to begin connecting." );
				}
				return true;
			}
			return false;
		}

		bool NewV6Connect()
		{
			if( v6Address_attempt < v6Addresses.Count )
			{
				TcpClient client = new TcpClient( AddressFamily.InterNetworkV6 );
				try
				{
					if( client == null )
					{
						Log.log( "Failed to make v6 client..." );
						return false;
					}
					//client.Client.SetSocketOption( SocketOptionLevel.Socket, SocketOptionName.
					v6Client = client;
					client.BeginConnect( v6Addresses[v6Address_attempt++], Settings_Hardware.MasterServerPort, ConnectionComplete, client );
					return true;
				}
				catch( SocketException )
				{
					client.Close();
					v6Client = null;
					Log.log( "Failed to begin connecting." );
				}
			}
			return false;
		}

		void RequestServers( int start )
		{
			if( start == 0 )
				ServerList.Clear();
			send_buffer.SetLength( 8 );
			send_buffer.Seek( 8, SeekOrigin.Begin );
			ListServers request_list = new ListServers();
			request_list.start_offset = 0;
			Serializer.Serialize( send_buffer, request_list );
			byte[] len = BitConverter.GetBytes( (int)( send_buffer.Length - 4) );
			byte[] msgId = BitConverter.GetBytes( (int)Protocol.Message.ListServers );
			byte[] sendbuf = send_buffer.GetBuffer();
			for( int n = 0; n < 4; n++ ) sendbuf[n] = len[n];
			for( int n = 0; n < 4; n++ ) sendbuf[4 + n] = msgId[n];
			socket.Send( sendbuf, (int)send_buffer.Length, SocketFlags.None );
		}

		void ReadComplete( IAsyncResult iar )
		{
			int toread = 4;
			try
			{
				socket.EndReceive( iar );
			}
			catch( ObjectDisposedException )
			{
				Log.log( "Already closed and disposed. {0} ", socket.GetHashCode(), 0 );
				return;
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
					socket.Send( ping_reply_message, 8, SocketFlags.None );
					break;
				case Message.Servers:
					Protocol.Server[] server_list = Serializer.Deserialize<Protocol.Server[]>( buffer );
					if( server_list.Length > 0 )
					{
						ServerList.AddRange( server_list );
						if( server_list.Length < 10 )
						{
							RequestServers( ServerList.Count );
						}
					}
					else
					{
						if( ServerList.Count == 0 )
						{
							Server NoServer = new Server();
							NoServer.ServerName = "No Servers";
							ServerList.Add( new Server() );
						}
					}
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

		void ConnectionComplete( IAsyncResult iar )
		{
			TcpClient client = iar.AsyncState as TcpClient;
			if( client.Client == null )
			{
				// closed by external forces.
				// v6 .close will set socket=NULL
				// v4 .close will .Dispose itself.
				client.Close();
				return;
			}
			try
			{
				client.EndConnect( iar );
			}
			catch( Exception e )
			{
				Log.log( "failed to connect: {0} {1} {2}", e.Message, client.GetHashCode()
								, client.Client==null?0:client.Client.GetHashCode() );
				if( client == v4Client )
				{
					client.Close();
					v4Client = null;
					if( NewV4Connect() )
						v4TimeoutTimer.Change( 4000, 0 );
					else
					{
						v4TimeoutTimer.Dispose();
						v4TimeoutTimer = null;
					}
					return;
				}
				if( client == v6Client )
				{
					client.Close();
					v6Client = null;
					if( NewV6Connect() )
						v6TimeoutTimer.Change( 4000, 0 );
					else
					{
						v6TimeoutTimer.Dispose();
						v6TimeoutTimer = null;
					}
					return;
				}
			}
			if( client == v4Client )
				v4TimeoutTimer.Dispose();
			if( client == v6Client )
				v6TimeoutTimer.Dispose();
			if( socket != null )
			{
				// nevermind, already had a good connection.
				Log.log( "Closing socket {0} {1}", client.GetHashCode(), client.Client.GetHashCode(), 0 );
				client.Close();
				return;
			}

			connected = true;
			Log.log( "Setting 'socket' to {0} {1}", client.GetHashCode(), client.Client.GetHashCode() );
			socket = client.Client;

			buffer = new MemoryStream( 4096 );
			send_buffer = new MemoryStream( 4096 );
			state = ReadState.readLength;
			client.Client.BeginReceive( buffer.GetBuffer(), 0, 4, SocketFlags.None, ReadComplete, null );

			RequestServers( 0 );
		}

		void v4ConnectionTimeout( object unused )
		{
			if( v4Client != null )
			{
				Log.log( "V4 Timeout Closing socket {0} {1}", v4Client.GetHashCode(), v4Client.Client.GetHashCode(), 0 );
				v4Client.Close();
			}
			if( NewV4Connect() )
				v4TimeoutTimer.Change( 4000, 0 );
			else
				v4TimeoutTimer.Dispose();
		}

		void v6ConnectionTimeout( object unused )
		{
			if( v6Client != null )
			{
				Log.log( "V6 Timeout Closing socket {0} {1}", v6Client.GetHashCode(), v6Client.Client.GetHashCode(), 0 );
				v6Client.Close();
			}
			if( NewV6Connect() )
				v6TimeoutTimer.Change( 4000, 0 );
			else
				v6TimeoutTimer.Dispose();
		}

		void ConnectionThread()
		{
			begin_connect_time = DateTime.Now;
			try
			{
				IPHostEntry he = Dns.GetHostEntry( Settings_Hardware.MasterServerHostname );
				IPAddress[] addresses = he.AddressList;
				if( addresses.Length > 0 )
				{
					total_time = addresses.Length * 4000;
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
					if( NewV4Connect() )
						v4TimeoutTimer = new Timer( v4ConnectionTimeout, null, 4000, Timeout.Infinite );
					if( NewV6Connect() )
						v6TimeoutTimer = new Timer( v6ConnectionTimeout, null, 4000, Timeout.Infinite );
				}
			}
			catch( SocketException )
			{
				Log.log( "Bad Address {0}", Settings_Hardware.MasterServerHostname );
			}
			connecting = false;
		}

		internal MasterServerConnection()
		{
			connect_thread = new Thread( ConnectionThread );
			connecting = true;
			connect_thread.Start();
		}

	}
}
