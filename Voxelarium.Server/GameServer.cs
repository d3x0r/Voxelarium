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

namespace Voxelarium.Server
{
	internal class GameServer
	{
		internal static int serving_port;
		internal static Guid ServerID;
		static Voxelarium.Core.VoxelGameEnvironment game;

		TcpListener listener;
		TcpListener listener_v6;

		static byte[] ping_message = { 4, 0, 0, 0, (byte)Protocol.Message.Ping, 0, 0, 0 };
		static byte[] ping_reply_message = { 4, 0, 0, 0, (byte)Protocol.Message.PingReply, 0, 0, 0 };

		static internal List<Protocol.Server> servers;

		class CommState
		{
			enum ReadState
			{
				getLength, getData
			}
			ReadState readstate;
			internal Socket socket;
			EndPoint _connected_from;
			internal EndPoint connected_from
			{
				set
				{
					_connected_from = value;
					host_address = ( (IPEndPoint)value ).Address;
				}
			}

			//RegisteredServer registered_server;
			internal IPAddress host_address;
			internal MemoryStream buffer;
			int toread;
			DateTime last_receive;
			Timer idle_tick;
			void CheckIdle( object state )
			{
				DateTime now = DateTime.Now;
				if( ( now - last_receive ) > new TimeSpan( 0, 1, 0 ) ) // one minute
				{
					socket.Send( ping_message, 8, SocketFlags.None );
				}

			}

			internal CommState()
			{
				idle_tick = new Timer( CheckIdle, null, 30000, 30000 );
				readstate = ReadState.getLength;
				buffer = new MemoryStream( 1024 );
			}



			internal void Received( IAsyncResult result )
			{
				Socket socket = result.AsyncState as Socket;
				int bytes = 0;
				try
				{
					bytes = socket.EndReceive( result );
				}
				catch( Exception e )
				{
					Log.log( "Socket Closed? {0}", e.Message );
					// disconnected.
					return;
				}
				switch( readstate )
				{
				case ReadState.getLength:
					toread = BitConverter.ToInt32( buffer.GetBuffer(), 0 );
					break;
				case ReadState.getData:
					readstate = ReadState.getLength;
					toread = 4;
					byte[] msg_id = new byte[4];
					buffer.Read( msg_id, 0, 4 );  // use 4 bytes before deserializing content.
					last_receive = DateTime.Now;

					Protocol.Message message = (Protocol.Message)BitConverter.ToInt32( msg_id, 0 );
					switch( message )
					{
					case Protocol.Message.PingReply:
						// nothing to do; no content
						break;
					case Protocol.Message.Ping:
						socket.Send( ping_reply_message, 8, SocketFlags.None );
						break;
					case Protocol.Message.Servers:
						Protocol.Server[] list = Serializer.Deserialize<Protocol.Server[]>( buffer );
						int n;
						for( n = 0; n < list.Length; n++ )
						{
							servers.Add( list[n] );
						}
						buffer.SetLength( 12 );
						byte[] output = buffer.GetBuffer();
						byte[] msg_len = BitConverter.GetBytes( buffer.Length - 4 );
						byte[] next_block = BitConverter.GetBytes( servers.Count );
						msg_id = BitConverter.GetBytes( (int)Protocol.Message.ListServers );
						for( n = 0; n < 4; n++ )
							output[n] = msg_len[n];
						for( n = 0; n < 4; n++ )
							output[4 + n] = msg_id[n];

						socket.Send( buffer.GetBuffer(), (int)buffer.Length, SocketFlags.None );
						break;
					}
					break;
				}
				//buffer.Capacity = toread; // make sure it's big enough
				buffer.SetLength( toread );
				socket.BeginReceive( buffer.GetBuffer(), 0, toread, SocketFlags.None, Received, this );
			}

		}


		static GameServer()
		{
			ServerID = Settings.Read( "Server ID", Guid.NewGuid() );
			serving_port = Settings.Read( "Serve on Port", 31733 );
			game = new Core.VoxelGameEnvironment();
			game.UniverseNum = Settings.Read( "Game Universe", 1 );
			game.Init( true );
			game.Start_Game( true );
		}


		void Accept( IAsyncResult result )
		{
			TcpListener listener = result.AsyncState as TcpListener;
			CommState commState = new CommState();

			commState.socket = listener.EndAcceptSocket( result );
			commState.connected_from = commState.socket.RemoteEndPoint;

			commState.socket.BeginReceive( commState.buffer.GetBuffer(), 0, 4, SocketFlags.None
						, commState.Received, commState );

			// accept another connection
			listener.BeginAcceptTcpClient( Accept, listener );
		}

		internal GameServer()
		{
			byte[] raw_ipv4 = { 0, 0, 0, 0 };
			IPAddress addr = new IPAddress( raw_ipv4 );
			listener = new TcpListener( addr, Settings.Read( "Server Port", GameServer.serving_port ) );
			listener.Start();
			byte[] raw_ipv6 = { 0,0,0,0
					,0, 0, 0, 0
					,0, 0, 0, 0
					,0, 0, 0, 0 };
			IPAddress addr_v6 = new IPAddress( raw_ipv6 );
			listener_v6 = new TcpListener( addr_v6, Settings.Read( "Server Port", GameServer.serving_port ) );
			listener_v6.Start();

			listener.BeginAcceptSocket( Accept, listener );
			listener_v6.BeginAcceptSocket( Accept, listener_v6 );


		}
	}
}
