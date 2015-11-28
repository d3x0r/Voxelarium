using ProtoBuf;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Voxelarium.Common;

namespace Voxelarium.MasterServer
{
	internal class RegistryServer
	{
		TcpListener listener;
		TcpListener listener_v6;
		static RegisteredServers servers = new RegisteredServers();
		static byte[] ping_message = { 4, 0, 0, 0, (byte)Protocol.Message.Ping, 0, 0, 0 };


		class CommState
		{
			enum ReadState {
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

			RegisteredServer registered_server;
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
					registered_server.RemoveAddress( host_address );
					if( registered_server.Addresses.Count == 0 )
					{
						servers.Remove( registered_server );
					}
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
					case Protocol.Message.ServerHello:
						Protocol.ServerHello msg = Serializer.Deserialize<Protocol.ServerHello>( buffer );
						registered_server = servers.AddServer( msg.ServerName, msg.Connections, host_address );
						break;
					case Protocol.Message.ListServers:
						Protocol.ListServers listcmd = Serializer.Deserialize<Protocol.ListServers>( buffer );
						int n;
						int count = 0;
						buffer.Seek( 8, SeekOrigin.Begin );
						for( n = listcmd.start_offset; count < 10 && n < servers.Count; n++ )
						{
							Serializer.Serialize<RegisteredServer>( buffer, servers[n] );
						}
						byte[] output = buffer.GetBuffer();
						byte[] msg_len = BitConverter.GetBytes( buffer.Length - 4 );
						msg_id = BitConverter.GetBytes( (int)Protocol.Message.Servers );
						for( n = 0; n < 4; n++ )
							output[n] = msg_len[n];
						for( n = 0; n < 4; n++ )
							output[4+n] = msg_id[n];

						socket.Send( buffer.GetBuffer(), (int)buffer.Length, SocketFlags.None );
						break;
					}
					break;
				}
				buffer.Capacity = toread; // make sure it's big enough
				socket.BeginReceive( buffer.GetBuffer(), 0, toread, SocketFlags.None, Received, this );
			}

		}



		void Accept( IAsyncResult result )
		{
			TcpListener listener = result.AsyncState as TcpListener;
			CommState commState = new CommState();

			commState.socket = listener.EndAcceptSocket( result );
			commState.connected_from = commState.socket.RemoteEndPoint;

            commState.socket.BeginReceive( commState.buffer.GetBuffer(), 0, 4, SocketFlags.None
						, commState.Received, commState );

			listener.BeginAcceptTcpClient( Accept, listener );
		}

		internal RegistryServer()
		{
			byte[] raw_ipv4 = { 0, 0, 0, 0 };
			IPAddress addr = new IPAddress( raw_ipv4 );
			listener = new TcpListener( addr, Settings.Read( "Server Port", 31732 ) );
			listener.Start();
			byte[] raw_ipv6 = { 0,0,0,0
					,0, 0, 0, 0
					,0, 0, 0, 0
					,0, 0, 0, 0 };
			IPAddress addr_v6 = new IPAddress( raw_ipv6 );
			listener_v6 = new TcpListener( addr_v6, Settings.Read( "Server Port", 31732 ) );
			listener_v6.Start();

			listener.BeginAcceptSocket( Accept, listener );
			listener_v6.BeginAcceptSocket( Accept, listener_v6 );
		}
	}
}
