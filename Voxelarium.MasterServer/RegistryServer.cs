using ProtoBuf;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;

namespace Voxelarium.MasterServer
{
	internal class RegistryServer
	{
		TcpListener listener;
		TcpListener listener_v6;

		class CommState
		{
			enum ReadState {
				getLength, getData
			}
			ReadState readstate;
			internal Socket socket;
			internal MemoryStream buffer;
			int toread;
			internal CommState()
			{
				readstate = ReadState.getLength;
				buffer = new MemoryStream( 1024 );
			}


			internal void Received( IAsyncResult result )
			{
				Socket socket = result.AsyncState as Socket;
				try
				{
					int bytes = socket.EndReceive( result );
					switch( readstate )
					{
					case ReadState.getLength:
						toread = BitConverter.ToInt32( buffer.GetBuffer(), 0 );
						break;
					case ReadState.getData:
						readstate = ReadState.getLength;
						toread = 4;
						Protocol.Message message = (Protocol.Message)BitConverter.ToInt32( buffer.GetBuffer(), 0 );
						switch( message )
						{
						case Protocol.Message.Hello:
							Protocol.Hello msg = Serializer.Deserialize<Protocol.Hello>( buffer );

							break;
						case Protocol.Message.ListServers:
							break;
						}
						break;
					}
				}
				catch( Exception e )
				{
					Log.log( "Socket Closed? " + e.Message );
					return;
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
            commState.socket.BeginReceive( commState.buffer.GetBuffer(), 0, 4, SocketFlags.None
						, commState.Received, commState );

			listener.BeginAcceptTcpClient( Accept, listener );
		}

		internal RegistryServer()
		{
			byte[] raw_ipv4 = { 0, 0, 0, 0 };
			IPAddress addr = new IPAddress( raw_ipv4 );
			listener = new TcpListener( addr, Settings.Read( "Server Port", 31732 ) );
			byte[] raw_ipv6 = { 0,0,0,0
					,0, 0, 0, 0
					,0, 0, 0, 0
					,0, 0, 0, 0 };
			IPAddress addr_v6 = new IPAddress( raw_ipv6 );
			listener_v6 = new TcpListener( addr_v6, Settings.Read( "Server Port", 31732 ) );

			listener.BeginAcceptSocket( Accept, listener );
			listener_v6.BeginAcceptSocket( Accept, listener_v6 );
		}
	}
}
