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
		internal static string ServerName;
		internal static int MaxConnections;
		static Voxelarium.Core.VoxelGameEnvironment game;

		TcpListener listener;
		TcpListener listener_v6;

		static byte[] ping_message = { 4, 0, 0, 0, (byte)Protocol.Message.Ping, 0, 0, 0 };
		static byte[] ping_reply_message = { 4, 0, 0, 0, (byte)Protocol.Message.PingReply, 0, 0, 0 };

		static List<CommState> clients = new List<CommState>();

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
			internal Timer idle_tick;
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

			public void Dispose(){
				GameServer.clients.Remove( this );
				socket.Close();
				idle_tick.Dispose();
			}

			internal void Received( IAsyncResult result )
			{
				Socket socket = result.AsyncState as Socket;
				int bytes = 0;
				try
				{
					bytes = socket.EndReceive( result );
					if( bytes == 0 )
					{
						Dispose();
						return;
					}
				}
				catch( Exception e )
				{
					Log.log( "Socket Closed? {0}", e.Message );
					Dispose();
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
			ServerName = Settings.Read( "Server Name", "Change Me" );
			MaxConnections = Settings.Read( "Max Connections", 16 );

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
			clients.Add( commState );
			// accept another connection
			listener.BeginAcceptTcpClient( Accept, listener );
		}



		internal GameServer()
		{
			Program.AtExit += Program_AtExit;;;
			byte[] raw_ipv6 = { 0,0,0,0
				,0, 0, 0, 0
				,0, 0, 0, 0
				,0, 0, 0, 0 };
			byte[] raw_ipv4 = { 0, 0, 0, 0 };
			byte[] val = BitConverter.GetBytes( (Int32)1 );

			IPAddress addr = new IPAddress( raw_ipv4 );
			listener = new TcpListener( addr, Settings.Read( "Server Port", GameServer.serving_port ) );
			Log.log( "Reuse is {0}", listener.Server.GetSocketOption( SocketOptionLevel.Socket, SocketOptionName.ReuseAddress ), 0 );
			listener.Server.SetSocketOption( SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, val );
			//listener.Server.SetSocketOption( SocketOptionLevel.Socket, (SocketOptionName)15/*SocketOptionName.ReusePort*/, val );
 			Log.log( "Reuse is {0}", listener.Server.GetSocketOption( SocketOptionLevel.Socket, SocketOptionName.ReuseAddress ), 0 );
			listener.Start();

			IPAddress addr_v6 = new IPAddress( raw_ipv6 );
			listener_v6 = new TcpListener( addr_v6, Settings.Read( "Server Port", GameServer.serving_port ) );
			Log.log( "Reuse is {0}", listener_v6.Server.GetSocketOption( SocketOptionLevel.Socket, SocketOptionName.ReuseAddress ), 0 );
			listener_v6.Server.SetSocketOption( SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, val );
			//istener_v6.Server.SetSocketOption( SocketOptionLevel.Socket, (SocketOptionName)15/*SocketOptionName.ReusePort*/, val );
			Log.log( "Reuse is {0}", listener_v6.Server.GetSocketOption( SocketOptionLevel.Socket, SocketOptionName.ReuseAddress ), 0 );
			listener_v6.Start();

			listener.BeginAcceptSocket( Accept, listener );
			listener_v6.BeginAcceptSocket( Accept, listener_v6 );
		}

		void Program_AtExit ()
		{
			foreach( CommState client in clients ) {
				client.Dispose();
			}
			clients.Clear();
		}
	}
}
