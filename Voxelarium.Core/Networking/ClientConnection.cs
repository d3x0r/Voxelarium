using ProtoBuf;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Voxelarium.Common;
using Voxelarium.Core.Game;
using Voxelarium.Core.Voxels.IO;

namespace Voxelarium.Core.Networking
{
	internal class ClientConnection: IDisposable
	{
		Socket socket;
		TcpClient client;
		Timer connection_timeout;
		AutoResetEvent sleep_event;
		List<EndPoint> connect_from = new List<EndPoint>();
		MemoryStream buffer;// = new MemoryStream( 2048 );
		bool failed;
		bool disconnected;

		enum ReadState
		{
			readLength, readData
		};
		ReadState state;


		void ConnectTimeout(object state)
		{
			client.Close();
		}

		public ClientConnection()
		{
			connection_timeout = new Timer( ConnectTimeout );
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
				disconnected = true;
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
				default:
					Log.log( "Received unhandled message: {0}", msgId );
					break;
					//case Protocol.Message.
				}
				break;
			}
			buffer.SetLength( toread );
			socket.BeginReceive( buffer.GetBuffer(), 0, toread, SocketFlags.None, ReadComplete, null );
		}

		void Connected( IAsyncResult iar )
		{
			try
			{
				client.EndConnect( iar );
				connect_from.Add( client.Client.LocalEndPoint );
			}
			catch( Exception e )
			{
				failed = true;
				sleep_event.Set();
				return;
			}
			socket = client.Client;
			buffer = new MemoryStream( 4096 );


			buffer.SetLength( 8 );
			buffer.Seek( 8, SeekOrigin.Begin );
			Protocol.ClientHello hello = new Protocol.ClientHello();
			hello.ClientID = Settings_Hardware.ClientID;
			Serializer.Serialize( buffer, hello );
			byte[] len = BitConverter.GetBytes( buffer.Length - 4 );
			byte[] msgId = BitConverter.GetBytes( (int)Protocol.Message.ClientHello );
			byte[] sendbuf = buffer.GetBuffer();
			for( int n = 0; n < 4; n++ ) sendbuf[n] = len[n];
			for( int n = 0; n < 4; n++ ) sendbuf[4 + n] = msgId[n];
			socket.Send( sendbuf, (int)buffer.Length, SocketFlags.None );


			socket.BeginReceive( buffer.GetBuffer(), 0, 4, SocketFlags.None, ReadComplete, null );
		}

		internal void ConnectToGameServer( Protocol.GameServer server )
		{
			foreach( byte[] addr in server.addresses )
			{
				IPAddress address = new IPAddress( addr );
				IPEndPoint connect_to = new IPEndPoint( address, server.Port );
				client = new TcpClient();
				client.BeginConnect( address, server.Port, Connected, null );
				connection_timeout.Change( 1000, Timeout.Infinite );
				sleep_event.WaitOne();
				if( !failed )
					break;
			}
		}

		#region IDisposable Support
		private bool disposedValue = false; // To detect redundant calls

		protected virtual void Dispose( bool disposing )
		{
			if( !disposedValue )
			{
				if( disposing )
				{
					// TODO: dispose managed state (managed objects).
					connection_timeout.Dispose();
				}

				// TODO: free unmanaged resources (unmanaged objects) and override a finalizer below.
				// TODO: set large fields to null.

				disposedValue = true;
			}
		}

		// TODO: override a finalizer only if Dispose(bool disposing) above has code to free unmanaged resources.
		// ~ClientConnection() {
		//   // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
		//   Dispose(false);
		// }

		// This code added to correctly implement the disposable pattern.
		public void Dispose()
		{
			// Do not change this code. Put cleanup code in Dispose(bool disposing) above.
			Dispose( true );
			// TODO: uncomment the following line if the finalizer is overridden above.
			// GC.SuppressFinalize(this);
		}
		#endregion
	}
}
