using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using Voxelarium.Common;
using System.IO;
using ProtoBuf;

namespace Voxelarium.MasterServer
{
	internal class Program
	{
		static AutoResetEvent sleep_event;
		static bool done;
		static RegistryServer server;

		internal static void Exit()
		{
			done = true;
			sleep_event.Set();
		}

#if test_protobuf_include
		[ProtoContract]
		public class BaseMessage
		{
			[ProtoMember(1)]
			public int value { set; get;}
			public BaseMessage() {}
		}

		public class DerivedMessage : BaseMessage
		{
			public DerivedMessage() {}
		}

		static void Stuff()
		{
			MemoryStream ms = new MemoryStream();
			DerivedMessage dm = new DerivedMessage();
			//Protocol.RegisteredGameServer rs = new Protocol.RegisteredGameServer();
			dm.value = 1234;
			Serializer.Serialize<BaseMessage>( ms, (BaseMessage)dm );
			Console.WriteLine( "Length is {0}", ms.Length );
		}
		static void Stuff2()
		{
			MemoryStream ms = new MemoryStream();
			BaseMessage bm = new BaseMessage();
			bm.value = 1234;
			Serializer.Serialize( ms, bm );
			Console.WriteLine( "Length is {0}", ms.Length );
		}
#endif

		static void Main( string[] args )
		{
			Log.log( "Master Server Starting.." );
#if test_protobuf_include
			Stuff();
			Stuff2();
#endif
			server = new RegistryServer();
			sleep_event = new AutoResetEvent( false );
			while( !done )
				sleep_event.WaitOne();
		}
	}
}
