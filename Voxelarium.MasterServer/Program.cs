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
