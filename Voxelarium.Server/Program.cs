/*
 * This file is part of Voxelarium.
 *
 * Copyright 2015-2016 James Buckeyne  
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
 *
 * Created 2015/12/01 d3x0r
*/
	using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using Voxelarium.Common;
using Voxelarium.Core.UI;

namespace Voxelarium.Server
{
	class Program
	{
		static AutoResetEvent sleep_event;
		static bool done;

		internal static GameServer game_server;
		internal static MasterServerConnector master;
		internal delegate void SimpleAtExit();
		internal static event SimpleAtExit AtExit;

		internal static void Exit()
		{
			if( AtExit != null )
				AtExit();
			done = true;
			sleep_event.Set();
		}


		static void Main( string[] args )
		{
			Log.log( "Game Server Starting.." );
			game_server = new GameServer();
			master = new MasterServerConnector();
			sleep_event = new AutoResetEvent( false );
			while( !done )
				sleep_event.WaitOne();
			// Yes, technically there is no display; but existing threads registered shutdown event
			// with AtExit event list in Display; so use that to gracefully exit.
			Display.Shutdown();
		}
	}
}
