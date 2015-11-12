using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;

namespace Bullet.UnitTests
{
	class Program
	{
		internal static btIDebugDraw Drawer;
		internal static Bullet.Debug.OpenGL2.OpenGLDisplay Display;

		static bool FindArg( string[] args, string arg )
		{
			foreach( string test in args )
				if( String.Compare( arg, test, true ) == 0 )
					return true;
			return false;
		}

		static void RunDisplay()
		{
			Display = new Debug.OpenGL2.OpenGLDisplay();
			Drawer = Display.GetDrawer();
			Display.Run( 0, 30 );
			Environment.Exit(0);
		}

		static void Main( string[] args )
		{
			if( FindArg( args, "draw" ) )
			{
				Thread RunThread = new Thread( RunDisplay );
				RunThread.Start();
				while( Display == null )
					Thread.Sleep( 10 );
			}
			if( args.Length == 0 || FindArg( args, "math" ) )
				MathTest_001.Run();

			if( args.Length == 0 || FindArg( args, "1" ) )
				Test_001.Run( FindArg( args, "1s" ) );

			if( args.Length == 0 || FindArg( args, "2" ) )
				Test_002.Run( FindArg( args, "2s" ) );

			if( args.Length == 0 || FindArg( args, "3" ) )
				Test_003.Run( FindArg( args, "3s" ) );

			if( args.Length == 0 || FindArg( args, "4" ) )
				Test_004.Run( FindArg( args, "4s" ) );

			if( args.Length == 0 || FindArg( args, "5" ) )
				Test_005.Run( FindArg( args, "5s" ) );

			Console.Read();
		}
	}
}
