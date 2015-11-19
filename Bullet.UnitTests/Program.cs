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
			Environment.Exit( 0 );
		}

		static System.Diagnostics.Stopwatch sw;

		static void Main( string[] args )
		{
			bool timing_only = false;
			bool large_count = false;
            if( FindArg( args, "time" ) )
			{
				sw = new System.Diagnostics.Stopwatch();
				timing_only = true;
			}
			if( FindArg( args, "large" ) )
			{
				large_count = true;
			}
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
			{
				if( timing_only )
					sw.Start();
				Test_001.Run( timing_only, sw, large_count, FindArg( args, "1s" ) );
				if( timing_only )
				{
					sw.Stop();
					Console.WriteLine( "Test 1 " + sw.ElapsedMilliseconds );
					sw.Reset();
				}
			}

			if( args.Length == 0 || FindArg( args, "2" ) )
			{
				if( timing_only )
					sw.Start();
				Test_002.Run( timing_only, FindArg( args, "2s" ) );
				if( timing_only )
				{
					sw.Stop();
					Console.WriteLine( "Test 2 " + sw.ElapsedMilliseconds );
					sw.Reset();
				}
			}

			if( args.Length == 0 || FindArg( args, "3" ) )
			{
				if( timing_only )
					sw.Start();
				Test_003.Run( timing_only, FindArg( args, "3s" ), FindArg( args, "3b" ) );
				if( timing_only )
				{
					sw.Stop();
					Console.WriteLine( "Test 3 " + sw.ElapsedMilliseconds );
					sw.Reset();
				}
			}

			if( args.Length == 0 || FindArg( args, "4" ) )
			{
				if( timing_only )
					sw.Start();
				Test_004.Run( timing_only, FindArg( args, "4s" ), FindArg( args, "4b" ) );
				if( timing_only )
				{
					sw.Stop();
					Console.WriteLine( "Test 4 " + sw.ElapsedMilliseconds );
					sw.Reset();
				}
			}

			if( args.Length == 0 || FindArg( args, "5" ) )
			{
				if( timing_only )
					sw.Start();
				Test_005.Run( timing_only, FindArg( args, "5s" ) );
				if( timing_only )
				{
					sw.Stop();
					Console.WriteLine( "Test 5 " + sw.ElapsedMilliseconds );
					sw.Reset();
				}
			}

			if( args.Length == 0 || FindArg( args, "6" ) )
			{
				if( timing_only )
					sw.Start();
				Test_006.Run( timing_only, FindArg( args, "6s" ) );
				if( timing_only )
				{
					sw.Stop();
					Console.WriteLine( "Test 6 " + sw.ElapsedMilliseconds );
					sw.Reset();
				}
			}


			btScalar.LoggingFlags = DbgFlag.None;
			//Console.Read();
		}
	}
}
