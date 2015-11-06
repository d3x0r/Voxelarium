using System;
using System.Collections.Generic;
using System.Text;

namespace Bullet.UnitTests
{
	class Program
	{
		static bool FindArg( string[] args, string arg )
		{
			foreach( string test in args )
				if( String.Compare( arg, test, true ) == 0 )
					return true;
			return false;
		}

		static void Main( string[] args )
		{
			if( args.Length == 0 || FindArg( args, "math" ) )
				MathTest_001.Run();
			if( args.Length == 0 || FindArg( args, "1" ) )
				Test_001.Run( FindArg( args, "1s" ) );
			if( args.Length == 0 || FindArg( args, "2" ) )
				Test_002.Run();
			if( args.Length == 0 || FindArg( args, "3" ) )
				Test_003.Run();
			if( args.Length == 0 || FindArg( args, "4" ) )
				Test_004.Run();
			Console.Read();
		}
	}
}
