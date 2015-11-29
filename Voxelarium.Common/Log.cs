/*
 * This file is part of Voxelarium.
 *
 * Copyright 2015-2016 James Buckeyne  *** Added 11/22/2015
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
using System.IO;
#if !BUILD_ANDROID
using System.Windows.Forms;
#else
#endif

namespace Voxelarium.Common
{
	public static class Log
	{
//#if __MonoCS__
		public static string LoggingRoot = Environment.GetFolderPath( Environment.SpecialFolder.UserProfile );
//#else
		//public static string LoggingRoot = Environment.GetFolderPath( Environment.SpecialFolder.CommonApplicationData );
//#endif
		public static string ApplicationProducer = "Freedom Collective";
		public static string ApplicationName;
		static FileStream fs;
		static StreamWriter sw;
		static void BackupFile( String source, int namelen, int n )
		{
			String backup;
			if( System.IO.File.Exists( source ) )
				if( n < 10 )
				{
					BackupFile( ( backup = source.Substring( 0, namelen ) + n.ToString() ), namelen, n + 1 );
					System.IO.File.Move( source, backup );
				}
				else
					System.IO.File.Delete( source );

		}
		static void AutoBackup( string filename )
		{
			BackupFile( filename, filename.Length, 1 );
			//throw new Exception( "The method or operation is not implemented." );
		}
		static Log()
		{
#if !BUILD_ANDROID
			int retry = 1;
			int idx1 = Application.ExecutablePath.LastIndexOfAny( new char[] { '/', '\\' } );
			int idx2 = Application.ExecutablePath.LastIndexOfAny( new char[] { '.' } );
			if( ApplicationName == null )
				ApplicationName = Application.ExecutablePath.Substring( idx1 + 1, ( idx2 - idx1 ) - 1 );
			//return;
			string logpath = LoggingRoot
							+ "/" + ApplicationProducer + "/"
							+ ApplicationName;
			if( !Directory.Exists( logpath ) )
				Directory.CreateDirectory( logpath );
			string logname = logpath + "/" + ApplicationName + ".Log";
			retry:
			try
			{

				AutoBackup( logname );
				fs = new FileStream( logname, FileMode.Create );
				sw = new StreamWriter( fs );
			}
			catch( IOException )
			{
				logname = Application.CommonAppDataPath
							+ Application.ExecutablePath.Substring( idx1 ) + "-" + ( retry++ ) + ".Log";
				//Log.log( "In use, attempting new name..." + logname );
				goto retry;
			}
#else
			ApplicationName = "XamarinAndroidProgram";
#endif
			LogToConsole = true || Settings.Read( "Log to debug console", 0 ) != 0;
			LogToFile = false || Settings.Read( "Log to debug file", 1 ) != 0;
			LogTimeDelta = ( Settings.Read( "Log Time Delta", 1 ) != 0 );
		}
		static DateTime then;

		static bool LogToConsole = true;
		static bool LogToFile = true;
		static bool LogTimeDelta = false;

		public static void log( string s, System.Diagnostics.StackFrame sf )
		{
			String time = "";
			String file = sf.GetFileName();
			//if( file != null )
			//file = file.Substring( file.LastIndexOf( "\\" ) + 1 );
			if( LogToFile )
				lock ( sw )
				{
					if( LogTimeDelta )
					{
						TimeSpan delta = DateTime.Now.Subtract( then );
						sw.WriteLine( ( time = delta.ToString() )
							//DateTime.Now.ToString("hh.mm.ss.fff")
							+ "@" + file + "(" + sf.GetFileLineNumber() + "):" + s );
						then = DateTime.Now;
						sw.Flush();
					}
					else
					{
						sw.WriteLine( ( DateTime.Now.ToString( "yyyy-MM-dd HH:mm:ss.fff" ) )
							+ "@" + file + "(" + sf.GetFileLineNumber() + "):" + s );
						sw.Flush();
					}
				}

			if( LogToConsole )
			{
				System.Diagnostics.Debug.WriteLine(
					 //Console.WriteLine( //"{0}{1}"
					 file + "(" + sf.GetFileLineNumber() + "):"
					 + System.Threading.Thread.CurrentThread.ManagedThreadId + "|"
					+ s );
			}

		}

		/// <summary>
		/// adds the given string as an entry in the log
		/// </summary>
		/// <param name="s"></param>
		public static void log( string s )
		{
			log( s, new System.Diagnostics.StackFrame( 1, true ) );
		}
		public static void log( string s, int frame_skip )
		{
			System.Diagnostics.StackFrame sf = new System.Diagnostics.StackFrame( 1 + frame_skip, true );
			int counter = 0;
			//while( sf.GetFileLineNumber() == 0 )
			{
				counter++;
				//sf = new System.Diagnostics.StackFrame( 1 + frame_skip + counter, true );
			}
			log( s, sf );
		}
		public static void log( string s, params object[] args )
		{
			log( String.Format( s, args ), 1 );
		}
		/// <summary>
		/// Adds a hex dump of the given string to the log.
		/// </summary>
		/// <param name="s"></param>
		public static void LogBinary( string s )
		{
			string sHex = "Hex breakdown of " + s.Length.ToString() + " byte(s):", sDisp = "";
			byte charAsByte = 0;

			for( int n = 0; n < s.Length; n++ )
			{
				if( n % 16 == 0 )
				{
					sHex += "   " + sDisp;
					sDisp = "";

					sHex += "\r\n\t\t";
				}

				charAsByte = Convert.ToByte( s[n] );

				sHex += charAsByte.ToString( "X2" ) + " ";

				if( charAsByte < ' ' || charAsByte > '~' )
					sDisp += '.';
				else
					sDisp += s[n];
			}

			if( sDisp != "" )
			{
				for( int x = s.Length % 16; x < 16; x++ )
					sHex += "   ";

				sHex += "   " + sDisp;
			}

			System.Diagnostics.StackFrame sf = new System.Diagnostics.StackFrame( 1, true );
			log( sHex, sf );
		}

		/// <summary>
		/// Adds a hex dump of the given byte array to the log.
		/// </summary>
		/// <param name="byteArray"></param>
		public static void LogBinary( byte[] byteArray )
		{
			string sHex = "Hex breakdown of " + byteArray.GetLength( 0 ).ToString() + " byte(s):", sDisp = "";

			for( int n = 0; n < byteArray.GetLength( 0 ); n++ )
			{
				if( n % 16 == 0 )
				{
					sHex += "   " + sDisp;
					sDisp = "";

					sHex += "\r\n\t\t";
				}

				sHex += byteArray[n].ToString( "X2" ) + " ";

				if( byteArray[n] < ' ' || byteArray[n] > '~' )
					sDisp += '.';
				else
					sDisp += (char)byteArray[n];
			}

			if( sDisp != "" )
			{
				for( int x = byteArray.GetLength( 0 ) % 16; x < 16; x++ )
					sHex += "   ";

				sHex += "   " + sDisp;
			}

			System.Diagnostics.StackFrame sf = new System.Diagnostics.StackFrame( 1, true );
			log( sHex, sf );
		}
	}
}
