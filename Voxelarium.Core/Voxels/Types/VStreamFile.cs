/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel. (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
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
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace Voxelarium.Core.Voxels.Types
{
	class VStreamFile
	{
		public static string Get_Directory_UserData()
		{
			return Environment.GetFolderPath( Environment.SpecialFolder.MyDocuments  ) + "/Games";
		}
    }
#if asdfasdfasdf
		public  string FileName;
		//protected FileStream Fl;
		protected FileStream Fl;
		protected bool ReadOk;
		protected bool WriteOk;
		protected bool Error;

#if asdfasdf
		bool CopyFileFrom( VStreamFile &SrcStream);
		bool AppendToOpenedFileFrom( VStreamFile &SrcStream);
		bool CompareFileFrom( VStreamFile &SrcStream);
		bool GetFileContent( ZString & Result);
		bool PutFileContent( ZString & Content);


		static bool Set_CurrentDirectory( char const * const Directory);
		static ZString Get_CurrentDirectory();
		static ZString Get_Directory_UserData();
		static bool Destroy_File( char* FileToDestroy );
		static bool Copy_File( char* ToCopy, char* NewFile, bool FailIfExists );
		static bool Compare_Files( char* File1, char* File2 );
#endif

		public void AddToSavedLen( ref uint Len, uint x ) { Len += sizeof( uint ); }
		public void AddToSavedLen( ref uint Len, long x ) { Len += sizeof( long ); }
		public void AddToSavedLen( ref uint Len, ushort x ) { Len += sizeof( ushort ); }
		public void AddToSavedLen( ref uint Len, short x ) { Len += sizeof( short ); }
		public void AddToSavedLen( ref uint Len, byte x ) { Len += sizeof( byte ); }
		public void AddToSavedLen( ref uint Len, char x ) { Len += sizeof( char ); }
		//public void AddToSavedLen( out uint Len, string String ) { uint i; for( i = 0; String[i] != 0; i++ ) ; Len += i + sizeof( uint ); }
		public void AddToSavedLen( ref uint Len, string String ) { Len += (uint)String.Length + sizeof( uint ); }
		//public void AddToSavedLen( out uint Len, string String) { Len += String.Len + sizeof( uint ); }

		bool IsError()
		{
			bool Flag;

			Flag = Error;
			Error = false;
			return ( Flag );
		}

		bool LookError()
		{
			return ( Error );
		}

		bool WriteObj( object Object, uint ObjectLen, uint nObjects )
		{
			if( Fl != null && WriteOk )
			{
				//if( nObjects == fwrite( Object, ObjectLen, nObjects, (FILE*)Fl ) )
				{
					return ( true );
				}
			}
			Error = true;
			return ( false );
		}

		uint ReadObj( object Object, uint ObjectLen, uint nObjects )
		{
			if( Fl != null && ReadOk )
			{
				//if( ( nObjects = fread( Object, ObjectLen, nObjects, (FILE*)Fl ) ) )
				{
					return ( nObjects );
				}
			}
			Error = true;
			return ( 0U );
		}

		bool Move( long Offset )
		{
			Fl.BaseStream.Seek( Offset, SeekOrigin.Current );
			return true;
		}

		long Note()
		{
			return Fl.BaseStream.Position;
		}

		bool Point( uint NoteVal )
		{
			Fl.BaseStream.Seek( NoteVal, SeekOrigin.Begin );
			return ( true );
		}

		void Flush()
		{
			if( Fl != null && ( WriteOk || ReadOk ) ) Fl.Flush();
		}


public bool CopyFileFrom( VStreamFile SrcStream)
{
	char[] Buffer;
	uint i;
	bool ReturnValue = false;
			Buffer = new char[10240];
	{
		if( SrcStream.OpenRead() )
		{
			if( OpenWrite() )
			{
				do
				{
					i = SrcStream.Fl.ReadObj( Buffer, 1, 10240 );
					if( i ) WriteObj( Buffer, 1, i );
				} while( i );
				ReturnValue = true;
				Close();
			}
			SrcStream.Close();
		}
		delete Buffer;
	}
	return ( ReturnValue );
}

bool AppendToOpenedFileFrom( VStreamFile &SrcStream)
{
	char* Buffer;
	uint i;
	bool ReturnValue = false;
	if( ( Buffer = new char[10240] ) )
	{
		if( SrcStream.OpenRead() )
		{
			if( WriteOk )
			{
				do
				{
					i = SrcStream.ReadObj( Buffer, 1, 10240 );
					if( i ) WriteObj( Buffer, 1, i );
				} while( i );
				ReturnValue = true;
			}
			SrcStream.Close();
		}
		delete Buffer;
	}
	return ( ReturnValue );
}


bool CompareFileFrom( VStreamFile &SrcStream)
{
	char* Buffer1;
	char* Buffer2;
	uint i1, i2;
	bool ReturnValue = false;
	if( ( Buffer1 = new char[10240] ) )
	{
		if( ( Buffer2 = new char[10240] ) )
		{
			if( SrcStream.OpenRead() )
			{
				if( OpenRead() )
				{
					ReturnValue = true;
					do
					{
						i1 = SrcStream.ReadObj( Buffer1, 1, 10240 );
						i2 = ReadObj( Buffer2, 1, 10240 );
						if( i1 != i2 ) ReturnValue = false;
						else
						{
							if( i2 )
								while( --i2 ) if( Buffer1[i2] != Buffer2[i2] ) { ReturnValue = false; continue; }
						}
					} while( ReturnValue && i1 );
					Close();
				}
				SrcStream.Close();
			}
			delete Buffer2;
		}
		delete Buffer1;
	}
	return ( ReturnValue );
}



bool Close()
{
	ReadOk = false;
	WriteOk = false;
	if( Fl )
	{
		fclose( (FILE*)Fl );
		Fl = 0;
		return ( true );
	}
	Error = true;
	return ( false );
}

bool OpenWrite()
{
	if( !Fl )
	{
		if( ( Fl = (void*)fopen( FileName.String, "wb" ) ) )
		{
			WriteOk = true;
			Error = false;
			return ( true );
		}
	}
	Error = true;
	return ( false );
}

bool OpenRead()
{
	if( !Fl )
	{
		if( ( Fl = (void*)fopen( FileName.String, "rb" ) ) )
		{
			ReadOk = true;
			Error = false;
			return ( true );
		}
	}
	Error = true;
	return ( false );
}



ZStream_File()
{
  Fl = 0;
  ReadOk =  false;
  WriteOk = false;
  Error   = false;
}

bool SetFileName( char const * const FileName)
{
	this.FileName = FileName;
	return ( true );
}

bool Set_CurrentDirectory( char const * const Directory)
{
#if ZENV_OS_WINDOWS
	return (::SetCurrentDirectory( Directory ) );
#endif
#if ZENV_OS_LINUX
	return ( chdir( Directory ) ? true : false );
#endif
}

ZString Get_CurrentDirectory()
{
#if ZENV_OS_LINUX
	ZString Buffer;
	Buffer.RaiseMem_DiscardContent( PATH_MAX );
	if( 0 == getcwd( Buffer.String, PATH_MAX ) ) Buffer.Clear();
	return ( Buffer );
#endif
#if ZENV_OS_WINDOWS
	uint Writen;
	ZString Directory;
	Directory.RaiseMem_DiscardContent( 4096 );
	Writen = ::GetCurrentDirectory( 4096UL, Directory.String );
	Directory.SetLen( Writen );
	return ( Directory );
#endif
}

ZString Get_Directory_UserData()
{
#if ZENV_OS_LINUX
	ZString Buffer;
	char* Str;

	Str = getenv( "HOME" );
	if( Str ) Buffer = Str;
	else Str = (char*)"/home/laurent";

	return ( Buffer );
#endif
#if ZENV_OS_WINDOWS
	HRESULT Result;
	ZString Directory, Buffer;
	Buffer.RaiseMem_DiscardContent( 4096 );
	Result = ::SHGetFolderPath( 0, CSIDL_PERSONAL, 0, 0, Buffer.String );
	// Result = ::SHGetFolderPath(0, CSIDL_APPDATA, 0, 0,  Buffer.String);
	if( Result == S_OK ) Directory = Buffer.String;
	return ( Directory );
#endif
}

bool Copy_File( char* ToCopy, char* NewFile, bool FailIfExists )
{
#if ZENV_OS_LINUX
	// To do
	return ( false );
#endif

#if ZENV_OS_WINDOWS
	return (::CopyFile( ToCopy, NewFile, FailIfExists ) );
#endif
}

bool Compare_Files( char* File1, char* File2 )
{
	char Buf1[4096], Buf2[4096];
	FILE* handle1, *handle2;
	uint F1Len, F2Len, i;

	handle1 = fopen( File1, "rb" ); if( !handle1 ) return ( false );
	handle2 = fopen( File2, "rb" ); if( !handle2 ) { fclose( handle1 ); return ( false ); }
	do
	{
		F1Len = fread( Buf1, 1, 4096, handle1 );
		F2Len = fread( Buf2, 1, 4096, handle2 );
		if( F1Len != F2Len ) return ( false );
		for( i = 0; i < F1Len; i++ ) if( Buf1[i] != Buf2[i] ) return ( false );
	} while( F1Len == 4096 );
	fclose( handle1 );
	fclose( handle2 );
	return ( true );
}

public static bool Directory_Create(const char* NewDir )
{
#if ZENV_OS_LINUX
	return ( mkdir( NewDir, S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP ) ? false : true );
#endif
#if ZENV_OS_WINDOWS
	return ::CreateDirectory( NewDir, 0 ) ? true : false;
#endif
}

public static bool Directory_Destroy(const char* DirectoryToRemove )
{
#if ZENV_OS_LINUX
	return ( rmdir( DirectoryToRemove ) ? false : true );
#endif
#if ZENV_OS_WINDOWS
	return RemoveDirectory( DirectoryToRemove );
#endif
}

bool Destroy_File( char* FileToDestroy )
{
#if ZENV_OS_LINUX
	return ( unlink( FileToDestroy ) ? false : true );
#endif
#if ZENV_OS_WINDOWS
	return DeleteFile( FileToDestroy );
#endif
}


bool PutFileContent( ZString & Content)
{
	if( OpenWrite() )
	{
		if( 1 != WriteObj( Content.String, Content.Len, 1 ) )
		{
			WriteOk = false;
			Error = true;
		}
		else
		{
			WriteOk = true;
			Error = false;
		}
		Close();
	}
	return ( Error );
}

bool GetFileContent( ZString & Result)
{
	uint Len;
	uint EffectiveLen;

	// Opening
	if( !Fl )
	{
		if( ( Fl = (void*)fopen( FileName.String, "rb" ) ) )
		{
			ReadOk = true;
			Error = false;
		}
		else
		{
			ReadOk = false;
			Error = true;
		}
	}
	else { Error = true; return ( false ); }


	// Reading
	if( Fl != null && ReadOk )
	{
		do
		{
			Len = Result.Len;
			Result.SetLen( Len + 4096 );

			EffectiveLen = fread( Result.String + Len, 1, 4096, (FILE*)Fl );
			Result.SetLen( Len + EffectiveLen );
		} while( EffectiveLen == 4096 );
	}
	else Error = true;

	// Closing
	ReadOk = false;
	WriteOk = false;
	if( Fl )
	{
		fclose( (FILE*)Fl );
		Fl = 0;
		return ( true );
	}
	Error = true;
	return ( false );

}

public static bool Directory_IsExists(const char* DirectoryName )
{
#if ZENV_OS_LINUX

	int Result;
  struct stat StatInfo;

  Result = stat( DirectoryName, &StatInfo);

  if (Result!=0) return(false);
  if (StatInfo.st_mode & S_IFDIR) return(true);
  return(false);

#endif
//#if ZENV_OS_WINDOWS

  DWORD Result;

Result = GetFileAttributesA( DirectoryName);

  if (Result == INVALID_FILE_ATTRIBUTES) return(false);
  if (Result & FILE_ATTRIBUTE_DIRECTORY) return(true);
  return(false);

//#endif
}

public static bool Directory_CreateIfNotExists( string NewDir )
{
	if( Directory_IsExists( NewDir ) ) return ( true );

	return ( Directory_Create( NewDir ) );

}

	}
#endif
	}
