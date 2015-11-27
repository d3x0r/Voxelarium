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
#define USE_EXTERNAL_COMPILER
using System;
using System.CodeDom.Compiler;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text;
using System.Windows.Forms;
using Voxelarium.Common;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Support
{
	public class Compiler
	{
#if USE_MONO_CSHARP
		// requires 4.0; 
		// 4.0 OpenTK requires signing
		// Signed OpenTK 4.0 library crashes in NuGet version.

		static StringWriter sw;
		static Mono.CSharp.ReportPrinter rp ;
		static Mono.CSharp.CompilerContext cc;
		static Mono.CSharp.Report r;
		static Mono.CSharp.Evaluator e;
#endif

		static Dictionary<int, Assembly> loaded_objects = new Dictionary<int, Assembly>();

		static bool AllowAssembly( Assembly a )
		{
			AssemblyName[] names = a.GetReferencedAssemblies();
			foreach( AssemblyName name in names )
			{
				if( name.Name == "System.IO" )
					return false;
			}
			Type[] types = a.GetTypes();
			foreach( Type t in types )
			{

			}
			//a.
			return true;
		}

		internal static IWorldGenesis LoadGenesisCode()
		{
			string FileName;
			FileName = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "/Code/genesis.cs";
			if( File.Exists( FileName ) )
			{
				string code = File.ReadAllText( FileName );
				Assembly a = CompileCode( code, FileName, new string[] { "System.Drawing.dll" } );
				if( a != null )
				{
					Type[] types = a.GetTypes();
					foreach( Type type in types )
					{
						if( type.GetInterface( "IWorldGenesis" ) != null )
						{
							object o = Activator.CreateInstance( type );
							return o as IWorldGenesis;
						}
					}
				}
			}
			return null;
		}


		static Assembly CompileCode( string code, string filename, string[] extra )
		{
#if USE_ROSLYN_COMPILATION
					Microsoft.CodeAnalysis.
				using( var ms = new MemoryStream() )
				{
					string assemblyFileName = "gen" + Guid.NewGuid().ToString().Replace( "-", "" ) + ".dll";

					CSharpCompilation compilation = CSharpCompilation.Create( assemblyFileName,
						new[] { CSharpSyntaxTree.ParseText( fooSource ) },
						new[]
						{
						new MetadataFileReference(typeof (object).Assembly.Location)
						},
						new CSharpCompilationOptions( OutputKind.DynamicallyLinkedLibrary )
						);

					compilation.Emit( ms );
					Assembly assembly = Assembly.Load( ms.GetBuffer() );
					return assembly;
				}
#endif
#if USE_MONO_CSHARP
				if( props.VoxelClassName != null )
				{
					if( sw == null )
						sw = new StringWriter();
					if( rp == null )
						rp = new Mono.CSharp.StreamReportPrinter( sw );
					if( cc == null )
						cc = new Mono.CSharp.CompilerContext( new Mono.CSharp.CompilerSettings(), rp );
					if( r == null )
						r = new Mono.CSharp.Report( cc, rp );
					if( e == null )
					{
						e = new Mono.CSharp.Evaluator( cc );
						e.ReferenceAssembly( Assembly.GetCallingAssembly() );
					}
					Mono.CSharp.CompiledMethod m;
					long now;
					now = DateTime.Now.Ticks;
                    Log.log( "About to compile" + (DateTime.Now.Ticks - now ));
					String s = e.Compile( code, out m );

					Log.log( "did compile    " + (DateTime.Now.Ticks - now ) );
					s = sw.ToString();
					if( !s.Contains( "error" ) )
					{
						Assembly asm = ( (Type)e.Evaluate( "typeof(" + props.VoxelClassName + ");" ) ).Assembly;
						Log.log( "got types   " + ( DateTime.Now.Ticks - now ) );
						Type[] types = asm.GetTypes();
						loaded_objects.Add( type, asm );
						foreach( Type t in types )
						{
							if( t.BaseType.Name == "VoxelType" )
							{
								object o = Activator.CreateInstance( t );
								return o as VoxelType;
							}
						}
					}
					else
						Log.log( "Compile Failure: " + s );
				}
#endif
#if USE_EXTERNAL_COMPILER
			//code = "using System; using Voxelarium.Core; using Voxelarium.Core.Voxels;\nnamespace Voxelarium.Core.Voxels { " + code + "}";

			string codeBase = Assembly.GetExecutingAssembly().CodeBase;
			UriBuilder uri = new UriBuilder( codeBase );
			string path = Uri.UnescapeDataString( uri.Path );

			using( Microsoft.CSharp.CSharpCodeProvider foo =
						new Microsoft.CSharp.CSharpCodeProvider() )
			{
				string[] externals;
				if( extra != null )
				{
					externals = new string[1 + extra.Length];
					externals[0] = path;
					for( int i = 0; i < extra.Length; i++ )
						externals[i + 1] = extra[i];
				}
				else
				{
					externals = new string[1];
					externals[0] = path;
				}
				CompilerResults res = foo.CompileAssemblyFromSource(
					new System.CodeDom.Compiler.CompilerParameters( externals )
					{
						IncludeDebugInformation = true
						, GenerateInMemory = true
					},
					code
				);
				if( res.Errors.Count == 0 )
				{
					if( AllowAssembly( res.CompiledAssembly ) )
					{
						return res.CompiledAssembly;
					}
				}
				else
				{
					StringBuilder sb = new StringBuilder();
					foreach( CompilerError err in res.Errors )
					{
						sb.Append( filename + "(" + err.Line + ":" + err.Column + "):" );
						sb.Append( err.ErrorText );
						sb.Append( "\n" );
					}
					MessageBox.Show( sb.ToString() );
					Log.log( sb.ToString() );
				}
			}
#endif
			return null;
		}

		internal static bool LoadVoxelCode( int type )
		{
			string FileName;
			if( type < 32768 )
			{
				FileName = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "/VoxelTypes/voxelinfo/" + String.Format( "voxelcode_" + type + ".cs" );
			}
			else
			{
				FileName = VStreamFile.Get_Directory_UserData()
					+ "/" + VoxelGlobalSettings.COMPILEOPTION_SAVEFOLDERNAME
					+ "/VoxelTypes/voxelinfo/" + String.Format( "voxelcode_" + type + ".cs" );
			}
			if( false && File.Exists( FileName ) )
			{
				string code = File.ReadAllText( FileName );
				Assembly a = CompileCode( code, FileName, null );
				if( a != null && AllowAssembly( a ) )
				{
					loaded_objects.Add( type, a );
					return true;
				}
			}
			return false;
		}

		internal static VoxelExtension LoadExtendedVoxelExtension( int type )
		{
			Assembly a = loaded_objects[type];
			if( a == null )
				if( LoadVoxelCode( type ) )
					a = loaded_objects[type];

			if( a != null )
			{
				Type[] types = a.GetTypes();
				foreach( Type t in types )
				{
					if( t.BaseType.Name == "VoxelExtension" )
					{
						object o = Activator.CreateInstance( t, new object[] { type }, null );
						return o as VoxelExtension;
					}
				}
			}
			return null;
		}


		internal static VoxelType LoadExtendedVoxelType( int type )
		{
			Assembly a = loaded_objects[type];
			if( a == null )
				if( LoadVoxelCode( type ) )
					a = loaded_objects[type];

			if( a != null )
			{
				Type[] types = a.GetTypes();
				foreach( Type t in types )
				{
					if( t.BaseType.Name == "VoxelType" )
					{
						object o = Activator.CreateInstance( t );
						return o as VoxelType;
					}
				}
			}
			return null;
		}

	}
}
