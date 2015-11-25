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
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Support;
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif
using Voxelarium.LinearMath;

namespace Voxelarium.Core.UI.Shaders
{
	/// <summary>
	/// Shader Class (Vertex Shader and Fragment Shader)
	/// </summary>
	public abstract class Shader : IDisposable
	{
		static int[] loaded_texture = new int[16];
		internal int Program;
		internal int modelview_id;
		internal int projection_id;
		internal int worldview_id;

		bool loaded;
		//bool parameters_set;
		internal static int prior_activated;

		/// <summary>
		/// Type of Shader
		/// </summary>
		public enum Type
		{
			Vertex = 0x1,
			Fragment = 0x2
		}

		/// <summary>
		/// Get Whether the Shader function is Available on this Machine or not
		/// </summary>
		public static bool IsSupported
		{
			get
			{
				return ( new Version( GL.GetString( StringName.Version ).Substring( 0, 3 ) ) >= new Version( 2, 0 ) ? true : false );
			}
		}

		internal abstract void Compile();

		internal void Unload(  )
		{
			loaded = false;
			GL.DeleteProgram( Program );
		}

		static int prior_thread;

		internal static void Deactivate()
		{
			prior_activated = 0;
			GL.UseProgram( 0 );
		}

		internal  bool Activate(  )
		{
			if( prior_thread == 0 )
				prior_thread = System.Threading.Thread.CurrentThread.ManagedThreadId;
			if( System.Threading.Thread.CurrentThread.ManagedThreadId != prior_thread )
				Log.log( "Activate.." );
			if( !loaded )
			{
				Compile();
				loaded = true;
			}
			if( prior_activated != Program )
			{
				prior_activated = Program;
				GL.UseProgram( Program );
				//if( !parameters_set )
				{
					Display.CheckErr();
					GL.UniformMatrix4( projection_id, false, ref Display.projection );
					Display.CheckErr();
					unsafe
					{
						btMatrix3x3 tmp;
						Display.active_camera.location.GetGLCameraMatrix( out tmp );
						float* matrix_ptr = &tmp.m_el0.x;
						//fixed ( float* matrix_ptr = &Display.worldview.m_el0.x )
						{
							GL.UniformMatrix4( worldview_id, 1, false, matrix_ptr );
						}
					}
					Display.CheckErr();
					if( modelview_id >= 0 )
					{
						GL.UniformMatrix4( modelview_id, false, ref Display.modelview );
						Display.CheckErr();
					}
				}
			}
			return true;
			//GL.UniformMatrix4( 
		}

		// I prefer to return the bool rather than throwing an exception lol
		protected bool Compile( string vertexSource = "", string fragmentSource = "" )
		{
			int status_code = -1;
			string info = "";

			if( vertexSource == "" && fragmentSource == "" )
			{
				Console.WriteLine( "Failed to compile Shader." +
					Environment.NewLine + "Nothing to Compile.", "Error" );
				return false;
			}

			if( Program > 0 )
				GL.DeleteProgram( Program );

			//Variables.Clear();

			Program = GL.CreateProgram();

			if( vertexSource != "" )
			{
				int vertexShader = GL.CreateShader( ShaderType.VertexShader );
				GL.ShaderSource( vertexShader, vertexSource );
				Display.CheckErr();
				GL.CompileShader( vertexShader );
				Display.CheckErr();
				GL.GetShaderInfoLog( vertexShader, out info );
				Display.CheckErr();
				GL.GetShader( vertexShader, ShaderParameter.CompileStatus, out status_code );
				Display.CheckErr();

				if( status_code != 1 )
				{
					Console.WriteLine( "Failed to Compile Vertex Shader Source." +
						Environment.NewLine + info + Environment.NewLine + "Status Code: " + status_code.ToString() );

					GL.DeleteShader( vertexShader );
					GL.DeleteProgram( Program );
					Program = 0;

					return false;
				}

				GL.AttachShader( Program, vertexShader );
				GL.DeleteShader( vertexShader );
			}

			if( fragmentSource != "" )
			{
				int fragmentShader = GL.CreateShader( ShaderType.FragmentShader );
				GL.ShaderSource( fragmentShader, fragmentSource );
				GL.CompileShader( fragmentShader );
				GL.GetShaderInfoLog( fragmentShader, out info ) ;
				GL.GetShader( fragmentShader, ShaderParameter.CompileStatus, out status_code );

				if( status_code != 1 )
				{
					Console.WriteLine( "Failed to Compile Fragment Shader Source." +
						Environment.NewLine + info + Environment.NewLine + "Status Code: " + status_code.ToString() );

					GL.DeleteShader( fragmentShader );
					GL.DeleteProgram( Program );
					Program = 0;

					return false;
				}

				GL.AttachShader( Program, fragmentShader );
				GL.DeleteShader( fragmentShader );
			}

			GL.LinkProgram( Program );
			GL.GetProgramInfoLog( Program, out info );
			GL.GetProgram( Program, GetProgramParameterName.LinkStatus, out status_code );

			if( status_code != 1 )
			{
				Console.WriteLine( "Failed to Link Shader Program." +
					Environment.NewLine + info + Environment.NewLine + "Status Code: " + status_code.ToString() );

				GL.DeleteProgram( Program );
				Program = 0;

				return false;
			}

			projection_id = GL.GetUniformLocation( Program, "Projection" );
			worldview_id = GL.GetUniformLocation( Program, "worldView" );
			modelview_id = GL.GetUniformLocation( Program, "modelView" );
			return true;
		}

		internal static void BindTexture( int texture_unit, int ID )
		{
			if( loaded_texture[texture_unit] != ID )
			{
				GL.ActiveTexture( TextureUnit.Texture0 + texture_unit );
				GL.BindTexture( TextureTarget.Texture2D, ID );
				loaded_texture[texture_unit] = ID;
			}
		}
		public void Dispose()
		{
			throw new NotImplementedException();
		}

	}
}
