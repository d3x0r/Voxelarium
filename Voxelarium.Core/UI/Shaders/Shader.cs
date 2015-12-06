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
//#define LOG_UNIFORMS
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Support;
using Voxelarium.Common;
using System.Diagnostics;


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

		public Shader()
		{
			Display.OnInvalidate += Display_OnInvalidate;
		}

		static Shader()
		{
			Display.OnInvalidate += Display_OnInvalidateStatic;
		}

		private static void Display_OnInvalidateStatic()
		{
			cache_used = 0;
			TextureCache.Clear();
		}

		void Display_OnInvalidate ()
		{
			loaded = false;
			Program = 0;
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
			Log.log( "Unload - delete" );
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
					#if LOG_UNIFORMS
					Log.log( "projection is {0}", projection_id );
					#endif
					GL.UniformMatrix4( projection_id, false, ref Display.projection );
					Display.CheckErr();
					unsafe
					{
						btMatrix3x3 tmp;
						Display.active_camera.location.GetGLCameraMatrix( out tmp );
						float* matrix_ptr = &tmp.m_el0.x;
						//fixed ( float* matrix_ptr = &Display.worldview.m_el0.x )
						{
							#if LOG_UNIFORMS
							Log.log( "worldview is {0}", worldview_id );
							#endif
							GL.UniformMatrix4( worldview_id, 1, false, matrix_ptr );
						}
					}
					Display.CheckErr();
					if( modelview_id >= 0 )
					{
						#if LOG_UNIFORMS
						Log.log( "modelview is {0}", modelview_id );
						#endif
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
				Log.log( "Failed to compile Shader. Nothing to Compile." );
				return false;
			}

			if( Program > 0 ) {
				Log.log( "Recompile - delete" );
				GL.DeleteProgram( Program );
			}

			//Variables.Clear();

			Program = GL.CreateProgram();
			//Log.log( "program ID is {0} for {1}", Program, this.ToString() );
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
					Log.log( "Failed to Compile Vertex Shader Source. \"" + info + "\"  Status Code: " + status_code.ToString() );

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
					Log.log( "Failed to Compile Fragment Shader Source." + info + "Status Code: " + status_code.ToString() );

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
#if USE_GLES2
			GL.GetProgram( Program, ProgramParameter.LinkStatus, out status_code );
#else
			GL.GetProgram( Program, GetProgramParameterName.LinkStatus, out status_code );
#endif

			if( status_code != 1 )
			{
				Log.log( "Failed to Link Shader Program." +
					Environment.NewLine + info + Environment.NewLine + "Status Code: " + status_code.ToString() );

				GL.DeleteProgram( Program );
				Program = 0;

				return false;
			}

			projection_id = GL.GetUniformLocation( Program, "Projection" );
			#if LOG_UNIFORMS
			Log.log( "uniform is {0}", projection_id );
			#endif
			worldview_id = GL.GetUniformLocation( Program, "worldView" );
			#if LOG_UNIFORMS
			Log.log( "uniform is {0}", worldview_id );
			#endif
			modelview_id = GL.GetUniformLocation( Program, "modelView" );
			#if LOG_UNIFORMS
			Log.log( "uniform is {0}", modelview_id );
			#endif
			return true;
		}

		static void BindTexture( int texture_unit, int ID )
		{
			if( loaded_texture[texture_unit] != ID )
			{
				GL.ActiveTexture( TextureUnit.Texture0 + texture_unit );
				GL.BindTexture( TextureTarget.Texture2D, ID );
				loaded_texture[texture_unit] = ID;
			}
		}

		internal struct TextureCacheEntry
		{
			internal int ID;
			internal int texture_unit;
		}
		static int cache_used;
		static LinkedList<TextureCacheEntry> TextureCache = new LinkedList<TextureCacheEntry>();

		/// <summary>
		/// returns texture unit to use for specified texture
		/// </summary>
		/// <param name="ID">Texture to make sure is loaded, if not, replace oldest texture</param>
		/// <returns></returns>
		internal static int BindTexture( int textureID )
		{
			LinkedListNode<TextureCacheEntry> entry;
			for( entry = TextureCache.First; entry != null; entry = entry.Next )
			{
				if( entry.Value.ID == textureID )
				{
					TextureCache.Remove( entry );
					TextureCache.AddFirst( entry );
					// already bound, but this makes it active for updating/uploading
					GL.ActiveTexture( TextureUnit.Texture0 + entry.Value.texture_unit );
					return entry.Value.texture_unit;
				}
			}
			if( cache_used >= Display.max_texture_units )
			{
				entry = TextureCache.Last;
				entry.Value = new TextureCacheEntry { ID = textureID, texture_unit = entry.Value.texture_unit };

				TextureCache.Remove( entry );
				TextureCache.AddFirst( entry );
			}
			else
			{
				entry = new LinkedListNode<TextureCacheEntry>( new TextureCacheEntry { ID = textureID, texture_unit = cache_used } );
				TextureCache.AddFirst( entry );
				cache_used++;
			}
			GL.ActiveTexture( TextureUnit.Texture0 + entry.Value.texture_unit );
			Display.CheckErr();
			GL.BindTexture( TextureTarget.Texture2D, entry.Value.ID );
			Display.CheckErr();
			return entry.Value.texture_unit;
		}

		public void Dispose()
		{
			GL.DeleteProgram( Program );
		}

	}
}
