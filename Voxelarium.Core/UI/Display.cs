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
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif
using OpenTK.Input;
using OpenTK;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using Voxelarium.Core.UI.Shaders;
using System.Diagnostics;
using Voxelarium.LinearMath;
using System.Threading;
using Voxelarium.Core.Game.Screens;
using Voxelarium.Core.Voxels;
using System.Drawing.Imaging;
using Voxelarium.Core.Support;
using Voxelarium.Core.Voxels.UI;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.Core.Voxels.Physics;
using OpenTK.Graphics;
using Voxelarium.Common;

namespace Voxelarium.Core.UI
{
	public class Display : OpenTK.GameWindow
	{
		Stopwatch sw = new Stopwatch();
		float frequency;
		internal Voxelarium.Core.VoxelGameEnvironment game;

		internal static int max_texture_size;
		internal static Matrix4 modelview;
		//internal static Matrix4 worldview;

		internal static Matrix4 projection;
		//internal static btTransform worldview;
		MouseDevice mouse;
		bool initialized;

		float _mouse_x = -2, _mouse_y;
		float mouse_x, mouse_y;
		float del_mouse_x, del_mouse_y;
		float prior_mouse_time;
		int display_width;
		int display_height;
		int display_x;
		int display_y;
		internal SimpleShader simple = new SimpleShader();
		internal SimpleInstanceShader simple_instance = new SimpleInstanceShader();
		internal SimpleGuiShader simple_gui = new SimpleGuiShader();
		internal SimpleEdgeShader edge = new SimpleEdgeShader();
		internal ColorEdgeShader color_edge = new ColorEdgeShader();
		internal SimpleTextureShader simple_texture = new SimpleTextureShader();
		internal SimpleGuiTextureShader simple_gui_texture = new SimpleGuiTextureShader();

		internal static List<Shaders.Shader> used_shaders = new List<Shaders.Shader>();
		internal List<Shaders.Shader> shaders = new List<Shaders.Shader>();

		static bool InitializedGame;
		static bool game_loaded;

		internal static Camera free_camera;
		internal static Camera active_camera;
		static btTransform debug_cube_transform;

		public Display( VoxelGameEnvironment game ) : base(
				  Settings.Read( "GL.Width", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Width )
				, Settings.Read( "GL.Height", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Height )
				, new OpenTK.Graphics.GraphicsMode( 32, 24, 24, 4, ColorFormat.Empty, 2, true )
				, "Voxelarium", GameWindowFlags.Default )
		{
			this.game = game;
			string versionOpenGL = GL.GetString(StringName.Version);
			//GL.Get
			mouse = Mouse;
			this.WindowBorder = OpenTK.WindowBorder.Hidden;
			display_width = Width;// = Settings.Read( "GL.Width", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Width );
			display_height = Height;// = Settings.Read( "GL.Height", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Height );
			mouse_x = display_width / 2;
			mouse_y = display_height / 2;
			display_x = X = Settings.Read( "GL.Display.X", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.X );
			display_y = Y = Settings.Read( "GL.Display.Y", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Y );
			System.Windows.Forms.Cursor.Position = new Point( display_x + ( display_width ) / 2, display_y + display_height / 2 );
			UpdateFrame += Display_UpdateFrame;
			RenderFrame += Display_RenderFrame;
			GL.Viewport( 0, 0, Width, Height );
			GL.Enable( EnableCap.Multisample );
			Matrix4.CreatePerspectiveFieldOfView( (float)( System.Math.PI / 2 ), (float)Width / (float)Height, 0.01f, 10000, out projection );
			//projection = Matrix4.Identity;

			// generic fly camera not attached to any object
			free_camera = new Camera();
			active_camera = free_camera; // default to freecam;
			free_camera.MoveTo( 0, 4, 0 );
			debug_cube_transform = btTransform.Identity;

			shaders.Add( new SimpleShader() );
			KeyDown += Display_KeyDown;
			KeyUp += Display_KeyUp;
			MouseWheel += Display_MouseWheel;
			MouseMove += Display_MouseMove;
			MouseDown += Display_MouseDown;
			MouseUp += Display_MouseUp;

			this.Load += Display_Load;
			this.Resize += Display_Resize;
			this.Move += Display_Move;
		}

		private void Display_MouseUp( object sender, MouseButtonEventArgs e )
		{
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.MouseButtonRelease( e.Button, mouse_x = e.X * 2.0f / Width - 1, mouse_y = ( Height - (float)e.Y ) * 2.0f / Height - 1 );
		}

		private void Display_MouseDown( object sender, MouseButtonEventArgs e )
		{
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.MouseButtonClick( e.Button, mouse_x = e.X * 2.0f / Width - 1, mouse_y = ( Height - (float)e.Y ) * 2.0f / Height - 1 );
		}

		void ProcessMouseFreeCam( float time )
		{
			float del_time;
			if( prior_mouse_time != 0 )
			{
				del_time = time;// - prior_mouse_time;
				if( del_mouse_x != 0 )
				{
					float delta = (float)( del_mouse_x * del_time ) * 100;
					free_camera.RotateYaw( delta );
					del_mouse_x = 0;
				}
				if( del_mouse_y != 0 )
				{
					float delta = (float)( del_mouse_y * del_time ) * 100;
					free_camera.RotatePitch( delta );
					del_mouse_y = 0;
				}
			}
			prior_mouse_time = time;
		}

		private void Display_MouseMove( object sender, MouseMoveEventArgs e )
		{
			//Log.log( "Mouse move {0},{1}", e.X, e.Y );
			mouse_x = e.X * 2.0f / Width - 1;
			mouse_y = ( Height - (float)e.Y ) * 2.0f / Height - 1;
			if( _mouse_x != -2 )
			{
				del_mouse_x += mouse_x - _mouse_x;
				del_mouse_y += mouse_y - _mouse_y;
			}
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.MouseMove( del_mouse_x, del_mouse_y, mouse_x, mouse_y );
			if( HiddenMouse )
			{
				System.Windows.Forms.Cursor.Position = new Point( ( X + Width ) / 2, ( Y + Height ) / 2 );
				_mouse_x = 0;
				_mouse_y = 0;
			}
			else
			{
				_mouse_x = mouse_x;
				_mouse_y = mouse_y;
			}
		}

		private void Display_MouseWheel( object sender, MouseWheelEventArgs e )
		{
			MouseButton button = e.Delta < 0 ? MouseButton.Button7 : MouseButton.Button6;
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.MouseButtonClick( button, mouse_x, mouse_y );

		}

		private void Display_KeyUp( object sender, KeyboardKeyEventArgs e )
		{
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.KeyDown( e.Key );
		}

		private void Display_KeyDown( object sender, KeyboardKeyEventArgs e )
		{
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.KeyDown( e.Key );
		}

		private void Display_Move( object sender, EventArgs e )
		{
			display_x = X;
			display_y = Y;
		}
		private void Display_Resize( object sender, EventArgs e )
		{
			display_width = Width;
			display_height = Height;
			GL.Viewport( 0, 0, Width, Height );
		}

		private void Display_Load( object sender, EventArgs e )
		{
			int val = GL.GetInteger( GetPName.CullFace );
			frequency = Stopwatch.Frequency / 1000.0f;
			sw.Start();

			max_texture_size = GL.GetInteger( GetPName.MaxTextureSize );

			CheckErr();
			GL.Hint( HintTarget.PerspectiveCorrectionHint, HintMode.Nicest );
			//throw new NotImplementedException();
		}

		static bool HiddenMouse;
		public static void HideMouse()
		{
			HiddenMouse = true;
			System.Windows.Forms.Cursor.Hide();
		}
		public static void ShowMouse()
		{
			HiddenMouse = false;
			System.Windows.Forms.Cursor.Show();
		}


		void DrawProgress()
		{
			Vector3[] coords = new Vector3[4];
			coords[0].X = -1;
			coords[0].Y = -0.01f;
			coords[0].Z = 0;
			coords[1].X = -1;
			coords[1].Y = 0.01f;
			coords[1].Z = 0;
			coords[2].X = -1 + ( game.percent_done ) / 50f;
			coords[2].Y = -0.01f;
			coords[2].Z = 0;
			coords[3].X = -1 + ( game.percent_done ) / 50f;
			coords[3].Y = 0.01f;
			coords[3].Z = 0;
			Vector4 c = new Vector4( 1 );
			simple_gui.Activate();
			simple_gui.DrawQuad( coords, ref c );
		}

		private void Shutdown()
		{
			VoxelGlobalSettings.Exiting = true;
			Exit();
		}

		private void Display_UpdateFrame( object sender, OpenTK.FrameEventArgs e )
		{
			if( !InitializedGame )
			{
				InitializedGame = true;
				Thread init = new Thread( InitGame );
				init.Start( game );
			}
			if( game_loaded )
			{
				if( !game.Update() )
				{
					VoxelGlobalSettings.Exiting = true;
					Shutdown();
				}
			}
			if( game.Game_Run )
				game.Engine.Step( (float)e.Time );
			ProcessMouseFreeCam( (float)e.Time );
			if( Keyboard[Key.AltLeft] && Keyboard[Key.F4] )
			{
				VoxelGlobalSettings.Exiting = true;
				Shutdown();
			}
			if( Keyboard[Key.Escape] )
			{
				Shutdown();
			}

			if( Keyboard[Key.X] && !Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.x += 0.2f;
			if( Keyboard[Key.X] && Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.x -= 0.2f;
			if( Keyboard[Key.Z] && !Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.z += 0.2f;
			if( Keyboard[Key.Z] && Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.z -= 0.2f;
			if( Keyboard[Key.Y] && !Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.y += 0.2f;
			if( Keyboard[Key.Y] && Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.y -= 0.2f;

			if( Keyboard[Key.R] && !Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 2, 0.1f );
			if( Keyboard[Key.R] && Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 2, -0.1f );

			if( Keyboard[Key.T] && !Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 1, 0.1f );
			if( Keyboard[Key.T] && Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 1, -0.1f );

			if( Keyboard[Key.P] && !Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 0, 0.1f );
			if( Keyboard[Key.P] && Keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 0, -0.1f );

			if( Keyboard[Key.Space] )
				free_camera.MoveUp( 10 * (float)e.Time );
			if( Keyboard[Key.AltLeft] )
				free_camera.MoveUp( -10 * (float)e.Time );
			if( Keyboard[Key.W] )
			{
				free_camera.MoveForward( 40 * (float)e.Time );
			}
			if( Keyboard[Key.S] )
			{
				free_camera.MoveForward( -10 * (float)e.Time );
			}
			if( Keyboard[Key.A] )
			{
				free_camera.MoveRight( -10 * (float)e.Time );
			}
			if( Keyboard[Key.D] )
			{
				free_camera.MoveRight( 10 * (float)e.Time );
			}
			if( Keyboard[Key.Q] )
			{
				free_camera.RotateRoll( -2f * (float)e.Time );
			}
			if( Keyboard[Key.E] )
			{
				free_camera.RotateRoll( 2f * (float)e.Time );
			}

			if( !initialized )
			{
				initialized = true;
			}
			if( game.VoxelProcessor != null )
				game.VoxelProcessor.SetPlayerPosition( ref free_camera.location.m_origin );
        }

		void InitGame( object o )
		{
			VoxelGameEnvironment game = o as VoxelGameEnvironment;
			if( !game.Init() )
			{
				VoxelGlobalSettings.Exiting = true;
				Exit();
			}
			game.Basic_Renderer.Camera = free_camera;
			game_loaded = true;

		}

		void DrawDebugCube()
		{
			Vector4 c = new Vector4(1);
			Vector3[] verts = new Vector3[4];
			simple_instance.Activate();
			simple_instance.SetModelMatrix( ref debug_cube_transform );

			c.X = 0; c.Y = 0;
			verts[0].X =  1; verts[0].Y =  1; verts[0].Z =  1;
			verts[1].X = -1; verts[1].Y =  1; verts[1].Z =  1;
			verts[2].X =  1; verts[2].Y = -1; verts[2].Z =  1;
			verts[3].X = -1; verts[3].Y = -1; verts[3].Z =  1;
			simple_instance.DrawQuad( verts, ref c );

			c.X = 1; c.Z = 0;
			verts[0].X = 1; verts[0].Y = 1; verts[0].Z = -1;
			verts[1].X = -1; verts[1].Y = 1; verts[1].Z = -1;
			verts[2].X = 1; verts[2].Y = -1; verts[2].Z = -1;
			verts[3].X = -1; verts[3].Y = -1; verts[3].Z = -1;
			simple_instance.DrawQuad( verts, ref c );

			c.X = 0; c.Y = 1; c.Z = 0;
			verts[0].X = 1; verts[0].Y = 1; verts[0].Z = 1;
			verts[1].X = -1; verts[1].Y = 1; verts[1].Z = 1;
			verts[2].X = 1; verts[2].Y = 1; verts[2].Z = -1;
			verts[3].X = -1; verts[3].Y = 1; verts[3].Z = -1;
			simple_instance.DrawQuad( verts, ref c );

			c.X = 1; c.Y = 0; c.Z = 1;
			verts[0].X = 1; verts[0].Y = -1; verts[0].Z = 1;
			verts[1].X = -1; verts[1].Y = -1; verts[1].Z = 1;
			verts[2].X = 1; verts[2].Y = -1; verts[2].Z = -1;
			verts[3].X = -1; verts[3].Y = -1; verts[3].Z = -1;
			simple_instance.DrawQuad( verts, ref c );

			c.X = 1; c.Y = 1; c.Z = 0;
			verts[0].X = 1; verts[0].Y = 1; verts[0].Z = 1;
			verts[1].X = 1; verts[1].Y = -1; verts[1].Z = 1;
			verts[2].X = 1; verts[2].Y = 1; verts[2].Z = -1;
			verts[3].X = 1; verts[3].Y = -1; verts[3].Z = -1;
			simple_instance.DrawQuad( verts, ref c );

			c.X = 0; c.Y = 1; c.Z = 1;
			verts[0].X = -1; verts[0].Y = 1; verts[0].Z = 1;
			verts[1].X = -1; verts[1].Y = -1; verts[1].Z = 1;
			verts[2].X = -1; verts[2].Y = 1; verts[2].Z = -1;
			verts[3].X = -1; verts[3].Y = -1; verts[3].Z = -1;
			simple_instance.DrawQuad( verts, ref c );
		}

		int frame;
		private void Display_RenderFrame( object sender, OpenTK.FrameEventArgs e )
		{
			// render graphics
			frame++;
			GL.Clear( ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit );
			if( !game_loaded )
			{
				DrawProgress();
				SwapBuffers();
				return;
			}
			game.Draw( this, sw.ElapsedTicks / frequency );
			GL.Enable( EnableCap.DepthTest );
			Vector3[] verts = new Vector3[4];
			Vector2[] text = new Vector2[4];
			Vector4[] colors = new Vector4[4];
			verts[0].X = 0;
			verts[0].Y = 0;
			verts[0].Z = 10;
			text[0].X = 0;
			text[0].Y = 0;
			colors[0] = new Vector4( 0.3f, 0, 0, 1 );

			verts[1].X = 0;
			verts[1].Y = 10;
			verts[1].Z = 10;
			text[1].X = 0;
			text[1].Y = 3;
			colors[1] = new Vector4( 0, 0.3f, 0, 1 );

			verts[2].X = 10;
			verts[2].Y = 0;
			verts[2].Z = 10;
			text[2].X = 3;
			text[2].Y = 0;
			colors[2] = new Vector4( 0, 0, 0.3f, 1 );

			verts[3].X = 10;
			verts[3].Y = 10;
			verts[3].Z = 10;
			text[3].X = 3;
			text[3].Y = 3;
			colors[3] = new Vector4( 0, 0.5f, 0.7f, 1 );

			Vector4 color = new Vector4( 0.5f, 0.5f, 0.2f, 1 );
			Vector4 face_color = new Vector4( 0f, 0f, 1f, 1 );
			if( edge.Activate() )
			{
				Display.CheckErr();
				edge.DrawQuad( verts, text, ref face_color, ref color );

				for( int i = 0; i < 4; i++ )
				{
					text[i].X *= 2;
					text[i].Y *= 2;
					verts[i].X += 10;
				}
				face_color = new Vector4( 0.5f, 0.5f, 0.5f, 1 );
				color = new Vector4( 0.0f, 0.0f, 0.0f, 1 );
				edge.Activate();
				Display.CheckErr();
				edge.DrawQuad( verts, text, ref face_color, ref color );

				for( int i = 0; i < 4; i++ )
				{
					text[i].X *= 2;
					text[i].Y *= 2;
					verts[i].X += 10;
				}
				color = new Vector4( 0.0f, 0.5f, 0.9f, 1 );
				color_edge.Activate();
				Display.CheckErr();
				color_edge.DrawQuad( verts, text, ref face_color, colors, 800 );
				//edge.DrawQuad( verts, ref color );
				Display.CheckErr();
			}
			GL.UseProgram( 0 );

			{
				TileSet.TileStyle style = game.TileSetStyles.GetStyle( 2 );
				Box box = new Box();
				Vector4 DrawColor = new Vector4( 1 );
				box.Size.X = 1;
				box.Size.Y = 1;
				box.Size.Z = 0;
				box.Position.X = -5;
				box.Position.Y = 0;
				box.Position.Z = 0;
				game.Font_1.RenderFont( this, style, ref box, "-20 X", ref DrawColor );
				box.Position.X = 5;
				game.Font_1.RenderFont( this, style, ref box, "20 X", ref DrawColor );
				box.Position.X = 0;
				box.Position.Y = -5;
				game.Font_1.RenderFont( this, style, ref box, "-20 Y", ref DrawColor );
				box.Position.Y = 5;
				game.Font_1.RenderFont( this, style, ref box, "20 Y", ref DrawColor );
				box.Position.Y = 0;
				box.Position.Z = -5;
				game.Font_1.RenderFont( this, style, ref box, "-20 Z", ref DrawColor );
				box.Position.Z = 5;
				game.Font_1.RenderFont( this, style, ref box, "20 Z", ref DrawColor );

				DrawDebugCube();
			}
			//Log.log( " Origin is " + free_camera.location.m_origin );
			Display.CheckErr();
#if !USE_GLES2
			GL.MatrixMode( MatrixMode.Projection );
			GL.LoadMatrix( ref projection );
			GL.MatrixMode( MatrixMode.Modelview );
			unsafe
			{
				btMatrix3x3 tmp;
				free_camera.location.GetGLCameraMatrix( out tmp );
				//Log.log( tmp.ToString() );
					//Console.WriteLine( worldview.ToString() );
					//Console.WriteLine( tmp.ToString() );
					float * matrix_ptr = &tmp.m_el0.x;
				{
					GL.LoadMatrix( matrix_ptr );
				}
			}
			Shader.Deactivate();
			BulletDebugDrawer.DrawSpace( game.Engine );

			//GL.BindTexture( TextureTarget.Texture2D, frame %100 );
			GL.Begin( PrimitiveType.Triangles );
			GL.TexCoord2( 0, 1 );
			GL.Color3( 1.0f, 1.0f, 0.0f ); GL.Vertex3( -1.0f, -1.0f, 4.0f );
			GL.TexCoord2( 1, 1 );
			GL.Color3( 1.0f, 0.0f, 0.0f ); GL.Vertex3( 1.0f, -1.0f, 4.0f );
			GL.TexCoord2( 1, 0 );
			GL.Color3( 0.2f, 0.9f, 1.0f ); GL.Vertex3( 0.0f, 1.0f, 4.0f );

			GL.End();
#endif
			SwapBuffers();
		}

		internal static float SclX( float x ) { return x * 2.0f / 1024f; }
		internal static float SclY( float y ) { return y * 2.0f / 768f; }

		[Conditional( "DEBUG" )]
		internal static void CheckErr()
		{
			ErrorCode code = GL.GetError();
			if( code != 0 )
			{
				Log.log( "error " + code, 1  );
				//return true;
			}
			//return false;
		}
	}
}
