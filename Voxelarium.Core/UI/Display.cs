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
using System.Drawing.Imaging;
using System.Windows.Forms;
#else
using OpenTK.Graphics.ES20;
using Android.Graphics;
using Android.Content.Res;
using Android.Content;
using OpenTK.Platform.Android;
using Android.App;
using Android.Views;
#endif
using OpenTK.Input;
using OpenTK;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;
using Voxelarium.Core.UI.Shaders;
using System.Diagnostics;
using Voxelarium.LinearMath;
using System.Threading;
using Voxelarium.Core.Game.Screens;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Support;
using Voxelarium.Core.Voxels.UI;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.Core.Voxels.Physics;
using OpenTK.Graphics;
using Voxelarium.Common;
using System.IO;
using System.Reflection;


namespace Voxelarium.Core.UI
{
	public class Display
	{
		Stopwatch sw = new Stopwatch();
		float frequency;
		internal Voxelarium.Core.VoxelGameEnvironment game;

		internal static int max_texture_size;
		internal static int max_texture_units;
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
		internal static float Aspect;
		internal SimpleShader simple = new SimpleShader();
		internal SimpleInstanceShader simple_instance = new SimpleInstanceShader();
		internal SimpleGuiShader simple_gui = new SimpleGuiShader();
		internal SimpleEdgeShader edge = new SimpleEdgeShader();
		internal ColorEdgeShader color_edge = new ColorEdgeShader();
		internal SimpleTextureShader simple_texture = new SimpleTextureShader();
		internal SimpleGuiTextureShader simple_gui_texture = new SimpleGuiTextureShader();
		internal static SimpleVertShader simpleVertShader = new SimpleVertShader();
		internal static List<Shaders.Shader> used_shaders = new List<Shaders.Shader>();
		internal List<Shaders.Shader> shaders = new List<Shaders.Shader>();

		internal delegate void InvalidateEvent();
		internal static event InvalidateEvent OnInvalidate;

		static bool InitializedGame;
		static bool game_loaded;

		internal static Voxelarium.Core.Voxels.UI.Camera free_camera;
		internal static Voxelarium.Core.Voxels.UI.Camera active_camera;
		static btTransform debug_cube_transform;

		internal KeyboardDevice keyboard;
		public KeyboardDevice Keyboard { set { keyboard = value; } }
		public MouseDevice Mouse { set { mouse = value; } }
		public int Width, Height;
		public int X, Y;
		internal static event SimpleMethod AtExit;

		void CannotExit()
		{
			//Log.log( "Still need a way to set exit... pass a methodinfo or something" );
			System.Environment.Exit(0);
		}

		public Display( VoxelGameEnvironment game )
		{
			Exit = CannotExit;
			this.game = game;
			//string versionOpenGL = GL.GetString(StringName.Version);
			//GL.Get
			//display_width = Width;// = Settings.Read( "GL.Width", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Width );
			//display_height = Height;// = Settings.Read( "GL.Height", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Height );
			//mouse_x = display_width / 2;
			//mouse_y = display_height / 2;
			Width = 1080;
			Height = 1920;
#if !USE_GLES2
			System.Windows.Forms.Cursor.Position = new Point( display_x + ( display_width ) / 2, display_y + display_height / 2 );
#endif

			//GL.Viewport( 0, 0, Width, Height );
#if !USE_GLES2
			GL.Enable( EnableCap.Multisample );
#endif
			Matrix4.CreatePerspectiveFieldOfView( (float)( System.Math.PI / 2 ), (float)Width / (float)Height, 0.01f, 10000, out projection );
			//projection = Matrix4.Identity;

			// generic fly camera not attached to any object
			free_camera = new Voxels.UI.Camera();
			active_camera = free_camera; // default to freecam;
			free_camera.MoveTo( 0, 4, 0 );
			free_camera.MoveTo( 1799, 4, 0 );
			debug_cube_transform = btTransform.Identity;

		}

		public void Display_MouseUp( object sender, MouseButtonEventArgs e )
		{
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.MouseButtonRelease( e.Button, mouse_x = e.X * 2.0f / Width - 1, mouse_y = ( Height - (float)e.Y ) * 2.0f / Height - 1 );
		}

		public void Display_MouseDown( object sender, MouseButtonEventArgs e )
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
				del_time = 1.0f;
				float scale = 1;//100;
				if( del_mouse_x != 0 )
				{
					float delta = (float)( del_mouse_x * del_time ) * scale;
					#if BUILD_ANDROID
					free_camera.RotateYaw( -delta );
					#else
					free_camera.RotateYaw( delta );
					#endif
					del_mouse_x = 0;
				}
				if( del_mouse_y != 0 )
				{
					float delta = (float)( del_mouse_y * del_time ) * scale;
					#if BUILD_ANDROID
					free_camera.RotatePitch( -delta );
					#else
					free_camera.RotatePitch( delta );
					#endif
					del_mouse_y = 0;
				}
			}
			prior_mouse_time = time;
		}

		public void Display_MouseMove( object sender, MouseMoveEventArgs e )
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
#if !USE_GLES2
				System.Windows.Forms.Cursor.Position = new Point( ( X + Width ) / 2, ( Y + Height ) / 2 );
#endif
				_mouse_x = 0;
				_mouse_y = 0;
			}
			else
			{
				_mouse_x = mouse_x;
				_mouse_y = mouse_y;
			}
		}

		public void Display_MouseWheel( object sender, MouseWheelEventArgs e )
		{
			MouseButton button = e.Delta < 0 ? MouseButton.Button7 : MouseButton.Button6;
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.MouseButtonClick( button, mouse_x, mouse_y );

		}
#if BUILD_ANDROID
		bool touch_down;
		bool first_down;
		public bool Display_OnTouchEvent( MotionEvent e )
		{
			if( e.Action == MotionEventActions.Down
				|| e.Action != MotionEventActions.Up ) {
				if( !touch_down )
					first_down = true;
				else
					first_down = false;
				touch_down = true;
			} else {
				foreach( EventConsumer consumer in game.EventManager.ConsumerList )
					consumer.MouseButtonRelease( MouseButton.Button1, mouse_x, mouse_y );
				touch_down = false;
				return true; // nothing to compute
			}
			MotionEvent.PointerCoords coords = new MotionEvent.PointerCoords();
			/*
			Log.log( "Motion Events {0}  on display of {1} {2}", e.PointerCount, Width, Height );
			//if( e.EdgeFlags & Edge.Top )
			for( int n = 0; n < e.PointerCount; n++ ) {
				e.GetPointerCoords( n, coords );
				float x = coords.X;
				float y = coords.Y;
				float p = e.GetPressure( n );
				Log.log( "got event at {0},{1}  {2}", x, y, p );
			}
			*/
			e.GetPointerCoords( 0, coords );

			mouse_x = coords.X * 2.0f / Width - 1;
			mouse_y = ( Height - coords.Y ) * 2.0f / Height - 1;
			if( first_down ) {
				_mouse_x = mouse_x;
				_mouse_y = mouse_y;

				foreach( EventConsumer consumer in game.EventManager.ConsumerList )
					consumer.MouseButtonClick( MouseButton.Button1, mouse_x, mouse_y );
			} else {
				if( _mouse_x != -2 ) {
					del_mouse_x += mouse_x - _mouse_x;
					del_mouse_y += mouse_y - _mouse_y;
				}
				foreach (EventConsumer consumer in game.EventManager.ConsumerList)
					consumer.MouseMove( del_mouse_x, del_mouse_y, mouse_x, mouse_y );
				if( HiddenMouse ) {
					#if !USE_GLES2
		System.Windows.Forms.Cursor.Position = new Point( ( X + Width ) / 2, ( Y + Height ) / 2 );
					#endif
					//_mouse_x = 0;
					//_mouse_y = 0;
				} else {
					//_mouse_x = mouse_x;
					//_mouse_y = mouse_y;
				}
				_mouse_x = mouse_x;
				_mouse_y = mouse_y;
			}

			return true;
		}
#endif
		
		public void Display_SetDisplayParams( int x, int y, int width, int height )
		{
			//Log.log( "New display Parameters {0},{1}", width, height );
			display_width = Width = width;
			display_height = Height = height;
			Matrix4.CreatePerspectiveFieldOfView( (float)( System.Math.PI / 2 ), (float)Width / (float)Height, 0.01f, 10000, out projection );
			// so we can redo inventory box appropriately
			// otherwise it'll be badly mis-shapen on rotated screens.
			Aspect = (float)Width / (float)Height;
		}

		public void Display_KeyUp( object sender, KeyboardKeyEventArgs e )
		{
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.KeyDown( e.Key );
		}

		public void Display_KeyDown( object sender, KeyboardKeyEventArgs e )
		{
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.KeyDown( e.Key );
		}

		public void Display_Move( object sender, EventArgs e )
		{
			display_x = X;
			display_y = Y;
		}

		public void Display_Resize( object sender, EventArgs e )
		{
			display_width = Width;
			display_height = Height;
			Aspect = (float)Width / (float)Height;
			Matrix4.CreatePerspectiveFieldOfView( (float)( System.Math.PI / 2 ), (float)Width / (float)Height, 0.01f, 10000, out projection );
			GL.Viewport( 0, 0, Width, Height );
		}

#if BUILD_ANDROID
		// OnLoad doesn't exist in Android GameWindow events...
		bool loaded;
#endif
		public void Display_Load( object sender, EventArgs e )
		{
			//int val = GL.GetInteger( GetPName.CullFace );
			frequency = Stopwatch.Frequency / 1000.0f;
			sw.Start();
#if !USE_GLES2
			max_texture_size = GL.GetInteger( GetPName.MaxTextureSize );
			GL.GetInteger( GetPName.MaxTextureImageUnits, out max_texture_units );

			CheckErr();
			GL.Hint( HintTarget.PerspectiveCorrectionHint, HintMode.Nicest );
			CheckErr();
#else
			GL.GetInteger( GetPName.MaxTextureSize, out max_texture_size );
			CheckErr();
#endif
			GL.BlendFunc( BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha );

			//throw new NotImplementedException();
		}

		static bool HiddenMouse;
		public static void HideMouse()
		{
			HiddenMouse = true;
#if !BUILD_ANDROID
			System.Windows.Forms.Cursor.Hide();
#endif
		}
		public static void ShowMouse()
		{
			HiddenMouse = false;
#if !BUILD_ANDROID
			System.Windows.Forms.Cursor.Show();
#endif
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

		public delegate void SimpleMethod();
		public SimpleMethod Exit;
		//public SimpleMethod SwapBuffers;

		private void Shutdown()
		{
			if( !VoxelGlobalSettings.Exiting )
			{
				VoxelGlobalSettings.Exiting = true;
				if( AtExit != null )
					AtExit();

				Exit();
			}
		}

		public void Display_SetExit( object o, MethodInfo mi )
		{
			Delegate d = Delegate.CreateDelegate( typeof( SimpleMethod ), o, mi );
			Exit = (SimpleMethod)d;// mi.CreateDelegate( typeof(SimpleMethod), o );
		}

		public void Display_InvalidateContext( object sender, OpenTK.FrameEventArgs e )
		{
			if( OnInvalidate != null )
				OnInvalidate();
			// have to reload textures here...
		}

		public void Display_UpdateFrame( object sender, OpenTK.FrameEventArgs e )
		{
			if( !InitializedGame )
			{
				InitializedGame = true;
				Thread init = new Thread( InitGame );
				init.Start( game );
			}
			if( game_loaded ) // basic initialization done.
			{
				if( !game.DoUpdate( keyboard, e.Time ) )
				{
					Shutdown();
				}
			}
			
			ProcessMouseFreeCam( (float)e.Time );
			if( keyboard == null )
				return;
			if( keyboard[Key.AltLeft] && keyboard[Key.F4] )
			{
				Shutdown();
			}
			if( keyboard[Key.Escape] )
			{
				Shutdown();
			}
			if( keyboard[Key.G] )
				VoxelReactor.StepOne = true;
			if( keyboard[Key.X] && !keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.x += 0.2f;
			if( keyboard[Key.X] && keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.x -= 0.2f;
			if( keyboard[Key.Z] && !keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.z += 0.2f;
			if( keyboard[Key.Z] && keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.z -= 0.2f;
			if( keyboard[Key.Y] && !keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.y += 0.2f;
			if( keyboard[Key.Y] && keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_origin.y -= 0.2f;

			if( keyboard[Key.R] && !keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 2, 0.1f );
			if( keyboard[Key.R] && keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 2, -0.1f );

			if( keyboard[Key.T] && !keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 1, 0.1f );
			if( keyboard[Key.T] && keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 1, -0.1f );

			if( keyboard[Key.P] && !keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 0, 0.1f );
			if( keyboard[Key.P] && keyboard[Key.ShiftLeft] )
				debug_cube_transform.m_basis.Rotate( 0, -0.1f );

			if( keyboard[Key.Space] )
				free_camera.MoveUp( 10 * (float)e.Time );
			if( keyboard[Key.AltLeft] )
				free_camera.MoveUp( -10 * (float)e.Time );
			if( keyboard[Key.W] )
			{
				free_camera.MoveForward( 40 * (float)e.Time );
			}
			if( keyboard[Key.S] )
			{
				free_camera.MoveForward( -10 * (float)e.Time );
			}
			if( keyboard[Key.A] )
			{
				free_camera.MoveRight( -10 * (float)e.Time );
			}
			if( keyboard[Key.D] )
			{
				free_camera.MoveRight( 10 * (float)e.Time );
			}
			if( keyboard[Key.Q] )
			{
				free_camera.RotateRoll( -2f * (float)e.Time );
			}
			if( keyboard[Key.E] )
			{
				free_camera.RotateRoll( 2f * (float)e.Time );
			}

			if( !initialized )
			{
				initialized = true;
			}
			if( game.VoxelProcessor != null )
				game.VoxelProcessor.SetPlayerPosition( game.World, ref free_camera.location.m_origin );
        }

		static bool blending_enabled;
		internal static void EnableBlending( bool enable )
		{
			if( blending_enabled != enable )
			{
				if( enable )
					GL.Enable( EnableCap.Blend );
				else
					GL.Disable( EnableCap.Blend );
				blending_enabled = enable;
			}
		}

		void InitGame( object o )
		{
			VoxelGameEnvironment game = o as VoxelGameEnvironment;
			if( !game.Init() )
			{
				Shutdown();
			}
			else
			{
				game.Basic_Renderer.Camera = free_camera;
				game_loaded = true;
			}

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
		public void Display_RenderFrame( object sender, OpenTK.FrameEventArgs e )
		{
#if BUILD_ANDROID
			if( !loaded ) {
				loaded = true;
				Display_Load( sender, e );
			}
#endif
			// render graphics
			frame++;
			GL.Clear( ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit );
			if( !game_loaded )
			{
				DrawProgress();
				return;
			}

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
			DrawDebugCube();
			if( game.TileSetStyles != null ) {
				TileSet.TileStyle style = game.TileSetStyles.GetStyle( 2 );
				Box box = new Box();
				Vector4 DrawColor = new Vector4( 1 );
				box.Size.X = 1;
				box.Size.Y = 1;
				box.Size.Z = 0;
				Vector3 u; free_camera.location.GetRight( out u );

				//( Vector3.UnitZ + Vector3.UnitY ).Normalized();
				Vector3 v;  free_camera.location.GetUp( out v );
				//( -Vector3.UnitZ + Vector3.UnitY ).Normalized(); ;
				Vector2 string_size;
				game.default_font.GetFontRenderSize( "-5 X", 1, out string_size );
				box.Position.X = -5;
				box.Position.Y = 0;
				box.Position.Z = 0;
				box.Position -= u * string_size.X/2 + v * string_size.Y/2;
				game.default_font.RenderFont( this, ref box, ref u, ref v, 1, "-5 X", ref DrawColor );
				box.Position.X = 5;
				box.Position.Y = 0;
				box.Position.Z = 0;
				box.Position -= u * string_size.X / 2 + v * string_size.Y / 2;
				game.default_font.RenderFont( this, ref box, ref u, ref v, 1, "5 X", ref DrawColor );

				game.default_font.GetFontRenderSize( "5 X", 1, out string_size );
				box.Position.X = 0;
				box.Position.Y = -5;
				box.Position.Z = 0;
				box.Position -= u * string_size.X / 2 + v * string_size.Y / 2;
				game.default_font.RenderFont( this, ref box, ref u, ref v, 1, "-5 Y", ref DrawColor );
				box.Position.X = 0;
				box.Position.Y = 5;
				box.Position.Z = 0;
				box.Position -= u * string_size.X / 2 + v * string_size.Y / 2;
				game.default_font.RenderFont( this, ref box, ref u, ref v, 1, "5 Y", ref DrawColor );
				box.Position.X = 0;
				box.Position.Y = 0;
				box.Position.Z = -5;
				box.Position -= u * string_size.X / 2 + v * string_size.Y / 2;
				game.default_font.RenderFont( this, ref box, ref u, ref v, 1, "-5 Z", ref DrawColor );
				box.Position.X = 0;
				box.Position.Y = 0;
				box.Position.Z = 5;
				box.Position -= u * string_size.X / 2 + v * string_size.Y / 2;
				game.default_font.RenderFont( this, ref box, ref u, ref v, 1, "5 Z", ref DrawColor );
			}
			//Log.log( " Origin is " + free_camera.location.m_origin );
			Display.CheckErr();

			BEPUDebugDrawer.DrawSpace( this, game.Engine );
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
			game.Draw( this, sw.ElapsedTicks / frequency );
			//SwapBuffers();
		}

		internal static float SclX( float x ) { return x * 2.0f / 1024f; }
		internal static float SclY( float y ) { return y * 2.0f / 768f; }

		[Conditional( "DEBUG" )]
		internal static void CheckErr()
		{
#if !USE_GLES2
			ErrorCode code = GL.GetError();
#else
			ErrorCode code = GL.GetErrorCode( );
#endif
			if( code != 0 )
			{
				Log.log( "error " + code, 1  );
				//return true;
			}
			//return false;
		}

		internal static Bitmap LoadBitmap( int location, string file )
		{
#if USE_GLES2
			if( location == 0 )
			{
				//Xamarin.Forms FileImageSource
				Stream s = Application.Context.Assets.Open( file );
				Bitmap bitmap = BitmapFactory.DecodeStream( s );
				s.Dispose();
				return bitmap;
			}
			else
				return BitmapFactory.DecodeFile( file );
#else
			return new Bitmap( file );
#endif
		}
		internal static Dictionary<string,string[]> paths = new Dictionary<string, string[]>();
		internal static bool FileExists( string file, out int location )
		{
#if BUILD_ANDROID
			string path, fileName;
			if( file.Contains( "/" ) )
			{
				path = file.Substring( 0, file.LastIndexOf( '/' ) );
				fileName = file.Substring( file.LastIndexOf( '/' ) + 1 );
			}
			else
			{
				path = "";
				fileName = file;
			}
			String[] assets;
			if( paths.ContainsKey( path ) )
				assets = paths[path];
			else
			{
				assets = Application.Context.Assets.List( path );
				paths.Add( path, assets );
			}
			//Log.log( "Test exists: " + path + " and " + fileName );
			foreach( string s in assets )
				if( String.Compare( s, fileName ) == 0 )
				{
					location = 0;
					return true;
				}
			if( File.Exists( file ) )
			{
				location = 1;
				return true;
			}
			location = 0;
			return false;
#else
			location = 0;
			return File.Exists( file );
#endif
		}
		internal static StreamReader FileOpenText( int location, string file )
		{
#if BUILD_ANDROID
			//Log.log( "can open " + file );
			if( location == 0 )
				return new StreamReader( Application.Context.Assets.Open( file ) );
			else
#endif
				return File.OpenText( file );
		}
		internal static byte[] FileReadAllBytes( int location, string file )
		{
#if BUILD_ANDROID
			//Log.log( "can open " + file );
			if( location == 0 )
			{
				int totlength = 0;
				int length ;
				byte[] tmp = new byte[4096];
				Stream s = Application.Context.Assets.Open( file );
				while( ( length = s.Read( tmp, 0, 4096 ) ) > 0 )
					totlength += length;
				s.Close();
				tmp = new byte[totlength];
				// have to re-open to seek 0
				s = Application.Context.Assets.Open( file );
				//s.Seek( 0, SeekOrigin.Begin );
				s.Read( tmp, 0, totlength );				
				s.Close();
				return tmp;
			}
			else
#endif
			{
				//Log.log( "Location is " + location + " " + file);
				byte[] bytes= File.ReadAllBytes( file );
				//Log.log( "Return " + bytes.Length );
				return bytes;
				}
		}
	}
}
