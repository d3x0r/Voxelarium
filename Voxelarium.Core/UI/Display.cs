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
using Bullet.LinearMath;
using System.Threading;
using Voxelarium.Core.Game.Screens;
using Voxelarium.Core.Voxels;
using System.Drawing.Imaging;
using Voxelarium.Core.Support;

namespace Voxelarium.Core.UI
{
	public class Display : OpenTK.GameWindow
	{
		Voxelarium.Core.VoxelGameEnvironment game;

		internal static int max_texture_size;
		internal static Matrix4 modelview;
		//internal static Matrix4 worldview;

		internal static Matrix4 projection;
		internal static btTransform worldview;
		MouseDevice mouse;
		bool initialized;

		float _mouse_x = -2, _mouse_y;
		float mouse_x, mouse_y;
		float del_mouse_x, del_mouse_y;
		int display_width;
		int display_height;
		int display_x;
		int display_y;
		internal SimpleShader simple = new SimpleShader();
		internal SimpleGuiShader simple_gui = new SimpleGuiShader();
		internal SimpleEdgeShader edge = new SimpleEdgeShader();
		internal ColorEdgeShader color_edge = new ColorEdgeShader();
		internal SimpleTextureShader simple_texture = new SimpleTextureShader();
		internal SimpleGuiTextureShader simple_gui_texture = new SimpleGuiTextureShader();

		internal static List<Shaders.Shader> used_shaders = new List<Shaders.Shader>();
		internal List<Shaders.Shader> shaders = new List<Shaders.Shader>();

		static bool InitializedGame;
		static bool game_loaded;
		static bool textures_loaded;

		public Display( VoxelGameEnvironment game ) : base()
		{
			this.game = game;

			mouse = Mouse;
			this.WindowBorder = OpenTK.WindowBorder.Hidden;
			display_width = Width = Settings.Read( "GL.Width", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Width );
			display_height = Height = Settings.Read( "GL.Height", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Height );
			mouse_x = display_width / 2;
			mouse_y = display_height / 2;
			display_x = X = Settings.Read( "GL.Display.X", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.X );
			display_y = Y = Settings.Read( "GL.Display.Y", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Y );
			System.Windows.Forms.Cursor.Position = new Point( display_x + ( display_width ) / 2, display_y + display_height / 2 );
			UpdateFrame += Display_UpdateFrame;
			RenderFrame += Display_RenderFrame;
			GL.Viewport( 0, 0, Width, Height );
			Matrix4.CreatePerspectiveFieldOfView( (float)( System.Math.PI / 2 ), (float)Width / (float)Height, 0.01f, 10000, out projection );
			//projection = Matrix4.Identity;

			//worldview = Matrix4.Identity;
			worldview = btTransform.Identity;
			modelview = Matrix4.Identity;
			worldview.Translate( 1, 2, -10 );
			//worldview = worldview * tmp;
			//worldview = Matrix4.

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
				consumer.MouseButtonRelease( e.Button, mouse_x = e.X * 2.0f / Width - 1, mouse_y=( Height - (float)e.Y ) * 2.0f / Height - 1 );
		}

		private void Display_MouseDown( object sender, MouseButtonEventArgs e )
		{
			foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.MouseButtonClick( e.Button, mouse_x = e.X * 2.0f / Width - 1, mouse_y = ( Height - (float)e.Y ) * 2.0f / Height - 1 );
		}

		private void Display_MouseMove( object sender, MouseMoveEventArgs e )
		{
			mouse_x = e.X * 2.0f / Width - 1;
			mouse_y = ( Height - (float)e.Y ) * 2.0f / Height - 1;
            foreach( EventConsumer consumer in game.EventManager.ConsumerList )
				consumer.MouseMove( del_mouse_x, del_mouse_y, mouse_x, mouse_y );
			if( _mouse_x != -2 )
			{
				del_mouse_x = mouse_x - _mouse_x;
				del_mouse_y = mouse_y - _mouse_y;
			}
			_mouse_x = mouse_x;
			_mouse_y = mouse_y;
			if( HiddenMouse )
			{
				mouse_x = display_width / 2;
				mouse_y = display_height / 2;
				System.Windows.Forms.Cursor.Position = new Point( display_x + ( display_width ) / 2, display_y + display_height / 2 );
			}
		}

		private void Display_MouseWheel( object sender, MouseWheelEventArgs e )
		{
			int abs = e.Delta < 0 ? -e.Delta : e.Delta;
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
			//GL.Enable( EnableCap.DepthTest );
			//GL.GetInteger(GetIndexedPName.
			max_texture_size = GL.GetInteger( GetPName.MaxTextureSize ); 

			GL.Enable( EnableCap.Texture2D );
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
					Exit();
				}
			}
			{
				//Console.WriteLine( "tick " + e.Time );
				float deltime = (float)e.Time;
				int x = mouse.X;
				int y = mouse.Y;
				//mouse. = 500;
				//mouse.Y = 500;
			}
			if( del_mouse_x != 0 )
			{
				float delta = (float)(del_mouse_x * e.Time) * 200;
				worldview.m_basis.Rotate( 0, -delta, 0 );
				del_mouse_x = 0;
			}
			if( del_mouse_y != 0 )
			{
				float delta = (float)( del_mouse_y * e.Time )*200;
				worldview.m_basis.Rotate( -delta, 0, 0 );
				del_mouse_y = 0;
			}



			if( Keyboard[Key.AltLeft] && Keyboard[Key.F4] )
			{
				VoxelGlobalSettings.Exiting = true;
				Exit();
			}
			if( Keyboard[Key.Escape] )
			{
				VoxelGlobalSettings.Exiting = true;
				Exit();
			}

			if( Keyboard[Key.W] )
			{
				btVector3 forward; worldview.m_basis.getColumn( 2 ).Mult( 50 * (float)e.Time, out forward );
				worldview.Move( forward.x, forward.y, forward.z );
			}
			if( Keyboard[Key.S] )
			{
				btVector3 forward; worldview.m_basis.getColumn( 2 ).Mult( 50 * (float)e.Time, out forward );
				worldview.Move( -forward.x, -forward.y, -forward.z );
			}
			if( Keyboard[Key.A] )
			{
				btVector3 right; worldview.m_basis.getColumn( 0 ).Mult( 50 * (float)e.Time, out right );
				worldview.Move( right.x, right.y, right.z );
			}
			if( Keyboard[Key.D] )
			{
				btVector3 right; worldview.m_basis.getColumn( 0 ).Mult( 50 * (float)e.Time, out right );
				worldview.Move( -right.x, -right.y, -right.z );
			}
			if( Keyboard[Key.Q] )
			{
				worldview.m_basis.Rotate( 2, -2f * (float)e.Time );
			}
			if( Keyboard[Key.E] )
			{
				worldview.m_basis.Rotate( 2, 2f * (float)e.Time );
			}

			if( !initialized )
			{
				initialized = true;
			}
		}

		void InitGame( object o )
		{
			VoxelGameEnvironment game = o as VoxelGameEnvironment;
			if( !game.Init() )
			{
				VoxelGlobalSettings.Exiting = true;
				Exit();
			}
			game_loaded = true;

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
			if( !textures_loaded )
			{
				LoadVoxelTexturesToGPU();
				//LoadTexturesToGPU();
				textures_loaded = true;
			}
			game.Draw( this );

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
			//Vector4 face_color = new Vector4( 1f, 1f, 1f, 1 );
			edge.Activate();
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

			GL.UseProgram( 0 );


			Display.CheckErr();
#if !USE_GLES2
			GL.MatrixMode( MatrixMode.Projection );
			GL.LoadMatrix( ref projection );

			GL.MatrixMode( MatrixMode.Modelview );
			unsafe
			{
				btMatrix3x3 tmp;
				worldview.GetGLMatrix( out tmp );
				//Console.WriteLine( worldview.ToString() );
				//Console.WriteLine( tmp.ToString() );
				float* matrix_ptr = &tmp.m_el0.x;
				{
					GL.LoadMatrix( matrix_ptr );
				}
			}
			//GL.BindTexture( TextureTarget.Texture2D, frame %100 );
			GL.Begin( BeginMode.Triangles );
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

		bool LoadVoxelTexturesToGPU()
		{
			int i;
			VoxelType VoxelType;
			GL.UseProgram( 0 );
			GL.Enable( EnableCap.Texture2D );
			GL.ActiveTexture( TextureUnit.Texture0 );
			for( i = 0; i < 65536; i++ )
			{
				if( !( VoxelType = game.VoxelTypeManager.VoxelTable[i] ).properties.Is_NoType )
				{
					if( VoxelType.MainTexture != null )
					{
                        GL.GenTextures( 1, out VoxelType.OpenGl_TextureRef );
						GL.BindTexture( TextureTarget.Texture2D, VoxelType.OpenGl_TextureRef );
						CheckErr();
						int param = (int)TextureMinFilter.NearestMipmapLinear;
						GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Nearest ); // GL_LINEAR GL_NEAREST
						CheckErr();
						param = (int)TextureMagFilter.Linear;
						//GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, param );
						CheckErr();
						// if (i & 1) glTexParameteri(GL_TEXTURE_2D, 0x84FE /*TEXTURE_MAX_ANISOTROPY_EXT*/, 8);
						//GL.TexParameterI( TextureTarget.Texture2D, TextureParameterName. 0x84FE /*TEXTURE_MAX_ANISOTROPY_EXT*/, 8 );
						BitmapData data = VoxelType.MainTexture.LockBits(
							new Rectangle( 0, 0, VoxelType.MainTexture.Width, VoxelType.MainTexture.Height )
							, System.Drawing.Imaging.ImageLockMode.ReadOnly
							, VoxelType.MainTexture.PixelFormat );
#if USE_GLES2
						GL.TexImage2D( TextureTarget2d.Texture2D, 0, TextureComponentCount.Rgba
							, data.Width, data.Height
							, 0, OpenTK.Graphics.ES20.PixelFormat.Rgba
							, PixelType.UnsignedByte
							, data.Scan0
							);
#else
						GL.TexImage2D( TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba
							, data.Width, data.Height
							, 0, OpenTK.Graphics.OpenGL.PixelFormat.Bgra
							, PixelType.UnsignedByte
							, data.Scan0
							);
#endif
						CheckErr();
						VoxelType.MainTexture.UnlockBits( data );

						//glTexEnvf(0x8500 /* TEXTURE_FILTER_CONTROL_EXT */, 0x8501 /* TEXTURE_LOD_BIAS_EXT */,3.0);
						// if ((i & 1) ) glTexEnvf(0x8500 /* TEXTURE_FILTER_CONTROL_EXT */, 0x8501 /* TEXTURE_LOD_BIAS_EXT */,-4.25);

						//glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
					}
				}
			}

			return ( true );
		}


		[Conditional( "DEBUG" )]
		internal static void CheckErr()
		{
			ErrorCode code = GL.GetError();
			if( code != 0 )
			{
				StackTrace st = new StackTrace();
				StackFrame sf = st.GetFrame( 3 );
				Console.WriteLine( "error " + code + sf.GetFileName() + "(" + sf.GetFileLineNumber() + ")" );
			}
		}
	}
}
