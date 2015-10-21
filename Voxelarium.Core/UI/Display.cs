using OpenTK.Graphics.OpenGL;
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

namespace Voxelarium.Core.UI
{
    public class Display: OpenTK.GameWindow
    {
		EventManager Events = new EventManager();
		ScreenManager screens = new ScreenManager();


		GameScreen activeScreen;
		internal static Matrix4 modelview;
		//internal static Matrix4 worldview;

		internal static Matrix4 projection;
		internal static btMatrix3x3 worldview;
		MouseDevice mouse;
		bool initialized;

		int mouse_x, mouse_y;
		int display_width;
		int display_height;
		int display_x;
		int display_y;
		SimpleShader simple = new SimpleShader();
		SimpleEdgeShader edge = new SimpleEdgeShader();
		ColorEdgeShader color_edge = new ColorEdgeShader();

		internal static List<Shaders.Shader> used_shaders = new List<Shaders.Shader>();
		internal List<Shaders.Shader> shaders = new List<Shaders.Shader>();

		public Display():base()
        {
			mouse = Mouse;
            this.WindowBorder = OpenTK.WindowBorder.Hidden;
			display_width = Width = Settings.Read( "GL.Width", Screen.PrimaryScreen.WorkingArea.Width );
			display_height = Height = Settings.Read( "GL.Height", Screen.PrimaryScreen.WorkingArea.Height );
			mouse_x = display_width / 2;
			mouse_y = display_height / 2;
			display_x = X = Settings.Read( "GL.Display.X", Screen.PrimaryScreen.WorkingArea.X );
			display_y = Y = Settings.Read( "GL.Display.Y", Screen.PrimaryScreen.WorkingArea.Y );
			System.Windows.Forms.Cursor.Position = new Point( display_x + ( display_width ) / 2, display_y + display_height / 2 );
			UpdateFrame += Display_UpdateFrame;
            RenderFrame += Display_RenderFrame;
			GL.Viewport( 0, 0, Width, Height );
			Matrix4.CreatePerspectiveFieldOfView( (float)(Math.PI / 2), (float)Width / (float)Height, 0.01f, 10000, out projection );
			//projection = Matrix4.Identity;

			//worldview = Matrix4.Identity;
			worldview = btMatrix3x3.Identity;
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
			foreach( EventConsumer consumer in Events.ConsumerList )
				consumer.MouseButtonRelease( e.Button, e.X, e.Y );
		}

		private void Display_MouseDown( object sender, MouseButtonEventArgs e )
		{
			foreach( EventConsumer consumer in Events.ConsumerList )
				consumer.MouseButtonClick( e.Button, e.X, e.Y );
		}

		private void Display_MouseMove( object sender, MouseMoveEventArgs e )
		{
			foreach( EventConsumer consumer in Events.ConsumerList )
				consumer.MouseMove( e.XDelta, e.YDelta, e.X, e.Y);
		}

		private void Display_MouseWheel( object sender, MouseWheelEventArgs e )
		{
			int abs = e.Delta < 0 ? -e.Delta : e.Delta;
			MouseButton button = e.Delta < 0 ? MouseButton.Button7 : MouseButton.Button6;
			foreach( EventConsumer consumer in Events.ConsumerList )
				consumer.MouseButtonClick( button, mouse_x, mouse_y );

		}

		private void Display_KeyUp( object sender, KeyboardKeyEventArgs e )
		{
			foreach( EventConsumer consumer in Events.ConsumerList )
				consumer.KeyDown( e.Key );
		}

		private void Display_KeyDown( object sender, KeyboardKeyEventArgs e )
		{
			foreach( EventConsumer consumer in Events.ConsumerList )
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
			//throw new NotImplementedException();
		}

		private void Display_UpdateFrame( object sender, OpenTK.FrameEventArgs e )
		{
			{
				//Console.WriteLine( "tick " + e.Time );
				float deltime = (float)e.Time;
				int x = mouse.X;
				int y = mouse.Y;
				if( x != mouse_x )
				{
					float delta = ( x - mouse_x );
					delta = (float)(( delta / 6 ) * e.Time );
					worldview.Rotate( 0, -delta, 0 );
					mouse_x = x;
				}
				if( y != mouse_y )
				{
					float delta = ( y - mouse_y );
					delta = (float)( ( delta / 6 ) * e.Time );
					worldview.Rotate( delta, 0, 0 );
					mouse_y = y;
				}
				mouse_x = display_width/2;
				mouse_y = display_height/2;
				System.Windows.Forms.Cursor.Position = new Point( display_x+ (display_width )/2, display_y+display_height/2 );
				//mouse. = 500;
				//mouse.Y = 500;
			}


			if( Keyboard[Key.AltLeft] && Keyboard[Key.F4] )
				Exit();
            if( Keyboard[Key.Escape] )
            {
                Exit();
            }

			if( Keyboard[Key.W] )
			{
				btVector3 forward; worldview.getColumn( 2 ).Mult( 50 * (float)e.Time, out forward );
				worldview.Move( forward.x, forward.y, forward.z );
			}
			if( Keyboard[Key.S] )
			{
				btVector3 forward; worldview.getColumn( 2 ).Mult( 50 * (float)e.Time, out forward );
				worldview.Move( -forward.x, -forward.y, -forward.z );
			}
			if( Keyboard[Key.A] )
			{
				btVector3 right; worldview.getColumn( 0 ).Mult( 50 * (float)e.Time, out right );
				worldview.Move( right.x, right.y, right.z );
			}
			if( Keyboard[Key.D] )
			{
				btVector3 right; worldview.getColumn( 0 ).Mult( 50 * (float)e.Time, out right );
				worldview.Move( -right.x, -right.y, -right.z );
			}
			if( Keyboard[Key.Q] )
			{
				worldview.Rotate( 2, -2f * (float)e.Time );
			}
			if( Keyboard[Key.E] )
			{
				worldview.Rotate( 2, 2f * (float)e.Time );
			}

			if( !initialized )
			{
				initialized = true;
			}
        }


		void DoGameDraw

        private void Display_RenderFrame( object sender, OpenTK.FrameEventArgs e )
        {
            // render graphics

            GL.Clear( ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit );
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
			edge.Activate();
			Display.CheckErr();
			edge.DrawQuad( verts, text, ref color );

			for( int i = 0; i < 4; i++ )
			{
				text[i].X *= 2;
				text[i].Y *= 2;
				verts[i].X += 10;
			}
			color = new Vector4( 0.0f, 0.5f, 0.9f, 1 );
			edge.Activate();
			Display.CheckErr();
			edge.DrawQuad( verts, text, ref color );

			for( int i = 0; i < 4; i++ )
			{
				text[i].X *= 2;
				text[i].Y *= 2;
				verts[i].X += 10;
			}
			color = new Vector4( 0.0f, 0.5f, 0.9f, 1 );
			color_edge.Activate();
			Display.CheckErr();
			color_edge.DrawQuad( verts, text, colors, 800 );
			//edge.DrawQuad( verts, ref color );
			Display.CheckErr();

			GL.UseProgram( 0 );
			Display.CheckErr();
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
			GL.Begin( BeginMode.Triangles );

			GL.Color3( 1.0f, 1.0f, 0.0f ); GL.Vertex3( -1.0f, -1.0f, 4.0f );
			GL.Color3( 1.0f, 0.0f, 0.0f ); GL.Vertex3( 1.0f, -1.0f, 4.0f );
			GL.Color3( 0.2f, 0.9f, 1.0f ); GL.Vertex3( 0.0f, 1.0f, 4.0f );

			GL.End();

			SwapBuffers();
        }

		internal static void CheckErr()
		{
			ErrorCode code = GL.GetError();
			if( code != 0 )
			{
				StackTrace st = new StackTrace();
				StackFrame sf = st.GetFrame( 3 );
				Console.WriteLine( "error " + code + sf.GetFileName() + "("+ sf.GetFileLineNumber() +")");
			}
		}
    }
}
