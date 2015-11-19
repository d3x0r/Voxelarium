using OpenTK.Input;
using OpenTK;
using Bullet.LinearMath;
using OpenTK.Graphics.OpenGL;
using OpenTK.Graphics;
using System;

namespace Bullet.Debug.OpenGL2
{
	public interface ITest
	{
		void Reset();
	}

	public class OpenGLDisplay : OpenTK.GameWindow
	{
		public delegate void DoTick();
		public event DoTick Tick;

		BulletDebugDrawer drawer;

		internal Matrix4 projection;
		public ITest Test;
		btTransform free_camera = btTransform.Identity;
		float[] m = new float[16];
		bool HiddenMouse;

		public OpenGLDisplay() : base()
		{
			this.Load += OpenGLDisplay_Load; ;
			this.Resize += OpenGLDisplay_Resize; ;
			this.Move += OpenGLDisplay_Move; ;
			UpdateFrame += OpenGLDisplay_UpdateFrame;
			RenderFrame += OpenGLDisplay_RenderFrame;
			MouseMove += OpenGLDisplay_MouseMove;
			MouseDown += OpenGLDisplay_MouseDown;
			MouseUp += OpenGLDisplay_MouseUp;
		}

		
		private void OpenGLDisplay_MouseDown( object sender, MouseButtonEventArgs e )
		{
			_mouse_x = 0;
			_mouse_y = 0;
			System.Windows.Forms.Cursor.Position = new System.Drawing.Point( X + ( Width ) / 2, Y + Height / 2 );
			HiddenMouse = true;
		}
		private void OpenGLDisplay_MouseUp( object sender, MouseButtonEventArgs e )
		{
			HiddenMouse = false;
		}

		private void OpenGLDisplay_RenderFrame( object sender, FrameEventArgs e )
		{

			GL.Clear( ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit );

			//Console.WriteLine( "Camera Matrix ------------- \n{0}", free_camera.ToString( ) );
			free_camera.GetGLCameraMatrix( m );
			GL.LoadMatrix( m );

			if( Tick != null )
				Tick();
			GL.Begin( BeginMode.Triangles );
			GL.TexCoord2( 0, 1 );
			GL.Color3( 1.0f, 1.0f, 0.0f ); GL.Vertex3( -1.0f, -1.0f, 4.0f );
			GL.TexCoord2( 1, 1 );
			GL.Color3( 1.0f, 0.0f, 0.0f ); GL.Vertex3( 1.0f, -1.0f, 4.0f );
			GL.TexCoord2( 1, 0 );
			GL.Color3( 0.2f, 0.9f, 1.0f ); GL.Vertex3( 0.0f, 1.0f, 4.0f );
			GL.End();

			SwapBuffers();
			//throw new System.NotImplementedException();
		}

		private void OpenGLDisplay_UpdateFrame( object sender, FrameEventArgs e )
		{
			HandleInput( e.Time );
			//Console.WriteLine( "time is " + e.Time );
			//throw new System.NotImplementedException();
		}

		private void OpenGLDisplay_Move( object sender, System.EventArgs e )
		{
			//throw new System.NotImplementedException();
		}

		private void OpenGLDisplay_Resize( object sender, System.EventArgs e )
		{
			//throw new System.NotImplementedException();
			GL.Viewport( 0, 0, Width, Height );
		}

		private void OpenGLDisplay_Load( object sender, System.EventArgs e )
		{
			GL.ClearColor( Color4.DarkGray );

			Matrix4.CreatePerspectiveFieldOfView( (float)( System.Math.PI / 2 ), (float)Width / (float)Height, 0.01f, 10000, out projection );
			GL.MatrixMode( MatrixMode.Projection );
			GL.LoadMatrix( ref projection );
			GL.MatrixMode( MatrixMode.Modelview );

			free_camera.MoveTo( 0, 4, -10 );
			//throw new System.NotImplementedException();
		}

		public btIDebugDraw GetDrawer()
		{
			if( drawer == null )
				drawer = new BulletDebugDrawer();
			return drawer;
		}

		void HandleInput( double time )
		{
			ProcessMouseFreeCam( (float)time );
			if( Keyboard[Key.AltLeft] && Keyboard[Key.F4] )
			{
				//Shutdown();
			}
			if( Keyboard[Key.Escape] )
			{
				//Shutdown();
			}


			if( Keyboard[Key.Space] )
				free_camera.MoveUp( 10 * (float)time );
			if( Keyboard[Key.AltLeft] )
				free_camera.MoveUp( -10 * (float)time );
			if( Keyboard[Key.W] )
			{
				free_camera.MoveForward( 2 * (float)time );
			}
			if( Keyboard[Key.S] )
			{
				free_camera.MoveForward( -2 * (float)time );
			}
			if( Keyboard[Key.A] )
			{
				free_camera.MoveRight( -2 * (float)time );
			}
			if( Keyboard[Key.D] )
			{
				free_camera.MoveRight( 2 * (float)time );
			}
			if( Keyboard[Key.Q] )
			{
				free_camera.RotateRoll( -2f * (float)time );
			}
			if( Keyboard[Key.E] )
			{
				free_camera.RotateRoll( 2f * (float)time );
			}
			if( Keyboard[Key.R] )
			{
				Test.Reset();
			}
		}

		float _mouse_x = -2, _mouse_y;
		float mouse_x, mouse_y;
		float del_mouse_x, del_mouse_y;
		float prior_mouse_time;
		float del_time;

		void ProcessMouseFreeCam( float time )
		{
			//float del_time;
			if( prior_mouse_time != 0 )
			{
				del_time += time;
				if( del_mouse_x != 0 || del_mouse_y != 0 )
				{
					//Console.WriteLine( "---------------------------------------" );
					//Console.WriteLine( "Mouse move {0},{1} {2} {3}", del_mouse_x, del_mouse_y, del_time, time );
					if( del_mouse_x != 0 )
					{
						float delta = (float)( del_mouse_x /** del_time*/ ) ;
						//Console.WriteLine( "Mouse move Yaw {0}", delta );
						free_camera.RotateYaw( delta );
						del_mouse_x = 0;
					}
					if( del_mouse_y != 0 )
					{
						float delta = (float)( del_mouse_y /** del_time*/ ) ;
						//Console.WriteLine( "Mouse move Pitch {0}", delta );
						free_camera.RotatePitch( delta );
						del_mouse_y = 0;
					}
					del_time = 0;
				}
			}
			else
				prior_mouse_time = time;
		}

	private void OpenGLDisplay_MouseMove( object sender, MouseMoveEventArgs e )
	{
			int eX = System.Windows.Forms.Cursor.Position.X - this.Bounds.X;
			int eY = System.Windows.Forms.Cursor.Position.Y - this.Bounds.Y;
			//Console.WriteLine( "Mouse move {0},{1}", eX, eY );
			mouse_x = eX * 2.0f / ClientSize.Width - 1;
			mouse_y = ( ClientSize.Height - (float)eY ) * 2.0f / ClientSize.Height - 1;
			//Console.WriteLine( "Mouse on screen is {0} {1}  (old {2} {3} )", mouse_x, mouse_y, _mouse_x, _mouse_y );
			if( _mouse_x != -2 )
			{
				del_mouse_x += mouse_x - _mouse_x;
				del_mouse_y += mouse_y - _mouse_y;
				//Console.WriteLine( "Mouse move {0},{1}", del_mouse_x, del_mouse_y );
			}
			if( HiddenMouse )
			{
				System.Windows.Forms.Cursor.Position = new System.Drawing.Point( X + ( Width ) / 2, Y + Height / 2 );
				_mouse_x = 0;
				_mouse_y = 0;
			}
			else
			{
				_mouse_x = mouse_x;
				_mouse_y = mouse_y;
			}
		}

	}
}
